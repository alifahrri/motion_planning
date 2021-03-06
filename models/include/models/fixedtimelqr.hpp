#ifndef FIXEDTIMELQR_HPP
#define FIXEDTIMELQR_HPP

#include <vector>
#include <tuple>
#include <math.h>
#include <utility>
#include <type_traits>
#include "rootfinder.hpp"

#ifdef GPU
#include <cuda.h>
#include <cuda_runtime.h>
#ifndef gpuErrchk
#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
  if (code != cudaSuccess)
  {
    fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
    if (abort) exit(code);
  }
}
#endif
#define ATTRIBUTE __host__ __device__
#else
#define ATTRIBUTE
#endif

template
<
    // should be a class that has 'set' function
    // with (const State&, const State&) signature and no return
    // and overloads operator () with (numeric, numeric)->numeric signature
    // basically map R^{1} to R^{1} space, given initial & final state
    class FN
    >
// find optimal arrival time numerically, given the derivative of cost function
class OptimalTimeFinder
{
public:
  ATTRIBUTE
  OptimalTimeFinder(FN& f) : f(f) {}
  template<typename scalar=double, typename State>
  ATTRIBUTE
  inline
  scalar solve(const State &x0, const State &xf, const scalar &t1 = scalar(1e-2),
               const scalar &t0 = scalar(1e-3), const scalar &e = scalar(1e-6)) const
  {
    RootFinder::SecantMethod<FN> root_finder(f);
    f.set(x0,xf);
    auto optimal_time = root_finder(t1, t0, e);
    auto off_t1 = 5e-1;
    auto off_t0 = 5e-1;
    while((optimal_time < 0)
          || isnan(optimal_time)
          || isinf(optimal_time)
          // || f(optimal_time > e)
          )
    {
      optimal_time = root_finder(t1 + off_t1, t0 + off_t0, e);
      off_t1 += 5e-1;
      off_t0 += 5e-1;
    }
    return optimal_time;
  }
  template<typename scalar=double, typename State>
  ATTRIBUTE
  inline
  scalar operator()(const State &x0, const State &xf,
                    const scalar &t1 = scalar(1e-2), const scalar &t0 = scalar(1e-3),
                    const scalar &e = scalar(1e-6)) const
  {
    return this->solve(x0,xf,t1,t0,e);
  }
  FN &f;
};

template
<
    // system classes should have function expm for computing exponential matrix
    // includes the system Matrix A and input Matrix B as member variables
    class System,
    // gramian classes should implements operator() with parameter (t) and invertible by calling G(t).inv()
    class Gramian
    // , class State = typename System::StateType
    // , class Input = typename System::InputType
    // , class Weight = typename System::InputWeightType
    >
// compute optimal open loop control input given time
class FixedTimeLQR
{
  using State = typename System::StateType;
  using Input = typename System::InputType;
  using Weight = typename System::InputWeightType;
public:
  ATTRIBUTE
  FixedTimeLQR(const System &sys, const System &sys_t, const Gramian &gram, Weight R = Weight::Identity())
    : sys(sys), sys_t(sys_t), G(gram), R(R) {}
  template <typename numeric>
  ATTRIBUTE
  Input solve(numeric final_time, numeric t, const State &xi, const State &xf) const
  {
    Input u;
    auto dt = final_time - t;
    auto g = G(dt).inverse();
    auto expm = sys.expm(dt);
    auto expmt = expm.transpose();
    auto r_inv = R.inverse();
    auto b_trans = sys.B.transpose();
    u = r_inv * b_trans * expmt * g * (xf - expm*xi);
    return u;
  }
  Weight R;
  const Gramian &G;
  const System &sys;
  const System &sys_t;
};

template
<
    // a Cost class should overrides operator(), takes State xi, State xf, numeric
    // t as arguments and return numeric scalar value
    class Cost,
    // an OptTime class should have 'solve' function
    // and takes (const State&, const State&) as an argument and returns
    // scalar numeric type
    class OptTime,
    // a Control class should have 'solve' function with
    // (numeric, numeric, const State&, const State&) argunemts and
    // returns Control::Input
    class Control,
    // a system should have matrix A and B
    class System,
    // gramian classes should implements operator() with parameter (t)
    // and invertible by calling G(t).inv()
    class Gramian,
    // a composite system has 2(nxn) size of System,
    // to compute the trajectory given time, optimal time and xf
    // should have 'expm' method that gives exponential matrix
    class CompositeSystem
    // State class should overloads operator << to fill its value
    // given states with another dimension
    // , class CompositeState = typename CompositeSystem::StateType
    // , class State = typename System::StateType
    // , class Input = typename System::InputType
    , class Scalar = double
    >
// compute optimal open loop trajectory given initial and final states
class OptTrjSolver
{
  // we use this so that we do not pollute namspace
  // or we could wrap this solver class inside some namespace
  template <class Type>
  using decay = std::decay_t<Type>;
public:
  using State = typename System::StateType;
  using CompositeState = typename CompositeSystem::StateType;
  // deduce the Input type while making sure the Control class has required function
  using Input = decay<decltype(std::declval<Control>().solve(std::declval<Scalar>(), std::declval<Scalar>(), std::declval<State>(), std::declval<State>()))>;
  //  using Input = System::Input;
  //  using State = std::decay_t<decltype(std::declval<System>().expm(double(0.0)))>;
  typedef std::vector<std::tuple<Scalar,State,Input>> Trajectory;
public:
  ATTRIBUTE
  OptTrjSolver(Cost &cost_fn, OptTime &time_solver, Control &controller, System &system, Gramian &G, CompositeSystem &cmp_sys)
    : cost_fn(cost_fn), opt_time_solver(time_solver), controller(controller), system(system), G(G), cmp_sys(cmp_sys) {}

  template<int segment = 10>
#ifdef GPU
  __host__
#endif
  Trajectory
  solve(const State& xi, const State& xf)
  {
    std::vector<std::tuple<Scalar,State,Input>> ret;
    if(xi != xf)
    {
      auto opt_time = opt_time_solver.solve(xi,xf);
      auto dt = opt_time/segment;
      auto g_mat = G(opt_time);
      auto g_inv = g_mat.inverse();
      auto d_opt = g_inv*(xf-system.expm(opt_time)*xi);
      CompositeState cmp_state;
      cmp_state << xf, d_opt;
      // ret.push_back(std::make_tuple(TimeIdx(0.0),xi,Input()));
      for(int i=0; i<=segment; i++) {
        auto t = opt_time*i/segment;
        // auto ctrl = controller.solve(opt_time, t, xi, xf);
        auto em = cmp_sys.expm(t-opt_time);
        CompositeState s = em*cmp_state;
        State state, yt;
        for(int k=0; k<s.rows()/2; k++)
          state(k) = s(k);
        for(int k=s.rows()/2; k<s.rows(); k++)
          yt(k-s.rows()/2) = s(k);
        // State state(s.data());
        // State yt(s.data()+s.rows());
        auto ctrl = controller.R.inverse() * system.B.transpose() * yt;
        ret.push_back(std::make_tuple(t,state,ctrl));
      }
    }
    return ret;
  }

  Trajectory
  solve(const State& xi, const State& xf, Scalar dt)
  {
    std::vector<std::tuple<Scalar,State,Input>> ret;
    if(xi != xf)
    {
      auto opt_time = opt_time_solver.solve(xi,xf);
      auto segment = int(opt_time/dt);
      auto g_mat = G(opt_time);
      auto g_inv = g_mat.inverse();
      auto d_opt = g_inv*(xf-system.expm(opt_time)*xi);
      CompositeState cmp_state;
      cmp_state << xf, d_opt;
      // ret.push_back(std::make_tuple(TimeIdx(0.0),xi,Input()));
      for(int i=0; i<=segment; i++) {
        auto t = opt_time*i/segment;
        t = std::min(t,opt_time);
        // auto ctrl = controller.solve(opt_time, t, xi, xf);
        auto em = cmp_sys.expm(t-opt_time);
        CompositeState s = em*cmp_state;
        State state, yt;
        for(int k=0; k<s.rows()/2; k++)
          state(k) = s(k);
        for(int k=s.rows()/2; k<s.rows(); k++)
          yt(k-s.rows()/2) = s(k);
        // State state(s.data());
        // State yt(s.data()+s.rows());
        auto ctrl = controller.R.inverse() * system.B.transpose() * yt;
        ret.push_back(std::make_tuple(t,state,ctrl));
      }
    }
    return ret;
  }

  template<typename scalar=Scalar>
#ifdef GPU
  __host__
#endif
  std::tuple<scalar,scalar> cost(const State& xi, const State& xf)
  {
    if(xi == xf)
      return std::make_tuple((scalar)0.0,(scalar)0.0);
    auto t = opt_time_solver.solve(xi,xf);
    auto c = cost_fn(xi,xf,t);
    return std::make_tuple((scalar)t,(scalar)c);
  }
  OptTime &opt_time_solver;
  const Cost &cost_fn;
  const Control &controller;
  const System &system;
  const Gramian &G;
  const CompositeSystem &cmp_sys;
};

#endif // FIXEDTIMELQR_HPP
