#ifndef INTEGRATOR2DRRT_HPP
#define INTEGRATOR2DRRT_HPP

#include <cmath>
#include <memory>
#include "environment.hpp"
#include "logger.hpp"
#include "states.hpp"
#include "kdtree.hpp"
#include "rrtstar.hpp"
#include "rrtvisual.hpp"
#include "integrator2d.hpp"
#include "random.hpp"

// #define NO_OBS


namespace Kinodynamic {

constexpr int segment = 10;
constexpr int obs_count = 9;
// collision check segment for dynamic obs
constexpr int collision_segment = 10;

typedef Models::scalar scalar;
typedef State<scalar,Models::n> state_t;
typedef States<scalar,Models::n> states_t;
typedef KDTree<states_t,4,Models::scalar,state_t> KDTreeInt2D;
typedef double CostType;
typedef Models::Integrator2DCost CostFN;

template<typename Scalar = double, typename State = state_t, int dim = State::dim>
struct TimeState : std::tuple<Scalar,State>
{
  TimeState() {}
  TimeState<Scalar, State, dim> &operator =(const std::tuple<Scalar,State>& rhs) {
    std::get<0>(*this) = std::get<0>(rhs);
    std::get<1>(*this) = std::get<1>(rhs);
    return *this;
  }

  template<typename Index, int time_idx = dim>
  Scalar operator()(Index idx) const {
    return (idx < dim ? std::get<1>(*this)(idx) : std::get<0>(*this));
  }
};

template<typename Scalar = double, typename State = state_t, int n = segment>
struct Trajectory : std::array<TimeState<Scalar,State>,n+1>
{
  Trajectory(const Models::Integrator2DTrajectorySolver::Trajectory &trajectory)
  {
    for(size_t i=0; i<=n; i++)
      (*this)[i] = std::make_tuple(Scalar(std::get<0>(trajectory[i])),State(std::get<1>(trajectory[i])));
  }
  Trajectory(const Trajectory& t) {
    for(size_t i=0; i<=n; i++)
      (*this)[i] = t[i];
  }
  Trajectory() {
    for(size_t i=0; i<=n; i++)
      (*this)[i] = std::make_tuple(Scalar(0.0),state_t());
  }
  Trajectory(const state_t &s) {
    for(size_t i=0; i<=n; i++)
      (*this)[i] = std::make_tuple(Scalar(0.0),s);
  }
  std::array<Scalar,n+1> time() const
  {
    std::array<Scalar,n+1> ret;
    for(size_t i=0; i<=n; i++)
      ret[i] = std::get<0>((*this)[i]);
    return ret;
  }
  Trajectory<Scalar,State,n> operator + (const Scalar &t)
  {
    auto trajectory = *this;
    for(auto & trj: trajectory)
      std::get<0>(trj) += t;
    return trajectory;
  }
  std::array<state_t,n+1> path() const {
    std::array<state_t,n+1> ret;
    for(size_t i=0; i<=n; i++)
      ret[i] = std::get<1>((*this)[i]);
    return ret;
  }
  // const std::tuple<Scalar,state_t>& operator[](size_t i) const { return trj[i]; }
  // std::tuple<Scalar,state_t>& operator[](size_t i) { return trj[i]; }
  // std::array<std::tuple<Scalar,state_t>,n+1> trj;
};
template <typename T, typename S, int n> using Trajectories = std::vector<Trajectory<T,S,n>>;

struct CostInt2D
{
  typedef scalar Scalar;
  CostInt2D(Models::Integrator2DTrajectorySolver &solver);
  Scalar operator()(const state_t& s0, const state_t& s1) const;
  Models::Integrator2DTrajectorySolver &solver;
};

struct TreeInt2D;

struct Connector
{
  typedef Trajectory<scalar,state_t,segment> Edge;

  Connector(Models::Integrator2DTrajectorySolver &solver, TreeInt2D &tree);
  inline
  auto operator()(const state_t &s0, const state_t &s1);
  inline
  auto last_connection()
  {
    return e;
  }
  Edge e; // last connection
  Models::Integrator2DTrajectorySolver &solver;
  TreeInt2D &tree;
};

struct TreeInt2D
{
  typedef int Index;
  typedef std::vector<Index> IndexList;
  typedef state_t State;
  // typedef std::vector<std::reference_wrapper<State>> StateList;
  typedef std::vector<State> StateList;

  TreeInt2D() {}

  inline
  auto nearest(const State &s0, const scalar& radius)
  {
    IndexList ret;
    auto vp = tree.nearest(s0,radius);
    for(const auto& p : vp)
      ret.push_back(Index(p.first));
    return ret;
  }

  inline
  auto states(const IndexList &indexes)
  {
    StateList ret;
    for(const auto &i : indexes)
      ret.push_back(tree(i));
    return ret;
  }

  inline
  auto insert(const State &s, const Index &idx)
  {
    Index id = tree.size();
    auto e = Trajectory<scalar,state_t,segment>(s);
    tree.addPoint(s);
    setParent(id, idx);
    setEdge(id, e);
    return id;
  }

  inline
  auto insert(const State &s, const Index &idx, const Connector::Edge &e)
  {
    Index id = tree.size();
    tree.addPoint(s);
    parent.push_back(idx);
    auto edge = e;
    // if(idx >= 0) {
    // auto t = std::get<0>(trajectories.at(idx).back());
    // edge = edge + t;
    // }
    trajectories.push_back(edge);
    return id;
  }

  void reset();

  inline
  void setParent(const Index &node, const Index &p)
  {
    if(int(parent.size()) < node+1)
      parent.push_back(p);
    else parent.at(node) = p;
  }

  inline
  void setEdge(const Index &n, const Connector::Edge &e)
  {
    auto edge = e;
    // if(parent.at(n) >= 0) {
    // auto t = std::get<0>(trajectories.at(parent.at(n)).back());
    // edge = edge + t;
    // }
    if(int(trajectories.size()) < n+1)
      trajectories.push_back(edge);
    else trajectories.at(n) = edge;
  }

  // const State& operator()(const Index &i) const
  // {
  // last_checked_idx = i;
  // return tree(i);
  // }

  inline
  auto& operator()(const Index &i)
  {
    last_checked_idx = i;
    return tree(i);
  }

  void dump_text(const std::string &node_file, const std::string &parent_file, const std::string &trajectory_file);

  void from_text(const std::string &node_file, const std::string &parent_file, const std::string &trajectory_file);

  Trajectories<scalar,state_t,segment> get_trajectory(Index idx);

  int last_checked_idx = -1;
  IndexList parent;
  KDTreeInt2D tree;
  Trajectories<scalar,state_t,segment> trajectories;
};

auto Connector::operator()(const state_t &s0, const state_t &s1)
{
  // Models::Integrator2DSS::StateType xi;
  // Models::Integrator2DSS::StateType xf;
  // for(size_t i=0; i<4; i++) {
  // xi(i) = s0(i);
  // xf(i) = s1(i);
  // }
  auto trajectory = solver.solve<segment>(s0, s1);
  auto ti_idx = tree.last_checked_idx;
  auto t0 = 0.0;
  e = Trajectory<scalar,state_t,segment>(trajectory);
  if(ti_idx > 0) {
    t0 = std::get<0>(tree.trajectories.at(ti_idx).back());
    e = e + t0;
  }
  return e;
}

#define SAMPLE_X0 (11.0)
#define SAMPLE_X1 (7.0)
#define SAMPLE_X2 (1.5)
#define SAMPLE_X3 (1.5)

// for now, dont compile this on cuda
// @TODO : make this work on both
#ifndef __NVCC__

struct CollisionTimeSpaceChecker
{
  CollisionTimeSpaceChecker(Models::Integrator2DTrajectorySolver& solver, DynamicRobosoccer<scalar,obs_count> &env);

  void setRandomObstacles();

  bool operator() (const Connector::Edge &e);

  //  bool operator() (const state_t &s0, const state_t &s1) const
  //  {

  //  }

  Models::Integrator2DTrajectorySolver &solver;
  DynamicRobosoccer<scalar,obs_count> &env;
  RandomGen<4,scalar> *rg;
};

struct CollisionChecker
{
  CollisionChecker(Models::Integrator2DTrajectorySolver &solver, Robosoccer<scalar,obs_count> &env);

#ifndef __NVCC__
  void dump_text(const std::string &env_file, const std::string &collision_file);

  void from_text(const std::string &file);

  void setRandomObstacles();
#endif

  bool operator() (const Connector::Edge &e);

  bool operator() (const state_t &s0, const state_t &s1) const;

  std::vector<std::tuple<state_t,state_t>> collisions;
  Models::Integrator2DTrajectorySolver &solver;
  Robosoccer<scalar,obs_count> &env;
  RandomGen<2,scalar> *rg;
};
#endif

template <typename Environment>
struct Sampler
{
  Sampler(Environment &env, bool direct_sample = false, double direct_sample_prob = 0.5)
    : env(env)
    , direct_sampling_enable(direct_sample)
  {
    rg = new RandomGen<4,scalar>(
    {-SAMPLE_X0,-SAMPLE_X1,-SAMPLE_X2,-SAMPLE_X3},
    {SAMPLE_X0,SAMPLE_X1,SAMPLE_X2,SAMPLE_X3});
    direct_sampler = new RandomGen<1,bool>({direct_sample_prob});
  }

  void set_direct_sample(bool en, double prob)
  {
    direct_sampling_enable = en;
    if(direct_sampler->p[0] != prob) {
      delete direct_sampler;
      direct_sampler = new RandomGen<1,bool>({prob});
    }
  }

  inline
  auto operator()()
  {
    if((direct_sampling_enable) && (*direct_sampler)(0)) {
      s = target;
    }
    else {
      (*rg)(s);
      // for now dont compile this on cuda
      // @TODO : fix
#ifndef __NVCC__
      while(env.collide(s))
        (*rg)(s);
#endif
    }
    return s;
  }

  inline
  auto last_sample()
  {
    return s;
  }

  state_t s;
  state_t target;
  Environment &env;
  bool direct_sampling_enable = false;
  RandomGen<1,bool> *direct_sampler;
  RandomGen<4,scalar> *rg = nullptr;
};

template <typename Environment>
struct GoalChecker
{
  GoalChecker(Environment &env) : env(env) {
    rg = new RandomGen<2,scalar>({-SAMPLE_X0,-SAMPLE_X1},{SAMPLE_X0,SAMPLE_X1});
  }
  inline
  bool operator()(const state_t &state)
  {
    // this implementation requires exact goal, so just return false for this fn
    return false;
  }
  state_t randomGoal() {
    state_t s;
    s(2) = s(3) = 0.0;
    (*rg)(s);
    // for now dont compile this on cuda
    // todo resolve
  #ifndef __NVCC__
    while(env.collide(s))
      (*rg)(s);
  #endif
    return s;
  }

  Environment &env;
  RandomGen<2,scalar> *rg;
};

#define SOME_CONSTANT (10.0)
#define SAMPLE_VOLUME (SAMPLE_X0*SAMPLE_X1*SAMPLE_X2*SAMPLE_X3)*(2*2*2*2)

struct NeighborRadius
{
  NeighborRadius();
  double operator()(const TreeInt2D::Index &i);
  double s;
  double scale = SOME_CONSTANT;
};

typedef Robosoccer<scalar,obs_count> StaticEnvironment;
typedef DynamicRobosoccer<scalar,obs_count> DynamicEnvironment;

#if 0
TreeInt2D tree_int2d;

StaticEnvironment robosoccer_env;
Environment dynamic_soccer_env;

// don't instantiate these object if use cuda
#ifndef __NVCC__
CostInt2D cost_int2d(Models::integrator2d_trj_solver);
Connector connector(Models::integrator2d_trj_solver, tree_int2d);
#endif

// automatic template class deduction, need c++17 (gcc >= 7)
#if (__GNUC__ < 7) || defined(__NVCC__)
GoalChecker<StaticEnvironment> goal(robosoccer_env);
GoalChecker<DynamicEnvironment> goal_dynamic_env(dynamic_soccer_env);
#else
GoalChecker goal(robosoccer_env);
GoalChecker goal_dynamic_env(dynamic_soccer_env);
#endif
// automatic template class deduction, need c++17 (gcc >= 7)
#if (__GNUC__ < 7) || defined(__NVCC__)
Sampler<StaticEnvironment> sampler(robosoccer_env);
Sampler<DynamicEnvironment> sampler_dynamic_env(dynamic_soccer_env);
#else
Sampler sampler(robosoccer_env);
Sampler sampler_dynamic_env(dynamic_soccer_env);
#endif
#else

typedef RRTStar
<TreeInt2D,CostInt2D,Sampler<StaticEnvironment>,NeighborRadius,CollisionChecker,GoalChecker<StaticEnvironment>,Connector>
RRTStarInt2D;

typedef RRTStar
<TreeInt2D,CostInt2D,Sampler<DynamicEnvironment>,NeighborRadius,CollisionTimeSpaceChecker,GoalChecker<DynamicEnvironment>,Connector>
RRTStarInt2DTimeSpaceObs;

class Wrapper
{
  Wrapper() {}
  static Models::Integrator2D integrator2d;
  static TreeInt2D *tree_int2d;
  static StaticEnvironment *robosoccer_env;
  static DynamicEnvironment *dynamic_soccer_env;
  static CostInt2D *cost_int2d;
  static Connector *connector;
  static GoalChecker<StaticEnvironment> *goal;
  static GoalChecker<DynamicEnvironment> *goal_dynamic_env;
  static Sampler<StaticEnvironment> *sampler;
  static Sampler<DynamicEnvironment> *sampler_dynamic_env;
  static NeighborRadius *radius;
  static CollisionChecker *checker;
  static CollisionTimeSpaceChecker *checker_time_space;
  static RRTStarInt2D *rrtstar_int2d;
  static RRTStarInt2DTimeSpaceObs *rrtstar_int2d_timespace_obs;

  static void initialize();

public:
  static TreeInt2D &get_tree_int2d();
  static StaticEnvironment &get_robosoccer_env();
  static DynamicEnvironment &get_dynamic_soccer_env();
  static CostInt2D &get_cost_int2d();
  static Connector &get_connector();
  static GoalChecker<StaticEnvironment> &get_goal();
  static GoalChecker<DynamicEnvironment> &get_goal_dynamic_env();
  static Sampler<StaticEnvironment> &get_sampler();
  static Sampler<DynamicEnvironment> &get_sampler_dynamic_env();
  static NeighborRadius &get_radius();
  static CollisionChecker &get_checker();
  static CollisionTimeSpaceChecker &get_checker_time_space();
  static RRTStarInt2D &get_rrtstar_int2d();
  static RRTStarInt2DTimeSpaceObs &get_rrtstar_int2d_timespace_obs();
};
#endif


#if 0
// dont instantiate these object if compile using cuda
#ifndef __NVCC__
CollisionChecker checker(Models::integrator2d_trj_solver, robosoccer_env);
CollisionTimeSpaceChecker checker_time_space(Models::integrator2d_trj_solver, dynamic_soccer_env);

// automatic template class deduction, need c++17 (gcc >= 7)
#if defined(__GNUC__) && (__GNUC__ >= 7)
RRTStar rrtstar_int2d(tree_int2d, cost_int2d, sampler, checker, radius, goal, connector);
RRTStar rrtstar_int2d_timespace_obs(tree_int2d, cost_int2d, sampler_dynamic_env, checker_time_space, radius, goal_dynamic_env, connector);
#else
typedef RRTStar
<TreeInt2D,CostInt2D,Sampler<StaticEnvironment>,NeighborRadius,CollisionChecker,GoalChecker<StaticEnvironment>,Connector>
RRTStarInt2D;

typedef RRTStar
<TreeInt2D,CostInt2D,Sampler<DynamicEnvironment>,NeighborRadius,CollisionTimeSpaceChecker,GoalChecker<DynamicEnvironment>,Connector>
RRTStarInt2DTimeSpaceObs;

RRTStarInt2D rrtstar_int2d(tree_int2d, cost_int2d, sampler, checker, radius, goal, connector);
RRTStarInt2DTimeSpaceObs rrtstar_int2d_timespace_obs(tree_int2d, cost_int2d, sampler_dynamic_env, checker_time_space, radius, goal_dynamic_env, connector);
#endif
#endif
#endif
}

#endif // INTEGRATOR2DRRT_HPP
