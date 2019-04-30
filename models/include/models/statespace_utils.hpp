#ifndef STATE_SPACE_UTILS
#define STATE_SPACE_UTILS

#include "choice.hpp"

#include <type_traits>
#include <vector>
#include <functional>

#include <math_constants.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Eigenvalues>

#ifdef GPU
#include <cuda.h>
#include <cuda_runtime.h>
#define ATTRIBUTE __host__ __device__
#else
#define ATTRIBUTE
#endif

namespace ss_utils {

/* TODO : use standalone function instead of struct (?) */
/* sink hole is disabled by default */
template <size_t n_functions = 3>
struct LinearizationResolver {
  static constexpr size_t n_tag  = n_functions;

  template <typename FunctionObject>
  static
  auto linearization_impl(choice<3>, const auto &state, FunctionObject&& fn)
    -> decltype(fn.linearize(state))
  {
    return fn.linearize(state);
  }

  template <typename FunctionObject, typename ...Args>
  static
  auto linearization_impl(choice<2>, Args&&...args, const auto &state, FunctionObject&& fn)
    -> decltype(fn(state, args...), bool{}) 
  {
    fn(state, args...);
    return true;
  }

  template <typename FunctionObject, typename ...Args>
  static
  auto linearization_impl(choice<1>, Args&&...args, const auto &state, FunctionObject&& fn)
    -> decltype(std::tie(args...)=fn(state), bool{}) 
  {
    std::tie(args...)=fn(state);
    return true;
  }

  template <typename ...Args>
  static
  auto linearization_impl(choice<0>, Args...args)
    -> decltype(bool{}) 
  {
    // sink hole, linearization not necessary
    return true;
  }  

  template <typename ...Args>
  auto operator()(Args&& ... args) 
    -> decltype(linearization_impl(choice<n_tag>{}, args...))
  {
    return linearization_impl(choice<n_tag>{}, args...);
  }

  /* provided for convinience */
  template <typename ...Args>
  static
  auto linearize(Args&& ... args) 
    -> decltype(this->linearization_impl(choice<n_tag>{}, args...))
  {
    return linearization_impl(choice<n_tag>{}, args...);
  }
	
  /* TODO : remove; reason : ill-defined & misleading */
	 /* set linearization if needed */
	 template <typename ExpFn, typename StateType>
	 static
  auto set_linearization_state(choice<2>,  ExpFn &exp_fn, const StateType &state) 
    -> decltype(exp_fn.linearize(state))
  {
    return exp_fn.linearize(state);
  }
   template <typename ExpFn, typename StateType>
	 static
  auto set_linearization_state(choice<1>,  ExpFn &exp_fn, const StateType &state) 
    -> decltype(exp_fn.set(state))
  {
    return exp_fn.set(state);
  }
	template <typename ExpFn, typename StateType>
	static
  auto set_linearization_state(choice<0>, ExpFn &exp_fn, const StateType &state) 
    -> decltype(void())
  {
    // sink hole, we don't need any particular fn before linearization
  }
	
	template <typename ExpFn, typename StateType>
	static
	auto set_linearization_state(ExpFn &exp_fn, const StateType &state) 
		-> decltype((set_linearization_state(choice<2>{},exp_fn,state)))
		{
			return set_linearization_state(choice<2>{},exp_fn,state);
		}
};

}

/* TODO : move to ss_utils namespace */
namespace helper {
  template <typename Type, int n, typename SystemMatrix>
  void computeTransitionMatrix(SystemMatrix &A, SystemMatrix &P, SystemMatrix &D, SystemMatrix &P_inv, auto &&jordan_form_fn)
  {
    Eigen::EigenSolver<SystemMatrix> eigen_solver(A);

    // Diagonalization
    // take only real parts of the given complex vectors from eigen, therefore
    // it is your job to ensure that the given system has only real eigen values
    // LIMITATIONS : you should make sure that the resulting Matrix P is Invertible
    // and has FULL RANK, otherwise the computation wouldn't be make sense
    // Hint : just print P_inv to see if it is make sense and balanced enough
    auto eigen_values = eigen_solver.eigenvalues().real();

    // create dictionary for eigen value, and find algebraic multiplicity
    std::vector<std::pair<Type,int>> eig_values;
    std::stringstream eg_dict_ss;
    for(size_t i=0; i<n; i++) {
      auto it = eig_values.begin();
      const auto &e = eigen_values(i);
      for(; it != eig_values.end(); it++) {
        if(std::get<0>(*it) == e)
          break;
      }
      if(it == eig_values.end())
        eig_values.push_back(std::make_pair(e,1));
      else
        std::get<1>(*it) += 1;
    }
    for(const auto &d : eig_values)
      eg_dict_ss << std::get<0>(d) << ": "<< std::get<1>(d) << ", ";

    if((eig_values.size()<n) && (jordan_form_fn)) {
      auto t = jordan_form_fn(A);
      auto J = std::get<0>(t);
      P = std::get<1>(t);
      P_inv = P.inverse();
      D = J;
      //      SystemMatrix a = P*J*P_inv;
    }
    else {
      P = eigen_solver.eigenvectors().real();
      P_inv = P.inverse();
      D = P_inv * A * P;
    }
  }
}

/* TODO : move to ss_utils namespace */
struct Factorial {
#ifndef __CUDA_ARCH__
  std::vector<int> cache;
  Factorial() { cache.push_back(1); }
  int operator()(int v) {
    if(v < cache.size())
      return cache[v];
    else {
      while(v >= cache.size())
        cache.push_back(cache.back() * (cache.size()));
      return cache.back();
    }
  }
#else
  ATTRIBUTE Factorial() {}
  ATTRIBUTE int operator() (int v) {
    int r = 1;
    for(int i=1; i<=v; i++)
      r*=i;
    return r;
  }
#endif
};

/* TODO : move to ss_utils namespace */
template <typename Type, int n>
struct JordanExp
{
  ATTRIBUTE JordanExp() {}
  ATTRIBUTE JordanExp(Eigen::Matrix<Type,n,n> *D) : D(D) {}
  ATTRIBUTE Eigen::Matrix<Type,n,n>
  operator()(Type t) const
  {
    auto tmp = *D;
    Factorial factorial;
    int init_block = 0;
    for(int i=0; i<n; i++) {
      auto d = (*D)(i,i);
      tmp(i,i) = exp(d*t);
      for(int k=1; k<=(i-init_block); k++)
        tmp(i-k,i) = (pow(t,k)/factorial(k)) * tmp(i,i);
      init_block = (i<n-1 ? ( (((*D)(i+1,i+1)==d) && ((*D)(i,i+1)==1)) ? init_block : i+1 ) : init_block);
    }
    return tmp;
  }
  Eigen::Matrix<Type,n,n> *D = nullptr;
};

#endif // STATE_SPACE_UTILS