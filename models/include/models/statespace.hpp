#ifndef STATESPACE_HPP
#define STATESPACE_HPP

#include "choice.hpp"
#include "statespace_utils.hpp"

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

template <typename Scalar, int n, int p, int q, typename ExpFN = JordanExp<Scalar,n>, typename LinearizationFN = nullptr_t>
class StateSpace
{
  using LinearizationResolver = ss_utils::LinearizationResolver<>;
public:
  /* states */
  typedef Eigen::Matrix<Scalar,n,1> StateType;
  typedef Eigen::Matrix<Scalar,p,1> InputType;
  /* matrices */
  typedef Eigen::Matrix<Scalar,p,p> InputWeightType;
  typedef Eigen::Matrix<Scalar,n,n> SystemMatrix;
  typedef Eigen::Matrix<Scalar,n,p> InputMatrix;
  typedef Eigen::Matrix<Scalar,q,n> OutputMatrix;
#ifndef __CUDA_ARCH__
  typedef std::function<std::tuple<SystemMatrix,SystemMatrix>(SystemMatrix)> JordanFormHelper;
#endif
public:
  ATTRIBUTE
  StateSpace(SystemMatrix A = SystemMatrix::Identity()
      , InputMatrix B = InputMatrix::Identity()
      , OutputMatrix C = OutputMatrix::Identity()
    #ifndef __CUDA_ARCH__
      , JordanFormHelper fn = nullptr
    #endif
      ) :
    A(A), B(B), C(C)
#ifndef __CUDA_ARCH__
  , jordan_form_fn(fn)
#endif
  {
#ifndef __CUDA_ARCH__
    computeTransitionMatrix();
#endif
  }

  ATTRIBUTE
  SystemMatrix expm(Scalar t) const
  {
    return exp_fn(t);
  }

#ifndef __CUDA_ARCH__
  void computeTransitionMatrix()
  {
    helper::computeTransitionMatrix<Scalar,n>(A,P,D,P_inv,jordan_form_fn);
  }
#endif

public:
  /* system matrix */
  SystemMatrix  A;
  InputMatrix   B;
  OutputMatrix  C;
  /* jordan form (to compute exp) */
  SystemMatrix P;
  SystemMatrix P_inv;
  SystemMatrix D;
  /* last linearization state */
  StateType x_hat;
  /* exponential function */
  ExpFN exp_fn;
  /* linearization function */
  LinearizationFN linearization_fn;

#ifndef __CUDA_ARCH__
  JordanFormHelper jordan_form_fn = nullptr;
#endif

private:
  template <typename LinearizationState>
  auto linearize(choice<2>, const LinearizationState &state) 
    -> decltype(LinearizationResolver::linearize(state, exp_fn))
  {
    return LinearizationResolver::linearize(state, exp_fn);
  }
  /* implementation of linearization calls */
  template <typename LinearizationState>
  auto linearize(choice<1>, const LinearizationState &state) 
    -> decltype(LinearizationResolver::linearize(this->A, state, linearization_fn))
  {
    return LinearizationResolver::linearize(A, state, linearization_fn);
  }

  template <typename LinearizationState>
  auto linearize(choice<0>, const LinearizationState &state) 
    -> decltype(LinearizationResolver::linearize(this->P, this->D, this->P_inv, state, linearization_fn))
  {
    return LinearizationResolver::linearize(P, D, P_inv, state, linearization_fn);
  }

  /* TODO : Remove; reason : ill-defined, let LinearizationFN decides */
  /* set linearization if needed */
  auto set_linearization_state(const StateType &state)
  -> decltype(LinearizationResolver::set_linearization_state(this->exp_fn, state))
  {
    return LinearizationResolver::set_linearization_state(this->exp_fn, state);
  }

public:
  /* entry point for linearization, only enabled if supported, 
    note : 
    - default (nullptr) will fail 
    - it is possible to have LinearizationState type different from StateType
  */
  template <typename LinearizationState>
  auto linearize(const LinearizationState &state) 
  -> decltype(linearize(choice<2>{}, state))
  {
    return linearize(choice<2>{}, state);
  }
};

/* TODO : simplify specialization */
// partial specialization
template <typename Scalar, int n, int p, int q, typename LinearizationFN>
class StateSpace<Scalar,n,p,q,JordanExp<Scalar,n>,LinearizationFN>
{
  using LinearizationResolver = ss_utils::LinearizationResolver<>;
public:
  /* states */
  typedef Eigen::Matrix<Scalar,n,1> StateType;
  typedef Eigen::Matrix<Scalar,p,1> InputType;
  /* matrices */
  typedef Eigen::Matrix<Scalar,p,p> InputWeightType;
  typedef Eigen::Matrix<Scalar,n,n> SystemMatrix;
  typedef Eigen::Matrix<Scalar,n,p> InputMatrix;
  typedef Eigen::Matrix<Scalar,q,n> OutputMatrix;
#ifndef __CUDA_ARCH__
  typedef std::function<std::tuple<SystemMatrix,SystemMatrix>(SystemMatrix)> JordanFormHelper;
#endif
public:
  ATTRIBUTE
  StateSpace(SystemMatrix A = SystemMatrix::Identity()
      , InputMatrix B = InputMatrix::Identity()
      , OutputMatrix C = OutputMatrix::Identity()
    #ifndef __CUDA_ARCH__
      , JordanFormHelper fn = nullptr
    #endif
      ) :
    A(A), B(B), C(C)
#ifndef __CUDA_ARCH__
  , jordan_form_fn(fn)
#endif
  {
    exp_fn = JordanExp<Scalar,n>(&D);
#ifndef __CUDA_ARCH__
    computeTransitionMatrix();
#endif
  }

  ATTRIBUTE 
  SystemMatrix expm(Scalar t) const
  {
    return P*exp_fn(t)*P_inv;
  }

#ifndef __CUDA_ARCH__
  void computeTransitionMatrix()
  {
    helper::computeTransitionMatrix<Scalar,n>(A,P,D,P_inv,jordan_form_fn);
  }
#endif

public:
  /* system matrix */
  SystemMatrix  A;
  InputMatrix   B;
  OutputMatrix  C;
  /* jordan form (to compute exp) */
  SystemMatrix P;
  SystemMatrix P_inv;
  SystemMatrix D;
  /* last linearization state */
  StateType x_hat;
  /* exp function */
  JordanExp<Scalar,n> exp_fn;
  /* linearization function */
  LinearizationFN linearization_fn;
#ifndef __CUDA_ARCH__
  JordanFormHelper jordan_form_fn = nullptr;
#endif

private:
  template <typename LinearizationState>
  auto linearize(choice<2>, const LinearizationState &state) 
    -> decltype(LinearizationResolver::linearize(state, exp_fn))
  {
    return LinearizationResolver::linearize(state, exp_fn);
  }
  /* implementation of linearization calls */
  template <typename LinearizationState>
  auto linearize(choice<1>, const LinearizationState &state) 
    -> decltype(LinearizationResolver::linearize(A, state, linearization_fn))
  {
    return LinearizationResolver::linearize(A, state, linearization_fn);
  }

  template <typename LinearizationState>
  auto linearize(choice<0>, const LinearizationState &state) 
    -> decltype(LinearizationResolver::linearize(P, D, P_inv, state, linearization_fn))
  {
    return LinearizationResolver::linearize(P, D, P_inv, state, linearization_fn);
  }

public:
  /* entry point for linearization, only enabled if supported, 
    note : 
    - default (nullptr) will fail 
    - it is possible to have LinearizationState type different from StateType
  */
  template <typename LinearizationState>
  auto linearize(const LinearizationState &state) 
    -> decltype(linearize(choice<2>{}, state))
  {
    return linearize(choice<2>{}, state);
  }

};

#endif // STATESPACE_HPP
