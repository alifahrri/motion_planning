#ifndef OBSTACLES_HPP
#define OBSTACLES_HPP

#include <random>
#include <cmath>
#include <array>
#include <tuple>
// #include "collision.hpp"

#ifndef NDEBUG
// #include "util.h"
// #define TRACE_EXEC
#ifdef __NVCC__
// #define TRACE_CUDA
#endif
#endif

#ifdef GPU
#include <cuda.h>
#include <cuda_runtime.h>
#define HOST __host__
#define DEVICE __device__
#define ATTRIBUTE __host__ __device__
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
#endif // gpuErrchk
#else
#define ATTRIBUTE
#define DEVICE
#define HOST
#endif // GPU


#define DEFAULT_ROBOT_RADIUS (0.26)
#define DEFAULT_SAFETY_RADIUS (0.15)
#define SAMPLE_X0 (11.0)
#define SAMPLE_X1 (7.0)
#define SAMPLE_X2 (1.5)
#define SAMPLE_X3 (1.5)

#ifdef __NVCC__
template <typename scalar> struct Obstacle
{
#else
template <typename scalar>
struct Obstacle : public std::tuple<scalar,scalar>
{
#endif

  ATTRIBUTE
  Obstacle() {}

  HOST
  Obstacle& operator = (const std::tuple<scalar,scalar> rv) {
    (*this)[0] = std::get<0>(rv);
    (*this)[1] = std::get<1>(rv);
    return *this;
  }

  ATTRIBUTE
  inline
  const scalar& operator[](size_t i) const {
#ifdef __NVCC__
    return (i == 0 ? x : (i == 1 ? y : 0));
#else
    switch (i) {
    case 0:
      return std::get<0>(*this);
    case 1:
      return std::get<1>(*this);
    }
#endif
  }

  ATTRIBUTE
  inline
  scalar& operator[](size_t i) {
#ifdef __NVCC__
    return (i == 0 ? x : y);
#else
    switch (i) {
    case 0:
      return std::get<0>(*this);
    case 1:
      return std::get<1>(*this);
    }
#endif
  }

  ATTRIBUTE
  inline
  const scalar& operator ()(size_t i) const {
    return (*this)[i];
  }
#ifdef __NVCC__
  scalar x;
  scalar y;
#endif
};

#ifdef __NVCC__
template <typename scalar>
struct DynamicObstacle
{
#else
template <typename scalar>
// struct DynamicObstacle : public std::tuple<scalar,scalar,scalar,scalar>
struct DynamicObstacle : public std::array<scalar,4>
{
#endif
  ATTRIBUTE
  DynamicObstacle() {}

  HOST
  DynamicObstacle& operator = (const std::array<scalar,4> &rv) {
    (*this)[0] = std::get<0>(rv);
    (*this)[1] = std::get<1>(rv);
    (*this)[2] = std::get<2>(rv);
    (*this)[3] = std::get<3>(rv);
    return *this;
  }

  template <typename time>
  ATTRIBUTE
  inline
  DynamicObstacle<scalar>
  operator ()(time t) const {
    DynamicObstacle<scalar> ret;
    ret[0] = (*this)[0] + (*this)[2] * t;
    ret[1] = (*this)[1] + (*this)[3] * t;
    return ret;
  }

  // template <>
  // gcc doesn't allow inline explicit template specialization, btw
  // but this does the trick
  ATTRIBUTE
  inline
  scalar operator ()(size_t i) const {
    return (*this)[i];
  }
  ATTRIBUTE
  inline
  scalar operator ()(int i) const {
    return (*this)[i];
  }

#ifdef __NVCC__
  __host__ __device__
  inline
  scalar& operator[](size_t i) {
    return state[i];
  }
  __host__ __device__
  inline
  const scalar& operator[](size_t i) const {
    return state[i];
  }
  scalar state[4];
#endif
};

template <typename scalar = double, int n = 9>
struct Obstacles : public std::array<Obstacle<scalar>,n>
{
  Obstacles() {}

  inline
  std::array<std::tuple<scalar,scalar>,n> operator()
  (std::array<std::tuple<scalar,scalar>,n> &lhs, Obstacles &rhs) {
    for(size_t i=0; i<n; i++)
      lhs[i] = std::tuple<scalar,scalar>(rhs[i]);
    return lhs;
  }
};

template <typename scalar = double, int n = 9>
struct DynamicObstacles : public std::array<DynamicObstacle<scalar>,n>
{
  DynamicObstacles() {}

  inline
  std::array<std::tuple<scalar,scalar,scalar,scalar>,n> operator()
  (std::array<std::tuple<scalar,scalar,scalar,scalar>,n> &lhs, DynamicObstacles &rhs) {
    for(size_t i=0; i<n; i++)
      lhs[i] = std::tuple<scalar,scalar,scalar,scalar>(rhs[i]);
    return lhs;
  }
};

namespace geometry {
template <typename scalar = double>
struct Point {
    scalar x;
    scalar y;
};

template <typename scalar = double>
struct Line {
    Point<scalar> p0;
    Point<scalar> p1;
};

template <typename scalar = double>
struct Polygon : std::vector<Line<scalar>> {

};

template <typename scalar = double>
struct Circle {
  scalar x;
  scalar y;
  scalar r;
};
}

#endif // OBSTACLES_HPP