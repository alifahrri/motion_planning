#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <type_traits>
#include <random>
#include <cmath>
#include <array>
#include <tuple>
#include "elements.hpp"
#include "collision.hpp"
#include "obstacles.hpp"
#include "random.hpp"

#ifndef NDEBUG
#include "util.h"
//#define TRACE_EXEC
#ifdef __NVCC__
//#define TRACE_CUDA
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
	if (code != cudaSuccess) {
		fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
		if (abort) exit(code);
	}
}
#endif
#else
#define ATTRIBUTE
#define DEVICE
#define HOST
#endif

template <typename scalar = double, int n = 9>
struct Robosoccer {
	Robosoccer(scalar x0 = scalar(SAMPLE_X0), scalar x1 = scalar(SAMPLE_X1))
	{
		rg = new RandomGen<2,scalar>({-x0,-x1}, {x0,x1});
		for(auto &o : obs)
			o = std::make_tuple(scalar(0.0),scalar(0.0));
	}
	Robosoccer(const std::array<std::tuple<scalar,scalar>,n> &obs,
	           scalar x0 = scalar(SAMPLE_X0),
	           scalar x1 = scalar(SAMPLE_X1))
	{
		auto i=0;
		for(const auto &o : obs)
			this->obs.at(i++) = o;
		rg = new RandomGen<2,scalar>({-x0,-x1}, {x0,x1});
	}

#ifndef __NVCC__
	void setRandomObstacles()
	{
		for(auto& o : obs) {
			std::get<0>(o) = (*rg)(0);
			std::get<1>(o) = (*rg)(1);
		}
	}
#endif

	template<typename ArrayLike, size_t x_idx=0, size_t y_idx=1>
	void setObstacles(const ArrayLike &obstacles)
	{
		for(size_t i=0; i<obstacles.size(); i++) {
			if(i == obs.size()) break;
			std::get<0>(obs.at(i)) = obstacles[i](x_idx);
			std::get<1>(obs.at(i)) = obstacles[i](y_idx);
		}
	}

	template <int x_idx=0, int y_idx=1, typename point_t>
	inline
	bool collide(const point_t &pt) const
	{
		return point_circle_collision<x_idx,y_idx>(pt,this->obs,this->collision_radius);
	}

	// see http://paulbourke.net/geometry/pointlineplane/
	template <int x_idx=0, int y_idx=1, typename p1_t,  typename p2_t>
	inline
	bool collide(const p1_t &pt0, const p2_t &pt1) const
	{
		return line_circle_collision<x_idx,y_idx>(pt0,pt1,this->obs,this->collision_radius);
	}

	// std::array<Obstacle<scalar>,n> obs;
	Obstacles<scalar,n> obs;
	scalar safety_radius = DEFAULT_SAFETY_RADIUS;
	scalar single_robot_radius = DEFAULT_ROBOT_RADIUS;
	scalar collision_radius = (DEFAULT_ROBOT_RADIUS+DEFAULT_SAFETY_RADIUS)*2;
	RandomGen<2,scalar> *rg;
};

template <typename scalar = double, int n = 9>
struct DynamicRobosoccer {
	DynamicRobosoccer(scalar x0 = scalar(SAMPLE_X0),
	                  scalar x1 = scalar(SAMPLE_X1),
	                  scalar x2 = scalar(SAMPLE_X2),
	                  scalar x3 = scalar(SAMPLE_X3))
	{
		rg = new RandomGen<4>({-x0,-x1,-x2,-x3}, {x0,x1,x2,x3});
		for(auto &o : obs)
			o = DynamicObstacle<scalar>();
	}
	DynamicRobosoccer(const std::array<DynamicObstacle<scalar>,n> &obs,
	                  scalar x0 = scalar(SAMPLE_X0),
	                  scalar x1 = scalar(SAMPLE_X1),
	                  scalar x2 = scalar(SAMPLE_X2),
	                  scalar x3 = scalar(SAMPLE_X3))
		: obs(obs)
	{
		rg = new RandomGen<4,scalar>({-x0,-x1,-x2,-x3}, {x0,x1,x2,x3});
	}

	inline
	std::array<DynamicObstacle<scalar>,n>
	at (scalar time)
	{
#ifdef TRACE_EXEC
		TRACE_FN(__PRETTY_FUNCTION__);
#endif
		std::array<DynamicObstacle<scalar>,n> ret;
		for(size_t i=0; i<ret.size(); i++)
			ret[i] = obs.at(i)(time);
#ifdef TRACE_EXEC
		DEBUG_PRINT(__FUNCTION__,"OK");
#endif
		return ret;
	}

	void setRandomObstacles()
	{
		for(auto& o : obs) {
#ifndef __NVCC__
			std::get<0>(o) = (*rg)(0);
			std::get<1>(o) = (*rg)(1);
			std::get<2>(o) = (*rg)(2);
			std::get<3>(o) = (*rg)(3);
#else
			(*rg)(o.state);
#endif
		}
	}

private:
	template<size_t x_idx=0, size_t y_idx=1, size_t vx_idx=2, size_t vy_idx=3>
	auto setObstaclesImpl(int, const auto &obstacles) -> decltype(obstacles[0](x_idx), void())
	{
		for(size_t i=0; i<obstacles.size(); i++) {
			if(i == obs.size()) break;
			std::get<0>(obs.at(i)) = obstacles[i](x_idx);
			std::get<1>(obs.at(i)) = obstacles[i](y_idx);
			std::get<2>(obs.at(i)) = obstacles[i](vx_idx);
			std::get<3>(obs.at(i)) = obstacles[i](vy_idx);
		}
	}

	template<size_t x_idx=0, size_t y_idx=1, size_t vx_idx=2, size_t vy_idx=3>
	auto setObstaclesImpl(char, const auto &obstacles) -> decltype(obstacles[0][x_idx], void())
	{
		for(size_t i=0; i<obstacles.size(); i++) {
			if(i == obs.size()) break;
			std::get<0>(obs.at(i)) = obstacles[i][x_idx];
			std::get<1>(obs.at(i)) = obstacles[i][y_idx];
			std::get<2>(obs.at(i)) = obstacles[i][vx_idx];
			std::get<3>(obs.at(i)) = obstacles[i][vy_idx];
		}
	}

public:
	template<typename ArrayLike, size_t x_idx=0, size_t y_idx=1, size_t vx_idx=2, size_t vy_idx=3>
	void setObstacles(const ArrayLike &obstacles)
	{
		setObstaclesImpl<x_idx,y_idx,vx_idx,vy_idx>(0, obstacles);
	}

	template <int x_idx=0, int y_idx=1, typename point_t>
	bool collide(const point_t &pt) const
	{
		return point_circle_collision<x_idx,y_idx>(pt,this->obs,this->collision_radius);
	}

	// see http://paulbourke.net/geometry/pointlineplane/
	template <int x_idx=0, int y_idx=1, int segment = 10, typename p1_t,  typename p2_t>
	bool collide(const p1_t &pt0, const p2_t &pt1, const scalar &t0, const scalar &t1) const
	{
		return parametrized_line_circle_collision<x_idx,y_idx,segment>(pt0,pt1,t0,t1,obs,collision_radius);
	}

	// std::array<DynamicObstacle<scalar>,n> obs;
	DynamicObstacles<scalar,n> obs;
	const scalar safety_radius = DEFAULT_SAFETY_RADIUS;
	const scalar single_robot_radius = DEFAULT_ROBOT_RADIUS;
	scalar collision_radius = (DEFAULT_ROBOT_RADIUS+DEFAULT_SAFETY_RADIUS)*2;
	RandomGen<4,scalar> *rg;
};

namespace environment
{
using namespace geometry;
using namespace collision;
using namespace elements;

template <typename scalar = double, size_t dim = 2>
struct Environment {
	std::vector<Polygon<scalar,dim>> polygons;
	std::vector<Circle<scalar>> circles;

	template <size_t x_idx=0, size_t y_idx=1, size_t z_idx=2, size_t p0_idx=0, size_t p1_idx=1>
	inline
	auto add_polygons(choice<0>, const auto &polys) -> decltype(polys.size(), x(p0(elements::line(polys.at(0),0))), void())
	{
		using line_t = decltype(elements::line(polys.at(0),0));
		for(size_t i=0; i<polys.size(); i++) {
			Polygon<scalar> p;
			auto fn = [&](const line_t &line) {
				Line<scalar,dim> l;
				auto lp0 = p0<p0_idx>(line);
				auto lp1 = p1<p1_idx>(line);
				x(p0(l)) = x<x_idx>(lp0);
				x(p1(l)) = x<x_idx>(lp1);
				y(p0(l)) = y<y_idx>(lp0);
				y(p1(l)) = y<y_idx>(lp1);
				if constexpr(dim == 3) {
					z(p0(l)) = z<y_idx>(lp0);
					z(p1(l)) = z<z_idx>(lp1);
				}
				p.push_back(l);
			};
			poly_iter(polys.at(i), fn);
			polygons.push_back(p);
			// this->add_polygons<x_idx,y_idx,p0_idx,p1_idx>(choice<0>{},polys.at(i));
		}
	}

	template <size_t x_idx=0, size_t y_idx=1, size_t z_idx=2, size_t p0_idx=0, size_t p1_idx=1>
	inline
	auto add_polygons(choice<1>, const auto &polys) -> decltype(x(p0(elements::line(polys,0))), void())
	{
		using line_t = decltype(elements::line(polys,0));
		Polygon<scalar> p;
		// auto fn = [&](const auto &line) {
		auto fn = [&](const line_t &line) {
			Line<scalar,dim> l;
			x(p0(l)) = x<x_idx>(p0<p0_idx>(line));
			x(p1(l)) = x<x_idx>(p1<p1_idx>(line));
			y(p0(l)) = y<y_idx>(p0<p0_idx>(line));
			y(p1(l)) = y<y_idx>(p1<p1_idx>(line));
			if constexpr(dim == 3) {
				z(p0(l)) = z<y_idx>(p0<p0_idx>(line));
				z(p1(l)) = z<z_idx>(p1<p1_idx>(line));
			}
			p.push_back(l);
		};
		poly_iter(polys, fn);
		polygons.push_back(p);
	}

	// template <size_t x_idx=0, size_t y_idx=1, size_t p0_idx=0, size_t p1_idx=1>
	template <size_t ...args>
	inline
	auto add_polygons(const auto &polys) -> decltype(add_polygons(choice<1> {}, polys))
	{
		add_polygons<args...>(choice<1> {}, polys);
	}

	template <size_t x_idx=0, size_t y_idx=1, size_t r_idx=2>
	inline
	auto add_circles(choice<1>, const auto &circles) -> decltype(circles.size(), x(circles.at(0)), y(circles.at(0)), radius(circles.at(0)), void())
	{
		for(size_t i=0; i<circles.size(); i++) {
			add_circles<x_idx,y_idx,r_idx>(choice<0> {}, circles.at(i));
		}
	}

	template <size_t x_idx=0, size_t y_idx=1, size_t r_idx=2>
	inline
	auto add_circles(choice<0>, const auto &circles) -> decltype(x(circles), y(circles), radius(circles), void())
	{
		Circle<scalar> c;
		x(c) = x<x_idx>(circles);
		y(c) = y<y_idx>(circles);
		radius(c) = radius<r_idx>(circles);
		this->circles.push_back(c);
	}

	template <size_t x_idx=0, size_t y_idx=1, size_t r_idx=3>
	inline
	auto add_circles(const auto &circles) -> decltype(add_circles(choice<1> {},circles))
	{
		add_circles<x_idx,y_idx,r_idx>(choice<1> {},circles);
	}

	// template <size_t x_idx=0, size_t y_idx=1, size_t p0_idx=0, size_t p1_idx=1>
	template <size_t ...args>
	inline
	auto collide(choice<3>, const auto &polygon) const -> decltype(poly_poly_collision<args...>(polygon, polygons.at(0)), true)
	{
		auto collide = false;
		for(const auto &p : polygons) {
			// collide = poly_poly_collision<x_idx,y_idx,p0_idx,p1_idx>(polygon, p) ? true : collide;
			collide = poly_poly_collision<args...>(polygon, p) ? true : collide;
		}
		return collide;
	}

	// template <size_t x_idx=0, size_t y_idx=1, size_t r_idx=2, size_t p0_idx=0, size_t p1_idx=1>
	template <size_t ...args>
	inline
	auto collide(choice<2>, const auto &circle) const -> decltype(circle_poly_collision<args...>(circle, polygons.at(0)), true)
	{
		auto collide = false;
		for(const auto &p : polygons) {
			// collide = circle_poly_collision<x_idx,y_idx,r_idx,p0_idx,p1_idx>(circle, p) ? true : collide;
			collide = circle_poly_collision<args...>(circle, p) ? true : collide;
		}
		return collide;
	}

	// template <size_t x_idx=0, size_t y_idx=1, size_t p0_idx=0, size_t p1_idx=1>
	template <size_t ...args>
	inline
	auto collide(choice<1>, const auto &line) const -> decltype(line_poly_collision<args...>(line, polygons.at(0)), true)
	{
		auto collide = false;
		for(const auto &p : polygons) {
			// collide = line_poly_collision<x_idx,y_idx>(line, p) ? true : collide;
			collide = line_poly_collision<args...>(line, p) ? true : collide;
		}
		return collide;
	}

	// template <size_t x_idx=0, size_t y_idx=1>
	template <size_t ...args>
	inline
	auto collide(choice<0>, const auto &point) const -> decltype(point_inside_poly<args...>(point, polygons.at(0)), true)
	{
		auto collide = false;
		for(const auto &p : polygons) {
			// collide = point_inside_poly<x_idx,y_idx>(point, p) ? true : collide;
			collide = point_inside_poly<args...>(point, p) ? true : collide;
		}
		return collide;
	}

	template <size_t ...args>
	inline
	auto collide(const auto &object) const -> decltype(collide<args...>(choice<3> {}, object), true)
	{
		return collide<args...>(choice<3> {}, object);
	}
};

}

#endif // ENVIRONMENT_HPP
