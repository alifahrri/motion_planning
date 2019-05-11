#ifndef COLLISION_HPP
#define COLLISION_HPP

#include <vector>
#include <math.h>
#include <functional>
#include <type_traits>

#include "elements.hpp"

#ifdef GPU
#include <cuda.h>
#include <cuda_runtime.h>
#include "util.cuh"

#define HOST __host__
#define DEVICE __device__
#define ATTRIBUTE __host__ __device__
#else
#define HOST
#define DEVICE
#define ATTRIBUTE
#endif

#ifndef NDEBUG
#ifdef __NVCC__
#include "util.h"
#define TRACE_CUDA
#endif
#endif


template <int x_idx, int y_idx, typename point_t, typename Iterable, typename scalar>
ATTRIBUTE
inline
bool point_circle_collision(const point_t &pt, const Iterable &obs, const scalar &collision_radius)
{
	bool collision = false;
	// for(size_t i=0; i<obs.size(); i++)
	for(const auto &o : obs) {
		// auto dx = pt(x_idx) - std::get<0>(o);
		// auto dy = pt(y_idx) - std::get<1>(o);
		auto dx = pt(x_idx) - o(x_idx);
		auto dy = pt(y_idx) - o(y_idx);
		auto r = sqrt(dx*dx+dy*dy);
		if(r <= collision_radius) {
			collision = true;
			break;
		}
	}
	return collision;
}

// see http://paulbourke.net/geometry/pointlineplane/
template <int x_idx, int y_idx, typename p1_t, typename p2_t, typename Iterable, typename scalar>
ATTRIBUTE
inline
bool line_circle_collision(const p1_t &pt0, const p2_t &pt1, const Iterable &obs, const scalar &collision_radius)
{
#ifdef TRACE_CUDA
	TRACE_KERNEL(blockIdx.x * blockDim.x + threadIdx.x, 0, __PRETTY_FUNCTION__);
#endif
	scalar x0, x1, y0, y1;
	bool collision = false;
	// x0 = pt0(x_idx); y0 = pt0(y_idx);
	// x1 = pt1(x_idx); y1 = pt1(y_idx);
	x0 = elements::x<x_idx>(pt0);
	y0 = elements::y<y_idx>(pt0);
	x1 = elements::x<x_idx>(pt1);
	y1 = elements::y<y_idx>(pt1);
	if(isnan(x0) || isnan(x1) || isnan(y0) || isnan(y1))
		return true;
	//  for(size_t i=0; i<obs.size(); i++)
	for(const auto &o : obs) {
		// auto cx = std::get<0>(o);
		// auto cy = std::get<1>(o);
		// auto cx = o(x_idx);
		// auto cy = o(y_idx);
		auto cx = elements::x<x_idx>(o);
		auto cy = elements::x<y_idx>(o);
		// auto dxc0 = pt0(x_idx) - cx; auto dxc1 = pt1(x_idx) - cx;
		// auto dyc0 = pt0(y_idx) - cy; auto dyc1 = pt1(y_idx) - cy;
		auto dxc0 = elements::x<x_idx>(pt0) - cx;
		auto dxc1 = elements::x<x_idx>(pt1) - cx;
		auto dyc0 = elements::y<y_idx>(pt0) - cy;
		auto dyc1 = elements::y<y_idx>(pt1) - cy;
		auto r = sqrt(dxc0*dxc0+dyc0*dyc0);
		if(r <= collision_radius) {
			collision = true;
			break;
		}
		r = sqrt(dxc1*dxc1+dyc1*dyc1);
		if(r <= collision_radius) {
			collision = true;
			break;
		}
		auto dx = (x1-x0);
		auto dy = (y1-y0);
		auto u = ((cx-x0)*(x1-x0) + (cy-y0)*(y1-y0)) / (dx*dx+dy*dy);
		if(u<scalar(0.0) || u>scalar(1.0))
			continue;
		auto x = x0+u*(dx);
		auto y = y0+u*(dy);
		auto dxr = x-cx;
		auto dyr = y-cy;
		r = sqrt(dxr*dxr+dyr*dyr);
		if(r <= collision_radius) {
			collision = true;
			break;
		}
	}
#ifdef TRACE_CUDA
	TRACE_KERNEL(blockIdx.x * blockDim.x + threadIdx.x, 0, __FUNCTION__, "OK");
#endif
	return collision;
}

template <int x_idx=0, int y_idx=1, int segment = 10, typename p1_t, typename p2_t, typename scalar, typename ArrayLike>
inline
bool parametrized_line_circle_collision(const p1_t &p1, const p2_t &p2, const scalar &t0, const scalar &t1, const ArrayLike &obs, const scalar &collision_radius)
{
	auto collision = false;
	scalar dt = (t1-t0)/segment;
	auto dpx = p2(x_idx) - p1(x_idx);
	auto dpy = p2(y_idx) - p1(y_idx);
	auto p1_test = p1;
	auto p2_test = p1;
	p1_test(x_idx) = p1(x_idx);
	p1_test(y_idx) = p1(y_idx);
	using ObsType = std::decay_t<decltype(obs[0])>;
	// auto obs_t0 = obs;
	// auto obs_t1 = obs;
	std::vector<ObsType> obs_t0, obs_t1;
	obs_t0.resize(obs.size());
	obs_t1.resize(obs.size());
	for(size_t i=1; i<=segment; i++) {
		scalar ti = t0+(i-1)*dt;
		scalar tf = ti+dt;
		for(size_t j=0; j<obs.size(); j++) {
			obs_t0[j] = obs[j](ti);
			obs_t1[j] = obs[j](tf);
		}
		p2_test(x_idx) = p1(x_idx) + (scalar(i) / segment * dpx);
		p2_test(y_idx) = p1(y_idx) + (scalar(i) / segment * dpy);
		if(line_circle_collision<x_idx,y_idx>(p1_test, p2_test, obs_t0, collision_radius) ||
		   line_circle_collision<x_idx,y_idx>(p1_test, p2_test, obs_t1, collision_radius) ) {
			collision = true;
			break;
		}
		p1_test = p2_test;
	}
	return collision;
}

// type agnostic collision check functions
namespace collision
{

// implementation of line collision in 3d,
// overloaded with template parameter
template <int x_idx, int y_idx, int z_idx>
ATTRIBUTE
inline
auto line_line_collision_(const auto &pt0, const auto &pt1, const auto &pt2, const auto &pt3)
-> decltype(elements::x(pt0), elements::y(pt0), elements::z(pt0), 
						elements::x(pt1), elements::y(pt1), elements::z(pt1), 
						elements::x(pt2), elements::y(pt2), elements::z(pt2), 
						elements::x(pt3), elements::y(pt3), elements::z(pt3), 
						bool{})
{
	
}

// implementation of line collision in 2d,
// overloaded with template parameter
// see https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
template <int x_idx, int y_idx>
ATTRIBUTE
inline
auto line_line_collision_(const auto &pt0, const auto &pt1, const auto &pt2, const auto &pt3)
-> decltype(elements::x(pt0), elements::y(pt0), 
						elements::x(pt1), elements::y(pt1), 
						elements::x(pt2), elements::y(pt2), 
						elements::x(pt3), elements::y(pt3), 
						bool{})
{
	using elements::x;
	using elements::y;
	auto on_segment = [](const auto &p, const auto &q, const auto &r) {
		return ((x(q)<=std::max(x<x_idx>(p),x<x_idx>(r))) && (x<x_idx>(q)>=std::min(x<x_idx>(p),x<x_idx>(r))) &&
		        (y<y_idx>(q)<=std::max(y<y_idx>(p),y<y_idx>(r))) && (y<y_idx>(q)>=std::min(y<y_idx>(p),y<y_idx>(r))));
	};
	auto orientation = [](const auto &p, const auto &q, const auto &r) {
		auto dy1 = y<y_idx>(q)-y<y_idx>(p);
		auto dy2 = y<y_idx>(r)-y<y_idx>(q);
		auto dx1 = x<x_idx>(r)-x<x_idx>(q);
		auto dx2 = x<x_idx>(q)-x<x_idx>(p);
		int val = dy1*dx1 - dx2*dy2;
		// int val = dy(q,p) * dx(r,q) - dx(q,p) * dy(r,q);
		return ((val == 0) ? 0 : (val > 0 ? 1 : 2));
	};
	auto o1 = orientation(pt0,pt1,pt2);
	auto o2 = orientation(pt0,pt1,pt3);
	auto o3 = orientation(pt2,pt3,pt0);
	auto o4 = orientation(pt2,pt3,pt1);
	if((o1!=o2) && (o3!=o4)) return true;
	if(o1==0 && on_segment(pt0,pt2,pt1)) return true;
	if(o2==0 && on_segment(pt0,pt3,pt1)) return true;
	if(o3==0 && on_segment(pt2,pt0,pt3)) return true;
	if(o4==0 && on_segment(pt2,pt1,pt3)) return true;
	return false;
}

// entry-point for line collision!
// TODO : support for SFINAE!
template <int ...indexes>
ATTRIBUTE
inline
auto line_line_collision(const auto &pt0, const auto &pt1, const auto &pt2, const auto &pt3)
{
	constexpr auto dim = sizeof...(indexes);
	// warn : if not specified, default call to 2d!
//	if constexpr(dim == 0) {
//		return line_line_collision_<0,1>(pt0,pt1,pt2,pt3);
//	} else if(dim == 2) {
//		return line_line_collision_<indexes...>(pt0,pt1,pt2,pt3);
//	} else if(dim == 3) {
//		return line_line_collision_<indexes...>(pt0,pt1,pt2,pt3);
//	} else {
//		// TODO : generate compile error!
//	}
	if constexpr(dim == 0) {
		return line_line_collision_<0,1>(pt0, pt1, pt2, pt3);
	} else {
		return line_line_collision_<indexes...>(pt0, pt1, pt2, pt3);
	}
}

// template <int x_idx=0, int y_idx=1, int z_idx=2, int p0_idx=0, int p1_idx=1>
template <int ...indexes>
ATTRIBUTE
inline
auto line_line_collision(const auto &line0, const auto &line1)
// all of these expression must be valid :
//-> decltype(elements::p0<p0_idx>(line0), elements::p1<p1_idx>(line0), 
//						elements::p0<p0_idx>(line1), elements::p1<p1_idx>(line1), 
//						line_line_collision(
//							elements::p0<p0_idx>(line0), elements::p1<p1_idx>(line0),
//							elements::p0<p0_idx>(line1), elements::p1<p1_idx>(line1)
//						),
//						bool{})
{
	using elements::p0;
	using elements::p1;
	constexpr auto dim = sizeof...(indexes);
	// constexpr std::array<int,dim> index_array(indexes...);
	constexpr int idx[dim] = {indexes...};
	if constexpr((dim == 0) || (dim == 2) || (dim == 4)) {
		// this should call 2d line collision :
		if constexpr(dim == 0) {
			return line_line_collision<0,1>(p0(line0),p1(line0),p0(line1),p1(line1));
		} else if(dim == 2) {
			return line_line_collision<idx[0],idx[1]>(p0(line0),p1(line0),p0(line1),p1(line1));
		} else if(dim == 4) {
			return line_line_collision<idx[0],idx[1]>(
												p0<idx[2]>(line0),p1<idx[3]>(line0),
												p0<idx[2]>(line1),p1<idx[3]>(line1)
											);
		}
	} else if((dim == 3) || (dim == 5)) {
		// this should call 3d line collision :
		if constexpr(dim == 3) {
			return line_line_collision<idx[0],idx[1],idx[2]>(p0(line0),p1(line0),p0(line1),p1(line1));
		} else if(dim == 5) {
			return line_line_collision<idx[0],idx[1],idx[2]>(
												p0<idx[3]>(line0),p1<idx[4]>(line0),
												p0<idx[3]>(line1),p1<idx[4]>(line1)
											);
		}
	} else {
		// TODO : generate compile error!
	}
}

// helper compile-time function
template <size_t n, int ...values>
constexpr auto get_n_values(size_t idx) {
	constexpr int v[sizeof...(values)] = {values...};
	constexpr auto dim = n;
	std::array<int,dim> ret;
	for(size_t i=0; (i<n) && (i+idx < sizeof...(values)); i++)
		ret[i] = v[i+idx];
	return ret;
}
// helper compile-time function
template <int ...indexes>
constexpr auto get_indexes() {
	constexpr auto dim = sizeof...(indexes);
	// std::pair<int,int> dims;
	// dims.second = 2; // line indexes
	constexpr std::array<int,2> def_pt_idx = {0,1};
	constexpr std::array<int,2> def_ln_idx = {0,1};
	if constexpr(dim == 0) {
		return std::make_pair(def_pt_idx,def_ln_idx);
	} else if(dim == 2) {
		return std::make_pair(
			get_n_values<2,indexes...>(0),
			def_ln_idx
		);
	} else if(dim == 4) {
		return std::make_pair(
			get_n_values<2,indexes...>(0),
			get_n_values<2,indexes...>(2)
		);
	} else if(dim == 3) {
		return std::make_pair(
			get_n_values<3,indexes...>(0),
			def_ln_idx
		);
	} else if(dim == 5) {
		return std::make_pair(
			get_n_values<3,indexes...>(0),
			get_n_values<2,indexes...>(3)
		);
	}
// the following expr need to have consistent return value
//	switch(dim) {
//		case 0 : 
//			return std::make_pair(def_pt_idx,def_ln_idx);
//		case 2 : 
//			return std::make_pair(
//				get_n_values<2,indexes...>(0),
//				def_ln_idx
//			);
//		case 4 :
//			return std::make_pair(
//				get_n_values<2,indexes...>(0),
//				get_n_values<2,indexes...>(2)
//			);
//		case 3 :
//			return std::make_pair(
//				get_n_values<3,indexes...>(0),
//				def_ln_idx
//			);
//		case 5 :
//			return std::make_pair(
//				get_n_values<3,indexes...>(0),
//				get_n_values<2,indexes...>(3)
//			);
//	}
	// constexpr auto point_idx = get_n_values<indexes...>(0, dims.first);
	// constexpr auto line_idx = get_n_values<indexes...>(dims.first, dims.second);
	// return std::make_pair(point_idx, line_idx);
}

// template <int x_idx=0, int y_idx=1, int z_idx=2, int p0_idx=0, int p1_idx=1>
template <int ...indexes>
ATTRIBUTE
inline
auto line_poly_collision(const auto &line, const auto &poly)
-> decltype(elements::line(poly, 0), bool{})
{
	bool collision = false;
	auto check = [&](const auto &pline) {
		collision = line_line_collision<indexes...>(line, pline) ? true : collision;
	};
	elements::poly_iter(poly, check);
	return collision;
}

// template <int x_idx=0, int y_idx=1, int p0_idx=0, int p1_idx=1>
template <int ...indexes>
ATTRIBUTE
inline
auto poly_poly_collision(const auto &poly0, const auto &poly1)
-> decltype(elements::line(poly0, 0), elements::line(poly1, 0), bool{})
{
	bool collision = false;
	auto check = [&](const auto &line0) {
		collision = (line_poly_collision<indexes...>(line0, poly1) ? true : collision);
	};
	elements::poly_iter(poly0, check);
	return collision;
}

// template <int x_idx=0, int y_idx=1, int p0_idx=0, int p1_idx=1>
template <int ...indexes>
ATTRIBUTE
inline
auto point_inside_poly(const auto &pt, const auto &poly)
// TODO : support sfinae
// -> decltype(elements::x<x_idx>(pt), elements::y<y_idx>(pt), elements::line(poly, 0), bool{})
{
	constexpr auto idx = get_indexes<indexes...>();
	if constexpr(idx.first.size() == 2) {		
		using pt_t = std::remove_cv_t<std::remove_reference_t<decltype(pt)>>;
		pt_t test_line[2];
		// it turns out that these structured binding (with constexpr)
		// were not allowed at the moment
		// constexpr auto [x_idx, y_idx] = idx.first;
		// constexpr auto [p0_idx, p1_idx] = idx.second;
		constexpr size_t x_idx = idx.first[0];
		constexpr size_t y_idx = idx.first[1];
		constexpr size_t p0_idx = idx.second[0];
		constexpr size_t p1_idx = idx.second[1];
		elements::x<x_idx>(test_line[0]) = 1000000;
		elements::y<y_idx>(test_line[0]) = elements::y<y_idx>(pt);
		elements::x<x_idx>(test_line[1]) = elements::x<x_idx>(pt);
		elements::y<y_idx>(test_line[1]) = elements::y<y_idx>(pt);
		size_t count = 0;
		auto check = [&](const auto &line) {
			if(line_line_collision<x_idx, y_idx, p0_idx, p1_idx>(line, test_line))
				count++;
		};
		elements::poly_iter(poly, check);
		return bool(count&1);
	} else {
		// TODO : provide 3d version
	}
}

template <int x_idx=0, int y_idx=1, int r_idx=2, int p0_idx=0, int p1_idx=1>
ATTRIBUTE
inline
auto circle_poly_collision(const auto &circle, const auto &poly)
-> decltype(elements::radius<r_idx>(circle), 
						elements::x<x_idx>(circle), 
						elements::y<y_idx>(circle), 
						elements::line(poly, 0), 
						bool{})
{
	bool collision = false;
	using circle_t = std::remove_cv_t<std::remove_reference_t<decltype(circle)>>;
	auto iterable_circle = std::vector<circle_t>();
	iterable_circle.push_back(circle);
	if(point_inside_poly(circle, poly))
		return true;
	auto check = [&](const auto &line) {
		const auto& p0 = elements::p0<p0_idx>(line);
		const auto& p1 = elements::p1<p1_idx>(line);
		const auto& radius = elements::radius<r_idx>(circle);
		collision = line_circle_collision<x_idx, y_idx>(p0, p1, iterable_circle, radius) ? true : collision;
	};
	elements::poly_iter(poly, check);
	return collision;
}

template <int x_idx=0, int y_idx=1, int p0_idx=0, int p1_idx=1>
// ATTRIBUTE
inline
auto line_grid_collision(const auto &line, const auto &grid) 
-> decltype(elements::p0<p0_idx>(line), 
						elements::p1<p1_idx>(line), 
						bool{})
{
	const auto& p0 = elements::p0<p0_idx>(line);
	const auto& p1 = elements::p1<p1_idx>(line);
	return grid.collide(p0, p1);
}

} // namespace collision

#endif // COLLISION_HPP
