#ifndef TEST_DATA_H
#define TEST_DATA_H

#include "elements.hpp"
#include "collision.hpp"
#include "environment.hpp"
#include "gridmap.hpp"

// for gridmap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

struct pt_0t {
	double xv=1.0;
	double yv=2.0;
	auto& x() const
	{
		return xv;
	}
	auto& y() const
	{
		return yv;
	}
	auto& x()
	{
		return xv;
	}
	auto& y()
	{
		return yv;
	}
	inline
	bool operator==(const pt_0t &rhs) const
	{
		return (x()==rhs.x()) && (y()==rhs.y());
	}
	inline
	bool operator!=(const pt_0t &rhs) const
	{
		return !(*this == rhs);
	}
};

struct pt_1t {
	double x=1.0;
	double y=2.0;
	inline
	bool operator==(const pt_1t &rhs) const
	{
		return (x==rhs.x) && (y==rhs.y);
	}
	inline
	bool operator!=(const pt_1t &rhs) const
	{
		return !(*this == rhs);
	}
};

struct ln_0t {
	pt_0t pt0;
	pt_0t pt1;
	auto& p0() const
	{
		return pt0;
	}
	auto& p1() const
	{
		return pt1;
	}
	auto& p0()
	{
		return pt0;
	}
	auto& p1()
	{
		return pt1;
	}
	inline
	bool operator == (const ln_0t &rhs) const
	{
		return (pt0 == rhs.pt0) && (pt1 == rhs.pt1);
	}
	inline
	bool operator != (const ln_0t &rhs) const
	{
		return !(*this == rhs);
	}
};

struct ln_1t {
	pt_1t p0;
	pt_1t p1;
	inline
	bool operator == (const ln_1t &rhs) const
	{
		return (p0 == rhs.p0) && (p1 == rhs.p1);
	}
	inline
	bool operator != (const ln_1t &rhs) const
	{
		return !(*this == rhs);
	}
};

struct poly_0t {
	std::vector<ln_0t> lines;
};

struct poly_1t {
	std::vector<ln_1t> line_vector;
	auto& lines() const
	{
		return line_vector;
	}
	auto& lines()
	{
		return line_vector;
	}
};

struct circle_t {
	double x = 0.0;
	double y = 0.0;
	double r = 1.0;
	inline
	bool operator == (const circle_t &rhs) const
	{
		return x==rhs.x && y==rhs.y && r==rhs.r;
	}
	inline
	bool operator != (const circle_t &rhs) const
	{
		return !(*this == rhs);
	}
};

namespace test_data
{
struct TestData {
	ln_0t l0;
	ln_1t l1;
	ln_1t l2;
	pt_0t p;
	pt_1t p1;
	std::vector<ln_0t> lines0;
	std::vector<ln_1t> lines1;
	std::vector<ln_1t> lines2;
	poly_0t poly0;
	poly_1t poly1;
	poly_1t poly2;
	pt_0t pt0;
	pt_1t pt1;
	circle_t c0;
	circle_t c1;
	std::vector<poly_0t> poly0_vec;
	std::vector<poly_1t> poly1_vec;

	// octomap::OcTree octree(0.1);
	// gridmap::OctomapWrapper<octomap::OcTree,gridmap::los_method::avg_occupancy> octo_wrapper(octree);
	octomap::OcTree octree;
	typedef gridmap::OctomapWrapper<octomap::OcTree,gridmap::los_method::avg_occupancy> gridmap_t;
	gridmap_t octo_wrapper;
	
	TestData() 
		: octree(0.1)
		, octo_wrapper(octree)
	{
		init();
	}

	void init()
	{
		l0.pt0.xv = 1.0;
		l0.pt0.yv = 1.0;
		l0.pt1.xv = -1.0;
		l0.pt1.yv = -1.0;
		l1.p0.x = 1.0;
		l1.p0.y = -1.0;
		l1.p1.x = -1.0;
		l1.p1.y = 1.0;
		l2.p0.x = -1.0;
		l2.p0.y = 0.0;
		l2.p1.x = -2.0;
		l2.p1.y = 1.0;

		pt0.xv = 0.0;
		pt0.yv = 0.0;
		pt1.x = 3.0;
		pt1.y = 0.0;

		c1.r = 0.25;
		c1.x = 3.0;

		{
			ln_0t l;
			l.pt0.xv = -1.;
			l.pt0.yv = -1.;
			l.pt1.xv = -1.;
			l.pt1.yv = 1.;
			lines0.push_back(l);
			l.pt0.xv = -1.;
			l.pt0.yv = 1.;
			l.pt1.xv = 1.;
			l.pt1.yv = 1.;
			lines0.push_back(l);
			l.pt0.xv = 1.;
			l.pt0.yv = 1.;
			l.pt1.xv = 1.;
			l.pt1.yv = -1.;
			lines0.push_back(l);
			l.pt0.xv = 1.;
			l.pt0.yv = -1.;
			l.pt1.xv = -1.;
			l.pt1.yv = -1.;
			lines0.push_back(l);
			for(const auto &line : lines0) {
				poly0.lines.push_back(line);
			}
		}

		{
			ln_1t l;
			l.p0.x = 0.;
			l.p0.y = 0.;
			l.p1.x = 0.;
			l.p1.y = 2.;
			lines1.push_back(l);
			l.p0.x = 0.;
			l.p0.y = 2.;
			l.p1.x = 2.;
			l.p1.y = 2.;
			lines1.push_back(l);
			l.p0.x = 2.;
			l.p0.y = 2.;
			l.p1.x = 2.;
			l.p1.y = 0.;
			lines1.push_back(l);
			l.p0.x = 2.;
			l.p0.y = 0.;
			l.p1.x = 0.;
			l.p1.y = 0.;
			lines1.push_back(l);
			for(const auto &line : lines1) {
				poly1.line_vector.push_back(line);
			}
		}

		{
			ln_1t l;
			l.p0.x = 3.;
			l.p0.y = 0.;
			l.p1.x = 3.;
			l.p1.y = 2.;
			lines2.push_back(l);
			l.p0.x = 3.;
			l.p0.y = 2.;
			l.p1.x = 5.;
			l.p1.y = 2.;
			lines2.push_back(l);
			l.p0.x = 5.;
			l.p0.y = 2.;
			l.p1.x = 5.;
			l.p1.y = 0.;
			lines2.push_back(l);
			l.p0.x = 5.;
			l.p0.y = 0.;
			l.p1.x = 3.;
			l.p1.y = 0.;
			lines2.push_back(l);
			for(const auto &line : lines2) {
				poly2.line_vector.push_back(line);
			}
		}

		//
		for(size_t i=0; i<4; i++) {
			poly0_vec.push_back(poly0);
			poly1_vec.push_back(poly1);
			auto p0 = poly0_vec.at(i);
			auto p1 = poly1_vec.at(i);
		}
		//
		{
			// gridmap initialization
			octo_wrapper.set_min_collision_prob(0.35);
			for (int x=-20; x<20; x++) {
				for (int y=-20; y<20; y++) {
					for (int z=-20; z<20; z++) {
						octomap::point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
						octree.updateNode(endpoint, false);
					}
				}
			}
			for (int x=-30; x<30; x++) {
				for (int y=-30; y<30; y++) {
					for (int z=-30; z<30; z++) {
						octomap::point3d endpoint ((float) x*0.02f-1.0f, (float) y*0.02f-1.0f, (float) z*0.02f-1.0f);
						octree.updateNode(endpoint, true);
					}
				}
			}

		}
	}
};
}

#endif
