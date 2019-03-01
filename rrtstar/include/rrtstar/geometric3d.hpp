#ifndef GEOMETRIC3D_HPP
#define GEOMETRIC3D_HPP

#include <cmath>
#include <memory>
#include <type_traits>

#include "sampler.hpp"
#include "kdtree.hpp"
#include "rrtstar.hpp"
#include "rrtvisual.hpp"
#include "integrator2d.hpp"
#include "gridmap.hpp"

#define SAMPLE_SCALE 5.0

namespace Geometric
{

typedef Point<double,3> Point3D;
typedef PointCloud<double,3> PointCloud3D;
typedef KDTree<PointCloud3D,3,double,Point3D> KDTree3D;
typedef double CostType;

struct State3D {
	// State3D() : p(Point3D()), c(CostType(0.0)) {}
	State3D() {}
	// State3D(Point3D *p, CostType *c) : p(p), c(c) {}
	State3D(std::shared_ptr<Point3D> p, std::shared_ptr<CostType> c) : p(p), c(c) {}
	inline
	bool operator==(State3D &rhs)
	{
		auto same = true;
		for(size_t i=0; i<3; i++) {
			same = ((*p)[i] == (*(rhs.p))[i]);
			if(!same) break;
		}
		return same;
	}
	inline
	bool operator!=(State3D &rhs)
	{
		auto same = (*this)==rhs;
		return !same;
	}
	std::shared_ptr<Point3D> p;
	std::shared_ptr<CostType> c;
	CostType cost() const
	{
		return *c;
	}
	void setCost(const CostType &cost)
	{
		*c = cost;
	}
};

struct Cost3D {
	typedef CostType Scalar;
	Cost3D() {}
	Scalar operator()(const State3D &s0, const State3D &s1)
	{
		ROS_INFO("computing cost");
		auto dx = s1.p->p[0]-s0.p->p[0];
		auto dy = s1.p->p[1]-s0.p->p[1];
		auto dz = s1.p->p[2]-s0.p->p[2];
		return std::sqrt(dx*dx+dy*dy+dz*dz);
	}
};

Cost3D cost3d;

struct Connector3D {
	typedef std::array<Point3D,2> Edge;
	Connector3D() {}
	Edge operator()(const State3D &s0, const State3D &s1) const
	{
		ROS_INFO("connecting states");
		Edge e;
		e.front() = *s0.p;
		e.back() = *s1.p;
		return e;
	}
};

Connector3D connector;

struct Tree3D {
	typedef int Index;
	typedef std::vector<Index> IndexList;
	typedef State3D State;

	Tree3D() {}
	IndexList nearest(const State &s, const double &radius)
	{
		ROS_INFO("finding nearest states");
		IndexList ret;
		auto vp = tree.nearest(*(s.p),radius);
		for(const auto& p : vp)
			ret.push_back(Index(p.first));
		return ret;
	}

	Index insert(const State& s, const Index &idx)
	{
		ROS_INFO("inserting states");
		tree.addPoint(*(s.p));
		costs.push_back(*(s.c));
		parent.push_back(idx);
		return tree.size()-1;
	}

	Index insert(const State& s, const Index &idx, const Connector3D::Edge &e)
	{
		ROS_INFO("inserting states");
		tree.addPoint(*(s.p));
		costs.push_back(*(s.c));
		parent.push_back(idx);
		return tree.size()-1;
	}

	void reset()
	{
		ROS_INFO("resetting tree");
		costs.clear();
		parent.clear();
		tree.clear();
	}

	void setParent(const Index& node, const Index &p)
	{
		// ROS_INFO("setting parent");
		parent[node] = p;
	}

	void setEdge(const Index &node, const Connector3D::Edge &e)
	{

	}

	State operator()(const Index &i)
	{
		State s(std::make_shared<Point3D>(tree(i)), std::make_shared<CostType>(costs[i]));
		return s;
	}

	std::vector<CostType> costs;
	KDTree3D tree;
	IndexList parent;
};

Tree3D tree3d;

template <typename grid_map_t = nullptr_t>
struct CollisionChecker {
	CollisionChecker(const grid_map_t &map)
		: map(map)
	{}
	CollisionChecker()
		: map(nullptr)
	{}
	inline
	bool operator ()(const Connector3D::Edge &e)
	{
		ROS_INFO("checking collision");
		return (*this)(e.front(),e.back());
		// for now, obstacle-free env
		// return false;
	}
	inline
	bool operator() (const auto &s0, const auto &s1)
	{
		ROS_INFO("checking collision");
		if constexpr(std::is_same_v<grid_map_t,nullptr_t>) {
			// for now, obstacle-free env
			return false;
		} else {
			return map.collide(s0,s1);
		}
	}
	const grid_map_t &map;
};

CollisionChecker checker;

struct Sampler : mpl::Sampler<3,double> {
	Sampler()
		: mpl::Sampler<3,double>()
	{
		using std::vector;
		this->set_bound(
			vector{-SAMPLE_SCALE/2,-SAMPLE_SCALE/2,0.0},
			vector{SAMPLE_SCALE/2,SAMPLE_SCALE/2,SAMPLE_SCALE}
		);
	}
	State3D operator()()
	{
		ROS_INFO("sample states");
		std::shared_ptr<Point3D> pt(new Point3D());
		std::shared_ptr<CostType> c(new CostType(0.0));
		mpl::Sampler<3,double>::operator()(pt->p);
		return State3D(pt,c);
	}
};

Sampler sampler;

struct NeighborRadius {
	NeighborRadius()
	{
		s = std::pow(2,3)*(1+1/3)*SAMPLE_SCALE*SAMPLE_SCALE*SAMPLE_SCALE;
	}
	double operator()(const Tree3D::Index &i)
	{
		ROS_INFO("computing radius, idx : %d", i);
		return s*std::log(i+1)/(i+1);
	}
	double s;
};

NeighborRadius radius;

struct GoalChecker {
	GoalChecker() {}
	bool operator()(const State3D &state)
	{
		ROS_INFO("checking goal");
		// for now, iterate indefinetly
		return false;
	}
};

GoalChecker goal;

typedef RRTStar<Tree3D,Cost3D,Sampler,NeighborRadius,CollisionChecker<>,GoalChecker,Connector3D> RRTStar3D;

RRTStar3D rrtstar_3d(tree3d, cost3d, sampler, checker, radius, goal, connector);

class GeomRRTStar3D
{
	typedef gridmap::Publisher<octomap::OcTree> publisher_t;
	typedef gridmap::OctomapWrapper<octomap::OcTree,gridmap::los_method::avg_occupancy> gridmap_t;
	typedef CollisionChecker<gridmap_t> collision_t;
	typedef RRTStar<Tree3D,Cost3D,Sampler,NeighborRadius,collision_t,GoalChecker,Connector3D> rrtstar_t;
public:
	Tree3D *tree3d;
	Cost3D *cost3d;
	Sampler *sampler;
	NeighborRadius *radius;
	collision_t *checker;
	GoalChecker *goal;
	Connector3D *connector;
	rrtstar_t *rrtstar_3d;
	gridmap_t octowrapper;
	publisher_t pub;
private :
	void init()
	{
		tree3d = new Tree3D;
		cost3d = new Cost3D;
		sampler = new Sampler;
		radius = new NeighborRadius;
		checker = new collision_t(octowrapper);
		goal = new GoalChecker;
		connector = new Connector3D;
		rrtstar_3d = new rrtstar_t(*tree3d, *cost3d, *sampler, *checker, *radius, *goal, *connector);
	}
public :
	GeomRRTStar3D(ros::NodeHandle &node, octomap::OcTree &octree)
		: octowrapper(gridmap_t(octree))
		, pub(publisher_t(node, octree, "/octomap_fullmap"))
	{
		this->init();
	}
	inline
	auto setStart(const Geometric::State3D &start)
	{
		this->rrtstar_3d->setStart(start);
	}
	inline
	auto setStart(const Geometric::Point3D &point, const Geometric::CostType &cost)
	{
		using Geometric::State3D;
		using Geometric::Point3D;
		using Geometric::CostType;
		this->setStart(State3D(std::make_shared<Point3D>(point),std::make_shared<CostType>(cost)));
	}
};
}

#endif // GEOMETRIC3D_HPP