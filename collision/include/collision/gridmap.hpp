#ifndef GRIDMAP_HPP
#define GRIDMAP_HPP

#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <iostream>
#include <type_traits>
#include <ros/ros.h>

#include "elements.hpp"

namespace gridmap
{

template <typename tree_t>
class OctomapWrapper
{
private:
	tree_t &tree;
	double min_collide_occupancy = 0.2;

private:
	// dummy fn, should never be called at runtime
	// for deduction purpose only
	constexpr auto search_expr()
	{
		if constexpr(!std::is_pointer_v<tree_t>)
			return this->tree.search(octomap::point3d());
		else return this->tree->search(octomap::point3d());
	}
	using key_t = decltype(std::declval<OctomapWrapper<tree_t>>().search_expr());

public:
	OctomapWrapper(tree_t &tree)
		: tree(tree)
	{

	}

	~OctomapWrapper()
	{
		// TODO : delete the tree if tree_t is a pointer type
	}

	/* set the minimum probability for each cell
	 * to be considered as un-traversible
	 */
	void set_min_collision_prob(double prob)
	{
		this->min_collide_occupancy = prob;
	}

	void print_query_info(octomap::point3d query, octomap::OcTreeNode* node)
	{
		using std::endl;
		using std::cout;
		if (node != NULL) {
			cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
		} else
			cout << "occupancy probability at " << query << ":\t is unknown" << endl;
	}

	void test(octomap::point3d origin, octomap::point3d end)
	{
		using std::endl;
		using std::cout;
		cout << "----------------------" << endl;
		cout << "performing computeRay:" << endl;
		std::vector<octomap::point3d> ray;
		if constexpr(std::is_pointer_v<tree_t>) {
			auto s = tree->computeRay(origin, end, ray);
		} else {
			auto s = tree.computeRay(origin, end, ray);
		}

		for(const auto &r : ray) {
			key_t key;
			if constexpr(std::is_pointer_v<tree_t>) {
				key = tree->search(r);
			} else {
				key = tree.search(r);
			}
			print_query_info(r, key);
		}
	}

	/* check if we have los from origin to endpoint
	 * given the minimum occupancy (or probability) threshold
	 */
	inline
	bool line_of_sight(const octomap::point3d &origin, const octomap::point3d &end, auto min_occupancy) const
	{
		std::vector<octomap::point3d> ray;
		if constexpr(std::is_pointer_v<tree_t>) {
			auto s = tree->computeRay(origin, end, ray);
		} else {
			auto s = tree.computeRay(origin, end, ray);
		}
		auto los = true;
		for(const auto &r : ray) {
			if constexpr(std::is_pointer_v<tree_t>) {
				if(tree->search(r)) {
					if(tree->search(r)->getOccupancy() > min_occupancy)
						los = false;
				}
			} else {
				if(tree.search(r)) {
					if(tree.search(r)->getOccupancy() > min_occupancy)
						los = false;
				}
			}
		}
		return los;
	}

	/* check if we have los from origin to endpoint
	 * given occupancy (or probability)
	 */
	inline
	bool line_of_sight(const octomap::point3d &origin, const octomap::point3d &end) const
	{
		return this->line_of_sight(origin, end, this->min_collide_occupancy);
	}

	/* check if a line from p0 to p1 is in collision
	 * conversion from type of p0 (and p1) to point3d
	 * is deduced automatically
	 */
	inline
	auto collide(const auto &p0, const auto &p1) const
	-> decltype(elements::x(p0), elements::y(p0), elements::z(p0),
	            elements::x(p1), elements::y(p1), elements::z(p1),
	            bool())
	{
		using elements::x;
		using elements::y;
		using elements::z;
		auto origin = octomap::point3d(x(p0), y(p0), z(p0));
		auto endpoint = octomap::point3d(x(p1), y(p1), z(p1));
		return !this->line_of_sight(origin, endpoint, min_collide_occupancy);
	}
};

template <typename grid_t>
class Publisher
{
private:
	grid_t &grid;
	std::string frame_id;
	size_t seq = 0;

private:
	// dummy function for deduction purpose only!
	constexpr auto grid_msgs_type()
	{
		using octomap_msgs_t = octomap_msgs::Octomap;
		if constexpr(!std::is_pointer_v<grid_t>) {
			// here, we have reference to OcTree object
			if constexpr(std::is_same_v<decltype(grid),octomap::OcTree>) {
				return octomap_msgs_t();
			}
			// here, we have pointer to AbstractOcTree object
			else if(std::is_same_v<decltype(grid),octomap::AbstractOcTree>) {
				return octomap_msgs_t();
			}
		} else {
			// here, we have pointer to OcTree object
			if constexpr(std::is_same_v<decltype(grid),octomap::OcTree*>) {
				return octomap_msgs_t();
			}
			// here, we have pointer to AbstractOcTree object
			else if(std::is_same_v<decltype(grid),octomap::AbstractOcTree*>) {
				return octomap_msgs_t();
			}
		}
		// TODO : add more for different type, e.g. nav_msgs/OccupancyGrid
	}

	using grid_msgs_t = decltype(std::declval<Publisher<grid_t>>().grid_msgs_type());

public:
	Publisher(ros::NodeHandle &node, grid_t &grid, std::string topic, size_t queue = 3)
		: grid(grid)
	{
		pub = node.advertise<grid_msgs_t>(topic, queue);
	}

	auto setFrameID(auto frame_id)
	{
		this->frame_id = frame_id;
	}

	auto getFrameID() const
	{
		return this->frame_id;
	}

	inline
	auto publish()
	{
		prepare_message(choice<1> {}, this->grid);
		msg.header.frame_id = frame_id;
		msg.header.stamp = ros::Time::now();
		msg.header.seq = seq;
		pub.publish(msg);
		seq++;
	}

	ros::Publisher pub;
	grid_msgs_t msg;

private:
	inline
	auto prepare_message(choice<0>, auto grid)
	-> decltype(octomap_msgs::fullMapToMsg(grid, msg), void())
	{
		octomap_msgs::fullMapToMsg(grid, msg);
	}

	inline
	auto prepare_message(choice<1>, auto grid)
	-> decltype(*grid, octomap_msgs::fullMapToMsg(*grid, msg), void())
	{
		octomap_msgs::fullMapToMsg(*grid, msg);
	}
};

}

#endif // GRIDMAP_HPP
