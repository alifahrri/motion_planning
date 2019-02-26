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
#include "utility.hpp"

namespace gridmap
{

enum class los_method {
	max_occupancy,
	avg_occupancy
};

template <
  // the type of the tree
  typename tree_t
  // calculation method for checking collision
  , los_method check_method = los_method::max_occupancy
  // used if check == avg_occupancy
  , size_t avg_window = 3
  >
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
	using key_t = decltype(std::declval<OctomapWrapper<tree_t,check_method,avg_window>>().search_expr());

	// helper function for getting key from point3d
	inline
	bool get_key(const octomap::point3d &pt, key_t &key) const
	{
		key_t k;
		bool ret = false;
		if constexpr(std::is_pointer_v<tree_t>) {
			k = tree->search(pt);
		} else {
			k = tree.search(pt);
		}
		if(k) {
			key = k;
			ret = true;
		}
		return ret;
	}

	// helper fn for ray computation
	inline
	bool get_ray(const octomap::point3d &origin, const octomap::point3d &end, std::vector<octomap::point3d> &ray) const
	{
		bool s{0};
		if constexpr(std::is_pointer_v<tree_t>) {
			s = tree->computeRay(origin, end, ray);
		} else {
			s = tree.computeRay(origin, end, ray);
		}
		return s;
	}

	// helper fn for type conversion
	inline
	auto to_point3d(const auto &p) const
	-> decltype(elements::x(p), elements::y(p), elements::z(p), octomap::point3d {})
	{
		using elements::x;
		using elements::y;
		using elements::z;
		return octomap::point3d(x(p), y(p), z(p));
	}

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

	/* check if we have los from origin to endpoint
	 * given the minimum occupancy (or probability) threshold
	 */
	inline
	bool line_of_sight(const octomap::point3d &origin, const octomap::point3d &end, auto threshold) const
	{
		// std::cout << __PRETTY_FUNCTION__ << std::endl;
		std::vector<octomap::point3d> ray;
		get_ray(origin, end, ray);
		auto los = true;
		double total_occupancy = 0.0;
		if constexpr(check_method == los_method::max_occupancy) {
			for(const auto &r : ray) {
//				std::cout << "("
//				          << elements::x(r) << ", "
//				          << elements::y(r) << ", "
//				          << elements::z(r)
//				          << ")";
				key_t key = nullptr;
				get_key(r, key);
				if(key) {
					los = (key->getOccupancy() > threshold ? false : los);
//					std::cout << " : " << key->getOccupancy();
				}
//				std::cout << std::endl;
				if(!los) break;
			}
		} else if(check_method == los_method::avg_occupancy) {
			utility::RingBuffer<double,avg_window> buffer;
			for(const auto &r : ray) {
//				std::cout << "("
//				          << elements::x(r) << ", "
//				          << elements::y(r) << ", "
//				          << elements::z(r)
//				          << ")";
				key_t key = nullptr;
				get_key(r, key);
				if(key) {
					buffer.put(key->getOccupancy());
					if(buffer.is_full()) {
						std::vector<double> occupancies(buffer.size());
						buffer.get_elements(occupancies);
						double sum = 0.0;
						for(const auto &o : occupancies)
							sum += o;
						auto avg_occ = sum/avg_window;
//						std::cout << " sum : " << sum << ", avg_occ : " << avg_occ;
						los = (avg_occ > threshold ? false : los);
					}
//					std::cout << ", occ : " << key->getOccupancy();
				}
//				std::cout << std::endl;
				if(!los) break;
			}
		}
		return los;
	}

	/* check if we have los from origin to endpoint
	 * conversion from type of p0 (and p1) to point3d
	 * is deduced automatically
	 */
	inline
	auto line_of_sight(const auto &origin, const auto &end) const
	-> decltype(to_point3d(origin), to_point3d(end), bool {})
	{
		return this->line_of_sight(to_point3d(origin), to_point3d(end), this->min_collide_occupancy);
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
		auto origin = to_point3d(p0);
		auto endpoint = to_point3d(p1);
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
