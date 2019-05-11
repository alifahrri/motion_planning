#include "geometric2d.hpp"

int main(int argc, char** argv)
{
  ros::init(argc,argv,"rrtstar_geometric_3d");
  ros::NodeHandle node;
  auto p0 = Geometric::Point2D();
  p0.p[0] = 0.0;
  p0.p[1] = 0.0;
  auto c = Geometric::CostType(0.0);
  auto &tree = Geometric::tree2d;
  auto &geom_rrt = Geometric::rrtstar_2d;
  auto start = Geometric::State2D(std::make_shared<Geometric::Point2D>(p0),std::make_shared<Geometric::CostType>(c));
  Geometric::checker.setRandomObstacles();
  RRTVisual vis(node);
  geom_rrt.setStart(start);
  ros::Rate rate(10.0f);
  ros::Duration duration(0,1);
  while(ros::ok()) {
    geom_rrt.grow();
    auto tree_size = tree.tree.size();
    ROS_INFO("tree size : %d;", tree_size);
    if((tree_size % 100) == 0) {
      vis.set_nodes<2>(tree.tree.cloud.states,tree.parent,tree.tree.size());
      auto r = Geometric::checker.env.collision_radius;
      for(const auto &o : Geometric::checker.env.obs)
        vis.add_obstacles(std::get<0>(o),std::get<1>(o),r);
      vis.delete_all();
      vis.publish();
      vis.clear();
    }
    // rate.sleep();
    duration.sleep();
  }
  return 0;
}
