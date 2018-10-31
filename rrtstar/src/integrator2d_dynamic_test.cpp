#include <fenv.h>
#include <array>
#include <boost/filesystem.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include "integrator2drrt.hpp"

int main(int argc, char** argv)
{
  // enable nan exception
  // feenableexcept(FE_DIVBYZERO);

  ros::init(argc, argv, "rrtstar_integrator2d_test");
  ros::NodeHandle node;

  Models::init_integrator2d();
  RRTVisual vis(node, "_test");

  ros::Rate rate(30.0f);
  ros::Duration duration(0,1);

  std::string home;
  ros::get_environment_variable(home, "HOME");

  // auto &rrt = Kinodynamic::Wrapper::get_rrtstar_int2d();
  auto &rrt = Kinodynamic::Wrapper::get_rrtstar_int2d_timespace_obs();
  auto &tree = Kinodynamic::Wrapper::get_tree_int2d();
  auto &env = Kinodynamic::Wrapper::get_dynamic_soccer_env();
  auto &checker = Kinodynamic::Wrapper::get_checker_time_space();
  auto &sampler = Kinodynamic::Wrapper::get_sampler_dynamic_env();
  auto &goal = Kinodynamic::Wrapper::get_goal_dynamic_env();
  auto &connector = Kinodynamic::Wrapper::get_connector();

  env.setRandomObstacles();
  auto xg = goal.randomGoal();
  auto xs = sampler();
  // rrt.setIteration(0);

  auto vis_t0 = ros::Time::now();
  bool solved = false;

  // read parameter
  int target_tree_size = 5000;
  double neighbor_radius_scale = 1.0;
  if(node.hasParam("target_tree_size")) {
    node.getParam("target_tree_size",target_tree_size);
    ROS_WARN("setting target size : %d",target_tree_size);
  }
  if(node.hasParam("neighbor_radius_scale")) {
    node.getParam("neighbor_radius_scale",neighbor_radius_scale);
    Kinodynamic::Wrapper::get_radius().scale = neighbor_radius_scale;
  }
  if(ros::param::has("obstacle_radius")) {
    double cr_param;
    ros::param::get("collision_radius", cr_param);
    env.collision_radius = cr_param;
    ROS_WARN("setting obstacle radius : %f", cr_param);
  }
  bool direct_sampling_en = false;
  double direct_sampling_prob = 0.5;
  if(ros::param::has("direct_sampling")) {
    bool ds_param;
    ros::param::get("direct_sampling", ds_param);
    direct_sampling_en = ds_param;
  }
  if(ros::param::has("direct_sampling_prob")) {
    double ds_prob;
    ros::param::get("direct_sampling_prob", ds_prob);
    direct_sampling_prob = ds_prob;
  }
  if(direct_sampling_en) {
    sampler.set_direct_sample(true, direct_sampling_prob);
    sampler.target = xg;
    ROS_WARN("direct sampling enabled with prob : %f", sampler.direct_sampler->p[0]);
  }
  if(node.hasParam("obstacles"))
  {
    XmlRpc::XmlRpcValue obstacles;
    node.getParam("obstacles", obstacles);
    ROS_ASSERT(obstacles.getType() == XmlRpc::XmlRpcValue::TypeArray);
    std::vector<std::array<double,4>> obs;
    for (int32_t i = 0; i < obstacles.size(); ++i)
    {
      ROS_ASSERT(obstacles[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
      std::array<double,4> o;
      for(int32_t j = 0; j<obstacles[i].size(); ++j) {
        ROS_ASSERT(obstacles[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        o[j] = obstacles[i][j];
      }
      obs.push_back(o);
    }
    ROS_WARN("setting obstacles from param");
    env.setObstacles(obs);
  }
  if(node.hasParam("init")) {
    ROS_WARN("setting initial states");
    XmlRpc::XmlRpcValue init;
    node.getParam("init", init);
    ROS_ASSERT(init.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < init.size(); ++i)
    {
      ROS_ASSERT(init[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      if(i<4) xs(i) = init[i];
    }
  }
  if(node.hasParam("goal")) {
    ROS_WARN("setting goal");
    XmlRpc::XmlRpcValue goal;
    node.getParam("goal", goal);
    ROS_ASSERT(goal.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < goal.size(); ++i)
    {
      ROS_ASSERT(goal[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      if(i<4) xg(i) = goal[i];
    }
  }
  double expl_opacity = 0.25;
  if(node.hasParam("trajectory_opacity"))
    node.getParam("trajectory_opacity", expl_opacity);

  for(size_t i=0; i<env.obs.size(); i++) {
    std::stringstream ss;
    auto print_obs = [](auto o) {
      std::stringstream ss;
      for(size_t i=0; i<4; i++) ss << o[i] << (i==3 ? "" : ",");
      return ss.str();
    };
    ss << "obstacles " << i << " : " << print_obs(env.obs.at(i));
    ROS_WARN("%s", ss.str().c_str());
  }

  rrt.setStart(xs);
  auto ts = tree.tree.size();
  ros::Rate rt(1);
  while(ros::ok()) {
    ROS_INFO("growing tree..");
    auto t0 = ros::Time::now();
    if(ts < target_tree_size) solved = rrt.grow(&xg);
    else rt.sleep();
    auto tree_size = tree.tree.size();
    auto t1 = ros::Time::now();
    auto s = sampler.last_sample();
    ROS_INFO("sample : (%f,%f,%f,%f) %s", s(0), s(1), s(2), s(3),
             (tree_size > ts ? "OK" : "failed"));
    ROS_INFO("tree size : %d;", tree_size);
    ts = tree_size;

    auto vis_t1 = ros::Time::now();
    auto dt = vis_t1-vis_t0;
    ROS_INFO("dt : %f", dt.toSec());
    if(dt.toSec() > 0.5) {
      ROS_INFO("adding visual..");
      double tf = 10.0;
      double delta = 0.2;
      auto iter = tf/delta;
      auto r = env.collision_radius;
      for(size_t i=0; i<iter; i++) {
        // draw dynamic obstacles, with black(0.0,0.0,0.0) color,
        // and decreasing opacity over time
        vis.add_circles(env.at(i*delta),r,delta,(i*delta),0.0,0.0,0.0,(iter-i)/iter,"_obstacles");
      }
      // draw 3d trajectory : xy pos (index 0,1) in xy-plane and time (index 4) as z-plane
      // with green color (0.0,1.0,0.0) and 0.1 opacity
      vis.add_trajectories<3,0,1,4>(tree.trajectories,0.0,1.0,0.0,expl_opacity,"_exploration");
      if(rrt.goalIndex() > 0) {
        auto goal = tree.get_trajectory(rrt.goalIndex());
        vis.add_trajectories<3,0,1,4>(goal,1.0,1.0,1.0,1.0,"_goal");
      }
      // draw start and goal in 2D
      vis.add_point<2,0,1>(xs, 1.0f, 0.0f, 0.0f, 1.0f, "_start");
      vis.add_point<2,0,1>(xg, 0.0f, 0.0f, 1.0f, 1.0f, "_goal");
      vis.add_point<2,0,1>(sampler.last_sample(), 0.0f, 1.0f, 1.0f, 1.0f, "_last_sampled");
      auto last_edge = connector.last_connection();
      vis.add_trajectories<3,0,1,4>(std::vector<decltype(last_edge)>{last_edge}, 1.0f, 0.0f, 0.0f, 1.0f, "_last_connection");
      /*
      for(size_t i=0; i<5; i++) {
        auto s = tree(0);
        auto xr = sampler();
        auto c = connector(s,xr);
        {
          auto r = 1.0f; auto g = 1.0f; auto b = 0.0f;
          if(checker(c)) g = 0.0f;
          vis.add_trajectories<3,0,1,4>(std::vector<decltype(c)>{c}, r, g, b, 1.0f, "_test");
        }
      }
      */
      ROS_INFO("publish visual..");
      // clear all before re-drawing
      vis.delete_all();
      vis.publish();
      vis.clear();
      vis_t0 = vis_t1;
    }

    duration.sleep();
  }

  return 0;
}
