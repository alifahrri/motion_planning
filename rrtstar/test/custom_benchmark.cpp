#include <vector>
#include <chrono>
#include <sstream>
#include <iostream>
#include <tuple>
#include <algorithm>
#include <ros/ros.h>
#include "random.hpp"
#include "states.hpp"
#include "rrtvisual.hpp"
#include "environment.hpp"
#include "integrator2drrt.hpp"
#include "logger.hpp"

using namespace std;

auto grow_dynamic_env(auto xs, auto xg, RRTVisual &vis, size_t target, size_t test, bool direct_sample = false, double prob = 0.0) {
  using clock = std::chrono::high_resolution_clock;
  using duration = std::chrono::duration<double>;
  auto &rrt = Kinodynamic::Wrapper::get_rrtstar_int2d_timespace_obs();
  auto &tree = Kinodynamic::Wrapper::get_tree_int2d();
  auto &env = Kinodynamic::Wrapper::get_dynamic_soccer_env();
  auto &sampler = Kinodynamic::Wrapper::get_sampler_dynamic_env();

  sampler.set_direct_sample(direct_sample, prob);
  sampler.target = xg;

  //  env.setRandomObstacles();
  //  auto xg = Kinodynamic::Wrapper::get_goal_dynamic_env().randomGoal();
  //  auto xs = Kinodynamic::Wrapper::get_sampler()();

  auto n = target;
  using cost = decltype(tree(0).cost());
  vector<bool> solved;
  vector<cost> costs;
  vector<double> time;
  vector<size_t> nodes;
  for(size_t k = 0; k<test; k++) {
    auto start = clock::now();
    rrt.setStart(xs);
    if(direct_sample) sampler.set_next_sample(xg, target);
    auto ts = tree.tree.size();
    while(ts < n) {
      rrt.grow(&xg);
      ts = tree.tree.size();
    }
    auto end = clock::now();
    solved.push_back(rrt.goalIndex()>=0);
    costs.push_back(tree(rrt.goalIndex()).cost());
    duration elapsed_seconds = end-start;
    time.push_back(elapsed_seconds.count());

    auto goal = tree.get_trajectory(rrt.goalIndex());
    nodes.push_back((goal.size()));

    {
      // ROS_INFO("adding visual..");
      double tf = 10.0;
      double delta = 0.2;
      auto iter = tf/delta;
      auto r = env.collision_radius;
      for(size_t i=0; i<iter; i++)
        vis.add_circles(env.at(i*delta),r,delta,(i*delta),0.0,0.0,0.0,(iter-i)/iter,"_obstacles");
      vis.add_trajectories<3,0,1,4>(tree.trajectories,0.0,1.0,0.0,0.4,"_exploration");
      if(rrt.goalIndex() > 0) {
        vis.add_trajectories<3,0,1,4>(goal,1.0,1.0,1.0,1.0,"_goal",.05f,.075f);
      }
      vis.add_point<2,0,1>(xs, 1.0f, 0.0f, 0.0f, 1.0f, "_start");
      vis.add_point<2,0,1>(xg, 0.0f, 0.0f, 1.0f, 1.0f, "_goal");
      // ROS_INFO("publish visual..");
      // clear all before re-drawing
      vis.delete_all();
      vis.publish();
      vis.clear();
    }
  }
  return std::make_tuple(time, costs, solved, nodes);
}

struct rrt_stats {
  // statistics for timing data
  double lq_time, uq_time;
  double min_time, max_time;
  double med_time, mean_time;
  std::vector<double> raw_time;
  // statistics for timing data
  double lq_cost, uq_cost;
  double min_cost, max_cost;
  double med_cost, mean_cost;
  std::vector<double> raw_cost;
  // probability of solved cases
  double solved;
  // show number of nodes for a given solution
  double mean_nodes;
  double min_nodes, max_nodes;
  std::vector<double> raw_nodes;
  auto compute_stats(auto v) {
    std::sort(v.begin(), v.end());
    auto min = v.front();
    auto max = v.back();
    // find median
    auto ii = v.size()/2;
    auto med = v.at(ii);
    if(!(v.size()%2)) med = (med + v.at(ii-1))/2;
    // find median of quartile
    auto lq = v.size() / 4;
    auto uq = lq + ii;
    auto lqv = v.at(lq);
    auto uqv = v.at(uq);
    auto s = std::accumulate(v.begin(), v.end(), 0.0);
    auto mean = s / v.size();
    return std::make_tuple(mean, med, lqv, uqv, min, max);
  }
  void compute_time(auto v) {
    raw_time.insert(raw_time.begin(), v.begin(), v.end());
    auto t = compute_stats(v);
    std::tie(mean_time, med_time, lq_time, uq_time, min_time, max_time) = t;
  }
  void compute_cost(auto v) {
    raw_cost.insert(raw_cost.begin(), v.begin(), v.end());
    std::vector<std::decay_t<decltype(v.front())>> nv;
    for(auto c : v) if(c > 0) nv.push_back((c));
    auto t = compute_stats(nv);
    std::tie(mean_cost, med_cost, lq_cost, uq_cost, min_cost, max_cost) = t;
  }
  void compute_solved(auto v) {
    auto count = 0;
    for(auto s : v)
      if(s) count++;
    solved = double(count)/v.size();
  }
  void compute_nodes(auto v) {
    size_t c = 0;
    for(auto n : v) c += n;
    mean_nodes = double(c)/v.size();
    auto res = std::minmax_element(v.begin(), v.end());
    min_nodes = v.at(res.first-v.begin());
    max_nodes = v.at(res.second-v.begin());
  }
  auto time_str() {
    stringstream ss;
    ss << "'time' : ["
       << mean_time << ","
       << min_time << ","
       << max_time << ","
       << med_time << ","
       << lq_time << ","
       << uq_time
       << "],";
    return ss.str();
  }
  auto cost_str() {
    stringstream ss;
    ss << "'cost' : ["
       << mean_cost << ","
       << min_cost << ","
       << max_cost << ","
       << med_cost << ","
       << lq_cost << ","
       << uq_cost
       << "],";
    return ss.str();
  }
  auto solved_str() {
    stringstream ss;
    ss << "solved probability : [" << solved << "]";
    return ss.str();
  }
  auto nodes_str() {
    stringstream ss;
    ss << "nodes[mean, min, max] : [" << mean_nodes << "," << min_nodes << "," << max_nodes << "]";
    return ss.str();
  }
  auto data_str() {
    std::stringstream ss;
    ss << "'rtime' : [";
    for(const auto &t : raw_time)
      ss << t << ",";
    ss << "]," << std::endl;
    ss << "'rcost' : [";
    for(const auto &c : raw_cost)
      ss << c << ",";
    ss << "]," << std::endl;
    return ss.str();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc,argv,"benchmark_node");
  ros::NodeHandle node;
  RRTVisual vis(node);
  // prepare nice python dictionary
  Logger log;
  auto &rrt = Kinodynamic::Wrapper::get_rrtstar_int2d_timespace_obs();
  auto &tree = Kinodynamic::Wrapper::get_tree_int2d();
  auto &env = Kinodynamic::Wrapper::get_dynamic_soccer_env();

  env.setRandomObstacles();
  auto xg = Kinodynamic::Wrapper::get_goal_dynamic_env().randomGoal();
  auto xs = Kinodynamic::Wrapper::get_sampler()();

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
  ROS_WARN("init state : %f, %f, %f, %f", xs(0), xs(1), xs(2), xs(3));
  ROS_WARN("goal state : %f, %f, %f, %f", xg(0), xg(1), xg(2), xg(3));

  map<size_t,rrt_stats> stats;
  vector<size_t> target_size{10, 25, 50, 75, 100, 150, 200};
  vector<double> ds_prob{.1, .25, .50, .75};
  int test_size = 100;
  if(ros::param::has("test_size"))
    ros::param::get("test_size",test_size);

  auto n_test = target_size.size() + target_size.size() * ds_prob.size();
  {
    stringstream ss;
    ss << "starting benchmark with " << test_size << " run(s) for each case";
    ROS_WARN("%s", ss.str().c_str());
  }

  auto compute_stats = [](auto &stat, auto &ret) {
    stat.compute_time(get<0>(ret));
    stat.compute_cost(get<1>(ret));
    stat.compute_solved(get<2>(ret));
    stat.compute_nodes(get<3>(ret));
  };

  auto print_stats = [&log](auto &stats) {
    log << "{\n";
    for(auto s : stats) {
      stringstream ss;
      ss << s.first << ":" << endl
           << s.second.time_str() << endl
           << s.second.cost_str() << endl
           << s.second.solved_str() << endl
           << s.second.nodes_str() << endl;
      log << "\t\t" << s.first << ": {\n"
          << "\t\t\t" << s.second.time_str() << "\n"
          << "\t\t\t" << s.second.cost_str() << "\n"
          << "\t\t\t" << s.second.data_str() << "\n"
          << "\t\t},\n";
      ROS_WARN("\n%s",ss.str().c_str());
    }
    log << "}\n";
  };

  int i = 0;
  for(auto size : target_size) {
    stringstream ss;
    ss << i << "/" << n_test << "; " << double(i) * 100. / n_test << "%...";
    ROS_INFO("%s", ss.str().c_str());
    auto ret = grow_dynamic_env(xs, xg, vis, size, test_size);
    rrt_stats stat;
    compute_stats(stat, ret);
    stats[size] = stat;
    i++;
  }

  {
    stringstream ss;
    ss << "direct sampling disabled :";
    ROS_INFO("%s", ss.str().c_str());
    log << "False : ";
    print_stats(stats);
  }

  log << "\tTrue : {";
  for(auto p : ds_prob) {
    stats.clear();
    for(auto size : target_size) {
      stringstream ss;
      ss << i << "/" << n_test << "; " << double(i) * 100. / n_test << "%...";
      ROS_INFO("%s", ss.str().c_str());
      auto ret = grow_dynamic_env(xs, xg, vis, size, test_size, true, p);
      rrt_stats stat;
      compute_stats(stat, ret);
      stats[size] = stat;
      i++;
    }

    {
      stringstream ss;
      ss << "direct sampling enabled [" << p << "] :";
      ROS_INFO("%s", ss.str().c_str());
      log << p << " : ";
      print_stats(stats);
    }
  }
  log << "\t}";

  ROS_WARN("%s",log.str().c_str());
  return 0;
}
