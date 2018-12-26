#include "ros/ros.h"
#include "integrator2d.hpp"
#include "models/TrajectoryService.h"

bool compute(models::TrajectoryService::Request &req,
             models::TrajectoryService::Response &res)
{
  if(req.model == "integrator2d") {
    Models::Integrator2D int2d;
    if(req.params.size())
      int2d.set_weight(req.params[0]);
    auto &solver = int2d.solver;
    Models::Integrator2D::State x0;
    Models::Integrator2D::State x1;
    for(size_t i=0; i<4; i++)
    {
      x0(i) = req.xi.state.at(i);
      x1(i) = req.xf.state.at(i);
    }
    ROS_INFO("computing double integrator trajectory");
    auto trajectory = solver.solve(x0, x1);
    ROS_INFO("trajectory computation done");
    ROS_INFO("computing double integrator cost");
    auto cost = solver.cost(x0, x1);
    ROS_INFO("cost computation done");
    res.model = "integrator2d";
    res.time = std::get<0>(cost);
    res.cost = std::get<1>(cost);
    for(const auto &t : trajectory) {
      ros::Time time;
      models::StampedState s;
      models::StampedState a;
      s.n = 4; a.n = 2;
      a.t.data = s.t.data = time.fromSec(std::get<0>(t));
      for(size_t i=0; i<4; i++){
        s.state.push_back(std::get<1>(t)(i));
        if(i<2) a.state.push_back(std::get<2>(t)(i));
      }
      res.trajectory.push_back(s);
      res.inputs.push_back(a);
    }
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "integrator2d_trajectory");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("compute_trajectory", compute);
  ROS_INFO("Ready to compute trajectory.");
  ros::spin();

  return 0;
}
