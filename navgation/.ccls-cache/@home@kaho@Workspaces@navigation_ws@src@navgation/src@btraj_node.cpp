#include "navigation/backward.hpp"
#include "navigation/bezier.h"
#include "navigation/trajectory_generator.h"
#include "ros/ros.h"
using namespace std;

namespace backward {
backward::SignalHandling sh;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "btraj_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  MatrixXd MQM;
  VectorXd CList;
  VectorXd CvList;
  int oder_min, order_max, traj_order;
  double min_order;
  nh_private.param("planning/order_min", oder_min, 3);
  nh_private.param("planning/order_max", order_max, 12);
  nh_private.param("planning/min_order", min_order, 2.5);
  nh_private.param("planning/traj_order", traj_order, 5);
  Bernstein bernstein;
  if (bernstein.setParam(3, 12, min_order) == -1)
    ROS_ERROR(
        " The trajectory order is set beyond the library's scope, please "
        "re-set ");

  MQM = bernstein.getMQM()[traj_order];
  CList = bernstein.getC()[traj_order];
  CvList = bernstein.getC_v()[traj_order];

  TrajectoryGenerator traj_generator(nh,nh_private,MQM,CList, CvList);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
