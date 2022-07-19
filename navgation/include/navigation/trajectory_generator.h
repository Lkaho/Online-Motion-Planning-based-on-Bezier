#ifndef _TRAJECTORY_GENERATOR_H
#define _TRAJECTORY_GENERATOR_H
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <osqp/osqp.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "btraj_msgs/trajectory.h"
#include "geometry_msgs/PoseStamped.h"
#include "navigation/bezier.h"
#include "navigation/data_struct.h"
#include "navigation/safe_corridor_generator.h"
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>


using namespace std;
using namespace Eigen;

class TrajectoryGenerator {
 private:
  ros::Publisher traj_vis_pub;
  ros::Publisher traj_pub_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber target_sub_;
  ros::Subscriber odom_sub_;
  ros::Timer exec_timer_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> cloudmap_sub_;
  tf::MessageFilter<sensor_msgs::PointCloud2> *tf_cloudmap_filter_;
  tf::TransformListener tf_listener_;

  MatrixXd MQM_;
  MatrixXd polyCoeff_;
  VectorXd Clist_;  // control point list;
  VectorXd CvList_;
  Vector2d odom_vel_;
  Vector2d odom_pos_;
  Vector2d target_pos_;
  VectorXd seg_time_;

  double odom_yaw_;
  double min_order_;
  double margin_;
  double max_vel_;
  double max_acc_;
  double arrive_thresh_;
  double no_replan_thresh_;
  double final_yaw_;
  int traj_order_;
  int gen_traj_num_;
  int traj_num_;

  bool isLimitVel_;
  bool isLimitAcc_;
  bool has_target_;
  bool has_odom_;
  bool has_cloudmap_;
  bool has_traj_;

  vector<Vector2d> ref_traj_;

  std::string target_frame_;

  SafeMoveCorridor safe_corridor_;

 public:
  enum STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, EXEC_TRAJ, REPLAN_TRAJ };
  TrajectoryGenerator(
      const ros::NodeHandle& nh, const ros::NodeHandle& pnh,
      const MatrixXd& MQM, const VectorXd& Clist, const VectorXd& CvList);
  ~TrajectoryGenerator()= default;;
  void targetCallback(const geometry_msgs::PoseStampedConstPtr& target);
  void odomCallback(const nav_msgs::OdometryConstPtr& odom);
  void laserToMapCallback(const sensor_msgs::PointCloud2ConstPtr& cloudmap);
  void execTimerCallback(const ros::TimerEvent& event);
  void traj_vis(VectorXd time);
  void pub_traj2ctrl(
      MatrixXd polyCoeff, VectorXd time, double _final_yaw);
  /* Use Bezier curve for the trajectory */
  int BezierPloyCoeffGeneration(
      const vector<Cube>& corridor, const MatrixXd& pos, const MatrixXd& vel,
      const MatrixXd& acc);
  Vector2d getPosFromBezier(
      const MatrixXd& polyCoeff, double t_now, int seg_now);
  Vector2d getVelFromBezier(
      const MatrixXd& polyCoeff, double t_now, int seg_now);
  void changeState(STATE new_state, const string& pos_call);
  bool trajPlanning();
  bool trajCheck();
  void printState();

 protected:
  STATE robot_state_;
  virtual OSQPSettings* SolverDefaultSettings();

  static void FreeData(OSQPData* data);

  template <typename T>
  T* CopyData(const std::vector<T>& vec) {
    T* data = new T[vec.size()];
    memcpy(data, vec.data(), sizeof(T) * vec.size());
    return data;
  }
};

#endif