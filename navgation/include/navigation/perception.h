#ifndef _NAVIGATION_PERCEPTION_H_
#define _NAVIGATION_PERCEPTION_H_

#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <mutex>
#include <vector>

// ros
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

// tf
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

// pcl_ros
#include <pcl_ros/point_cloud.h>

// pcl
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LidarUndistortion {

  // 使用PCL中点的数据结构 pcl::PointXYZ
  typedef pcl::PointXYZ PointT;
  // 使用PCL中点云的数据结构 pcl::PointCloud<pcl::PointXYZ>
  typedef pcl::PointCloud<PointT> PointCloudT;

private:
  ros::NodeHandle node_handle_;           // ros中的句柄
  ros::NodeHandle private_node_;          // ros中的私有句柄
  ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber
  ros::Subscriber imu_subscriber_;        // 声明一个Subscriber
  ros::Subscriber odom_subscriber_;       // 声明一个Subscriber
  ros::Publisher corrected_pointcloud_publisher_;

  bool use_imu_, use_odom_;

  std::deque<sensor_msgs::LaserScan> laser_queue_; // 保存雷达数据
  std::deque<sensor_msgs::Imu> imu_queue_;
  std::deque<nav_msgs::Odometry> odom_queue_;

  bool first_scan_;
  std::vector<double> a_cos_; // 保存下来雷达各个角度的cos值
  std::vector<double> a_sin_; // 保存下来雷达各个角度的sin值

  sensor_msgs::LaserScan current_laserscan_;
  PointCloudT::Ptr corrected_pointcloud_;

  std_msgs::Header current_laserscan_header_;
  double current_scan_time_increment_;
  double current_scan_time_start_;
  double current_scan_time_end_;
  double scan_count_;

  std::mutex imu_lock_;
  std::mutex odom_lock_;

  int current_imu_index_;
  std::vector<double> imu_time_;
  std::vector<double> imu_rot_x_;
  std::vector<double> imu_rot_y_;
  std::vector<double> imu_rot_z_;

  nav_msgs::Odometry start_odom_msg_, end_odom_msg_;
  double start_odom_time_, end_odom_time_;
  float odom_incre_x_, odom_incre_y_, odom_incre_z_;

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point end_time_;
  std::chrono::duration<double> time_used_;

  bool CacheLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScanMsg);
  void CreateAngleCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
  bool PruneImuDeque();
  bool PruneOdomDeque();
  void CorrectLaserScan();
  void ComputeRotation(double pointTime, float *rotXCur, float *rotYCur,
                       float *rotZCur);
  void ComputePosition(double pointTime, float *posXCur, float *posYCur,
                       float *posZCur);
  void PublishCorrectedPointCloud();
  void ResetParameters();

public:
  LidarUndistortion();
  ~LidarUndistortion();

  void ImuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odometryMsg);
  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &laserScanMsg);
};

#endif