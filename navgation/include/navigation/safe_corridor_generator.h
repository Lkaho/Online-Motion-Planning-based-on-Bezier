#ifndef _SAFE_CORRIDOR_GENERATOR_H
#define _SAFE_CORRIDOR_GENERATOR_H

#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "navigation/astar.h"
#include "navigation/data_struct.h"
#include "ros/ros.h"
using namespace std;
using namespace Eigen;

class SafeMoveCorridor {
 private:
  ros::Publisher corridor_pub_;
  ros::Publisher center_pt_publisher;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  double x_low_bound_;
  double x_up_bound_;
  double y_low_bound_;
  double y_up_bound_;
  int x_map_index_;
  int y_map_index_;

  int iteration_num_, expand_step_;
  double map_resolution_;
  double max_vel_;
  double max_acc_;

  vector<Cube> safe_corridor_;
  vector<double> time_seg;
  visualization_msgs::MarkerArray cube_vis;  // MarkerArray
                                             // 类似一个vector<Marker>
  Astar astar_;

 public:
  SafeMoveCorridor(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~SafeMoveCorridor() = default;
  pair<Cube, bool> expandCube(Cube& newCube, Cube& lastCube);
  static Cube generateCube(Vector2d pt);
  static bool isConstains(Cube& Cube1, Cube& Cube2);
  void genCorridor(
      const Vector2d& start_ps, const Vector2d& target_ps,
      const Vector2d& start_vel, const vector<Vector2d>& path);
  void visCorridor(vector<Cube>& CubeList);
  void visCenter(vector<Vector2d>& center_pt_list);
  static void simplify(vector<Cube>& corridor);
  static bool isOverlap(Cube& cube1, Cube& cube2);
  bool isOccupy(const Vector2i& index);
  vector<Vector2d> getCenterPt();
  void timeAllocation(
      const Vector2d& start_ps, const Vector2d& target_ps,
      const Vector2d& start_vel, const vector<Vector2d>& corridor_center);
  bool IsTargetValid(double x, double y) const;
  void setObs(double x, double y);
  void frontEndPathFinder(
      const Vector2d& start_ps, const Vector2d& target_ps,
      const Vector2d& start_vel);
  vector<Cube> getCorridor(){return safe_corridor_;}
  Vector2i getGridmapIndex(const Vector2d& pt);
  void visGridMap();
};

#endif