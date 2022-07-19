#ifndef _A_STAR_H
#define _A_STAR_H

#include "navigation/data_struct.h"
#include "ros/node_handle.h"
#include "sensor_msgs/PointCloud2.h"
#include <Eigen/Eigen>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;
class Astar {
public:
  Astar(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~Astar()= default;
  double getHeu(GridNodePtr node1, GridNodePtr node2);
  double getEuclidean(GridNodePtr node1, GridNodePtr node2);
  static double getManhHeu(GridNodePtr node1, GridNodePtr node2);
  void expandNode(GridNodePtr currentNode,
                  std::vector<GridNodePtr> &neighborSet,
                  std::vector<double> &fscoreSet);

  Eigen::Vector2d grid2coor(
      const Eigen::Vector2i
          &index); // Vector2d like a array, so use & as param is more effective
  Eigen::Vector2i coor2grid(const Eigen::Vector2d &positon);

  void AstarFinder(const Eigen::Vector2d& start_ps, const Eigen::Vector2d& end_ps);
  static void resetGrid(GridNodePtr node);
  void resetGridNodemap();
  void updateGridNodemap(const sensor_msgs::PointCloud2 &cloud);
  [[nodiscard]] bool isOccupied(const int &idx_x, const int &idx_y) const;
  [[nodiscard]] bool isOccupied(const Eigen::Vector2i &index) const;
  std::vector<Eigen::Vector2d> getPath();
  std::vector<Eigen::Vector2d> getVisitedNodes();
  void visGridPath(vector<Vector2d> nodes);
  void visVistedNodes(vector<Vector2d> nodes);
  void visGridMap();
  void setObs(double coord_x, double coord_y);
  double cost2Obs(GridNodePtr gridnode);
  void inflateObs();

private:
  GridNodePtr **GridNodeMap; // a GridMap to put the GridNodePtr
  Eigen::Vector2i goalIdx_;
  GridNodePtr terminatePtr_;

  int x_max_index_, y_max_index_;
  double time_brk_ ;
  double resolution_, inv_resolution_;
  double x_up_bound_, x_low_bound_, y_up_bound_, y_low_bound_;



  ros::Publisher grid_path_vis_pub_, visited_nodes_vis_pub_,
      GridNodeMap_vis_pub_, Obs_vis_pub_;
  // std::vector<GridNodePtr> pathGrid;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std::multimap<double, GridNodePtr> openSet_;
};

#endif