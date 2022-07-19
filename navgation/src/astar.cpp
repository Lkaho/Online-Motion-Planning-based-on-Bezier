#include "navigation/astar.h"
#include "ros/node_handle.h"
#include "sensor_msgs/PointCloud2.h"

using namespace std;
using namespace Eigen;
/*
    gmapping地图的原点设置为与机器人初始位置重合，然后原点对称对称
*/
Astar::Astar(const ros::NodeHandle& nh,const ros::NodeHandle& nh_private): nh_(nh), nh_private_(nh_private),
                                                                           x_max_index_(0), y_max_index_(0),
                                                                           x_up_bound_(0), x_low_bound_(0), y_up_bound_(0), y_low_bound_(0),
                                                                           resolution_(0), inv_resolution_(0),
                                                                           terminatePtr_(nullptr)
{
    nh_private_.param("map/resolution_", resolution_, 0.05);
    inv_resolution_ = 1.0 / resolution_;
    nh_private_.param("map/x_up_bound", x_up_bound_, 10.0);
    nh_private_.param("map/x_low_bound", x_low_bound_, -10.0);
    nh_private_.param("map/y_up_bound", y_up_bound_, 10.0);
    nh_private_.param("map/y_low_bound", y_low_bound_, -10.0);
    nh_private_.param("map/x_map_index", x_max_index_, 400);
    nh_private_.param("map/y_map_index", y_max_index_, 400);
    nh_private_.param("planning/time_brk", time_brk_, 1.0 + 1.0 / 1000.0);

    grid_path_vis_pub_ =
            nh_private_.advertise<visualization_msgs::Marker>("grid_path_vis", 1, true);
    visited_nodes_vis_pub_ =
            nh_private_.advertise<visualization_msgs::Marker>("visited_nodes_vis", 1, true);
    GridNodeMap_vis_pub_ =
            nh_private_.advertise<visualization_msgs::Marker>("vis_grid_map", 1, true);
    Obs_vis_pub_ = nh_private_.advertise<visualization_msgs::Marker>("visObs", 1, true);
    GridNodeMap = new GridNodePtr *[x_max_index_];
    for (int i = 0; i < x_max_index_; ++i) {
      GridNodeMap[i] = new GridNodePtr[y_max_index_];
      for (int j = 0; j < y_max_index_; ++j) {
        Vector2d coor;
        Vector2i index(i, j);
        coor = grid2coor(index);
        GridNodeMap[i][j] = new GridNode(index, coor);
      }
    }
    std::cout << "GridNodeMap init finished in astar cpp 43" << std::endl;
//    visGridMap();
}

void Astar::inflateObs() {
  GridNodePtr tempPtr;
  for (int i = 0; i < x_max_index_; ++i)
    for (int j = 0; j < y_max_index_; ++j) {
      tempPtr = GridNodeMap[i][j];
      if (tempPtr->isOccupy == 1) {
        for (int k = -4; k <= 4; ++k) {
          for (int l = -4; l <= 4; ++l) {
            if (k == 0 && l == 0) {
              continue;
            }
            Vector2i neighberidx;
            neighberidx(0) = (tempPtr->index)(0) + k;
            neighberidx(1) = (tempPtr->index)(1) + l;
            if (neighberidx(0) < 0 || neighberidx(0) >= x_max_index_ ||
                neighberidx(1) < 0 || neighberidx(1) >= y_max_index_) {
              continue;
            }
            GridNodeMap[neighberidx(0)][neighberidx(1)]->inflatOccpy = true;
          }
        }
      }
    }
}
void Astar::resetGrid(GridNodePtr node) {
  node->id = 0;
  node->fscore = inf;
  node->gscore = inf;
  node->ParentNode = nullptr;
}

void Astar::resetGridNodemap() {
  for (int i = 0; i < x_max_index_; ++i)
    for (int j = 0; j < y_max_index_; ++j)
      resetGrid(GridNodeMap[i][j]);
}

Eigen::Vector2d Astar::grid2coor(const Eigen::Vector2i &index) {
  Vector2d coor;
  coor(0) = ((double)index(0) + 0.5) * resolution_ + x_low_bound_;
  coor(1) = ((double)index(1) + 0.5) * resolution_ + y_low_bound_;
  return coor;
}
Eigen::Vector2i Astar::coor2grid(const Eigen::Vector2d &positon) {
  Vector2i index;
  index(0) = min(max(int((positon(0) - x_low_bound_) * inv_resolution_), 0),
                 x_max_index_ - 1);
  // std::cout << "index(0)" <<index(0) << std::endl;
  index(1) = min(max(int((positon(1) - y_low_bound_) * inv_resolution_), 0),
                 y_max_index_ - 1);
  // std::cout << "index(1)" <<index(1) << std::endl;

  return index;
}
bool Astar::isOccupied(const int &idx_x, const int &idx_y) const {
  return (idx_x >= 0 && idx_x < x_max_index_ && idx_y >= 0 &&
          idx_y < y_max_index_ &&
          ((GridNodeMap[idx_x][idx_y]->isOccupy == 1) ||
           (GridNodeMap[idx_x][idx_y]->inflatOccpy)));
}

bool Astar::isOccupied(const Eigen::Vector2i &index) const {
  return isOccupied(index(0), index(1));
}

void Astar::expandNode(GridNodePtr currentNode,
                       std::vector<GridNodePtr> &neighborSet,
                       std::vector<double> &fscoreSet) {
  neighborSet.clear();
  fscoreSet.clear();
  GridNodePtr temPtr;
  double edgeCost;
  for (int i = -1; i < 2; ++i)
    for (int j = -1; j < 2; ++j) {
      if (i == 0 && j == 0) {
        continue;
      }

      Vector2i neighberidx;
      // the difference of  (currentPtr->index)(0)  currentPtr->index(0)
      neighberidx(0) = (currentNode->index)(0) + i;
      neighberidx(1) = (currentNode->index)(1) + j;
      if (neighberidx(0) < 0 || neighberidx(0) >= x_max_index_ ||
          neighberidx(1) < 0 || neighberidx(1) >= y_max_index_) {
        continue;
      }

      if (isOccupied(neighberidx)) {
        continue;
      }

      temPtr = GridNodeMap[neighberidx(0)][neighberidx(1)];
      edgeCost = (temPtr->coor - currentNode->coor).norm();
      neighborSet.push_back(temPtr);
      fscoreSet.push_back(edgeCost);
    }
}

double Astar::getEuclidean(GridNodePtr node1, GridNodePtr node2) {
  return (node1->coor - node2->coor).norm();
}
double Astar::getManhHeu(GridNodePtr node1, GridNodePtr node2) {
  double dx = abs(node1->coor(0) - node2->coor(0));
  double dy = abs(node1->coor(1) - node2->coor(1));

  return dx + dy;
}

double Astar::getHeu(GridNodePtr node1, GridNodePtr node2) {
  return time_brk_ * getManhHeu(node1, node2);
}

void Astar::AstarFinder(const Eigen::Vector2d& start_ps, const Eigen::Vector2d& end_ps) {
  ros::Time time_1 = ros::Time::now();

  // index of start_point and end_point
  Vector2i start_idx = coor2grid(start_ps);
  // std::cout << "start_idx" << start_idx << std::endl;
  Vector2i end_idx = coor2grid(end_ps);
  //  std::cout << "end_idx" << end_idx << std::endl;

  goalIdx_ = end_idx;

  // position of start_point and end_point
  // start_ps = grid2coor(start_idx);
  std::cout << "start_ps" << start_ps << std::endl;
  // end_ps   = grid2coor(end_idx);
  std::cout << "end_ps" << end_ps << std::endl;

  // Initialize the pointers of struct GridNode which represent start node and
  // goal node
  GridNodePtr startPtr = new GridNode(start_idx, start_ps);
  GridNodePtr endPtr = new GridNode(end_idx, end_ps);

  // openSet_ is the open_list implemented through multimap in STL library
  openSet_.clear();
  // currentPtr represents the node with lowest f(n) in the open_list
  GridNodePtr currentPtr = NULL;
  GridNodePtr neighborPtr = NULL;

  // put start node in open set
  startPtr->gscore = 0;
  startPtr->fscore = getHeu(startPtr, endPtr);
  startPtr->id = 1;
  startPtr->coor = start_ps;
  openSet_.insert(make_pair(startPtr->fscore, startPtr));
  double temCost;
  vector<GridNodePtr> neighborPtrSets;
  vector<double> edgeCostSets;

  // this is the main loop
  while (!openSet_.empty()) {
    currentPtr = openSet_.begin()->second;
    currentPtr->id = -1;
    openSet_.erase(openSet_.begin());
    // if the current node is the goal
    if (currentPtr->index == goalIdx_) {
      ros::Time time_2 = ros::Time::now();
        terminatePtr_ = currentPtr;
      ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost is %f m",
               (time_2 - time_1).toSec() * 1000.0, currentPtr->gscore);
      return;
    }
    // get the succetion
    expandNode(currentPtr, neighborPtrSets, edgeCostSets);
    for (int i = 0; i < (int)neighborPtrSets.size(); i++) {
      neighborPtr = neighborPtrSets[i];
      temCost = currentPtr->gscore + edgeCostSets[i] + cost2Obs(neighborPtr);
      if (neighborPtr->id == 0) { // discover a new node, which is not in the
                                  // closed set and open set
        neighborPtr->ParentNode = currentPtr;
        neighborPtr->id = 1;

        neighborPtr->gscore = temCost;
        neighborPtr->fscore = neighborPtr->gscore + getHeu(neighborPtr, endPtr);
        neighborPtr->NodeItself =
            openSet_.insert(make_pair(neighborPtr->fscore, neighborPtr));
        continue;
      } else if (neighborPtr->id ==
                 1) { // this node is in open set and need to judge if it needs
                      // to update, the "0" should be deleted when you are
                      // coding
        if (neighborPtr->gscore > temCost) {
          openSet_.erase(neighborPtr->NodeItself);
          neighborPtr->gscore = temCost;
          neighborPtr->fscore = temCost + getHeu(neighborPtr, endPtr);
          neighborPtr->ParentNode = currentPtr;
          neighborPtr->NodeItself =
              openSet_.insert(make_pair(neighborPtr->fscore, neighborPtr));
        }
        continue;
      } else { // this node is in the closeSet
        continue;
      }
    }
  }
  // if search fails
  ros::Time time_2 = ros::Time::now();
  if ((time_2 - time_1).toSec() > 0.1)
    ROS_WARN("Time consume in Astar path finding is %f",
             (time_2 - time_1).toSec());
}

std::vector<Eigen::Vector2d> Astar::getPath() {
  vector<Vector2d> path;
  GridNodePtr temPtr;
  temPtr = terminatePtr_;
  std::vector<GridNodePtr> pathGrid;
  pathGrid.push_back(temPtr);

  while (temPtr->ParentNode != nullptr) {
    temPtr = temPtr->ParentNode;
    pathGrid.push_back(temPtr);
  }

  for (auto gridnode : pathGrid) {
    path.push_back(gridnode->coor);
  }

  reverse(path.begin(), path.end());
  // std::cout << path.size()<<std::endl;

  return path;
}

std::vector<Eigen::Vector2d> Astar::getVisitedNodes() {
  std::vector<Eigen::Vector2d> VisitedNodes;
  for (int i = 0; i < x_max_index_; ++i)
    for (int j = 0; j < y_max_index_; ++j)
      if (GridNodeMap[i][j]->id == 1 || GridNodeMap[i][j]->id == -1)
        VisitedNodes.push_back(GridNodeMap[i][j]->coor);
  return VisitedNodes;
}

void Astar::visGridPath(vector<Vector2d> nodes) {
  visualization_msgs::Marker node_vis;
  node_vis.header.frame_id = "map";
  node_vis.header.stamp = ros::Time::now();

  node_vis.type = visualization_msgs::Marker::CUBE_LIST;
  node_vis.action = visualization_msgs::Marker::ADD;
  node_vis.id = 0;

  node_vis.pose.orientation.x = 0.0;
  node_vis.pose.orientation.y = 0.0;
  node_vis.pose.orientation.z = 0.0;
  node_vis.pose.orientation.w = 1.0;

  node_vis.color.a = 0.5;
  node_vis.color.r = 1.0;
  node_vis.color.g = 0.0;
  node_vis.color.b = 0.0;

  node_vis.scale.x = resolution_;
  node_vis.scale.y = resolution_;
  node_vis.scale.z = resolution_;

  geometry_msgs::Point pt;
  int length = nodes.size();
  for (int i = 0; i < length; ++i) {
    Vector2d coord = nodes[i];
    std::cout << "nodes[i]" <<  nodes[i] << std::endl;
    pt.x = coord(0);
    pt.y = coord(1);
    pt.z = -0.01;

    node_vis.points.push_back(pt);
  }
  grid_path_vis_pub_.publish(node_vis);
}

void Astar::visVistedNodes(vector<Vector2d> nodes) {
  visualization_msgs::Marker node_vis;
  node_vis.header.frame_id = "map";
  node_vis.header.stamp = ros::Time::now();

  node_vis.type = visualization_msgs::Marker::SPHERE;
  node_vis.action = visualization_msgs::Marker::ADD;
  node_vis.id = 0;
  node_vis.ns = "expanded_nodes";

  node_vis.pose.orientation.x = 0.0;
  node_vis.pose.orientation.y = 0.0;
  node_vis.pose.orientation.z = 0.0;
  node_vis.pose.orientation.w = 1.0;

  node_vis.color.a = 1.0;
  node_vis.color.r = 0.0;
  node_vis.color.g = 1.0;
  node_vis.color.b = 0.0;

  node_vis.scale.x = resolution_;
  node_vis.scale.y = resolution_;
  node_vis.scale.z = 1.0;

  geometry_msgs::Point pt;
  for (int i = 0; i < nodes.size(); ++i) {
    Vector2d coord = nodes[i];
    pt.x = coord(0);
    pt.y = coord(1);
    pt.z = 0;

    node_vis.points.push_back(pt);
  }
  visited_nodes_vis_pub_.publish(node_vis);
}

void Astar::setObs(const double coord_x, const double coord_y) {
  if (coord_x < x_low_bound_ || coord_y < y_low_bound_ ||
      coord_x >= x_up_bound_ || coord_y >= y_up_bound_)
    return;

  int idx_x = static_cast<int>((coord_x - x_low_bound_) * inv_resolution_);
  int idx_y = static_cast<int>((coord_y - y_low_bound_) * inv_resolution_);

  GridNodeMap[idx_x][idx_y]->isOccupy = 1;
}

double Astar::cost2Obs(GridNodePtr gridnode) {

  for (int i = -1; i < 2; ++i)
    for (int j = -1; j < 2; ++j) {
      if (i == 0 && j == 0) {
        continue;
      }
      Vector2i neighberidx;
      neighberidx(0) = (gridnode->index)(0) + i;
      neighberidx(1) = (gridnode->index)(1) + j;
      if (neighberidx(0) < 0 || neighberidx(0) >= x_max_index_ ||
          neighberidx(1) < 0 || neighberidx(1) >= y_max_index_) {
        continue;
      }
      if (isOccupied(neighberidx))
        return 30.0;
    }
  return 0.0;
}

void Astar::visGridMap() {
  visualization_msgs::Marker node_vis;
  node_vis.header.frame_id = "map";
  node_vis.header.stamp = ros::Time::now();

  node_vis.type = visualization_msgs::Marker::CUBE_LIST;
  node_vis.action = visualization_msgs::Marker::ADD;
  node_vis.id = 0;
  node_vis.ns = "grid_map";

  node_vis.pose.orientation.x = 0.0;
  node_vis.pose.orientation.y = 0.0;
  node_vis.pose.orientation.z = 0.0;
  node_vis.pose.orientation.w = 1.0;

  node_vis.color.a = 0.8;
  node_vis.color.r = 0.0;
  node_vis.color.g = 0.0;
  node_vis.color.b = 1.0;

  node_vis.scale.x = resolution_;
  node_vis.scale.y = resolution_;
  node_vis.scale.z = 0.1;

  geometry_msgs::Point pt;
  for (int i = 0; i < x_max_index_; ++i) {
    for (int j = 0; j < y_max_index_; ++j) {
      Vector2i index(i, j);
      if (isOccupied(i, j)) {
        pt.x = GridNodeMap[i][j]->coor(0);
        pt.y = GridNodeMap[i][j]->coor(1);
        pt.z = 0.0;
        node_vis.points.push_back(pt);
      }
    }
  }
  GridNodeMap_vis_pub_.publish(node_vis);
}
