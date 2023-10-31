#include "navigation/safe_corridor_generator.h"

#include <utility>

#include "navigation/astar.h"
#include "navigation/data_struct.h"

SafeMoveCorridor::SafeMoveCorridor(
    const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh),
      nh_private_(pnh),
      x_low_bound_(0.0),
      x_up_bound_(0.0),
      y_low_bound_(0.0),
      y_up_bound_(0.0),
      x_map_index_(0),
      y_map_index_(0),
      iteration_num_(0),
      expand_step_(0),
      map_resolution_(0.05),
      max_acc_(0.0),
      max_vel_(0.0),
      astar_(nh, pnh) {
  {
    nh_private_.param("map/x_low_bound", x_low_bound_, -10.0);
    nh_private_.param("map/x_up_bound", x_up_bound_, 10.0);
    nh_private_.param("map/y_low_bound", y_low_bound_, -10.0);
    nh_private_.param("map/y_up_bound", y_up_bound_, 10.0);
    nh_private_.param("map/x_map_index", x_map_index_, 400);
    nh_private_.param("map/y_map_index", y_map_index_, 400);
    nh_private_.param("map/resolution", map_resolution_, 0.05);
    nh_private_.param("planning/max_acc", max_acc_, 1.0);
    nh_private_.param("planning/max_vel", max_vel_, 1.0);
    nh_private_.param("planning/iteration_num", iteration_num_, 50);
    nh_private_.param("planning/expand_step", expand_step_, 1);
  }
  corridor_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "vis_corridor", 100);
  center_pt_publisher =
      nh_private_.advertise<visualization_msgs::Marker>("vis_center", 10);
}

/*
    pari<Corridor,  bool>  record every Cube of waypoints
*/
pair<Cube, bool> SafeMoveCorridor::expandCube(Cube& newCube, Cube& lastCube) {
  Cube Cube_max = newCube;  // now newcube is a point
  MatrixXi vertex_idx(4, 2);
  MatrixXi vertex_idx_last(4, 2);
  MatrixXd vertex_coord(4, 2);
  Vector2d center_coord = newCube.center;
  Vector2i center_index = astar_.coor2grid(center_coord);
  bool collide_p1_p2 = false;
  bool collide_p2_p3 = false;
  bool collide_p3_p4 = false;
  bool collide_p1_p4 = false;
  if (isConstains(
          lastCube,
          Cube_max))  // 之前没加入iter=3的限制导致了生成的corrido太过稀疏，公共部分太小导致
    return make_pair(lastCube, false);

  if (astar_.isOccupied(center_index)) {
    ROS_ERROR("[Planning Node] path has node in obstacles !");
    return make_pair(Cube_max, false);
  }
  for (int i = 0; i < 4; i++) {
    vertex_idx.row(i) = center_index;
  }
  int idx_x_up, idx_x_lo, idx_y_up, idx_y_lo;
  int iter = 0;
  while (iter < iteration_num_) {
    // X , Y inflation
    idx_x_up = min(center_index(0) + iter + 1, x_map_index_ - 1);
    idx_x_lo = max(center_index(0) - (iter + 1), 0);
    idx_y_lo = max(center_index(1) - (iter + 1), 0);
    idx_y_up = min(center_index(1) + (iter + 1), y_map_index_ - 1);

    // check wheter collide

    // P1 - P2  Y+ direction expand direction
    if (!collide_p1_p2) {
      for (int i = vertex_idx(0, 1) + 1; i <= idx_y_up; ++i)  // y coordinate
      {
        for (int j = vertex_idx(0, 0); j <= vertex_idx(1, 0);
             ++j)  // x coordinate
        {
          if (astar_.isOccupied(j, i)) {
            collide_p1_p2 = true;
            break;
          }
        }
      }
      if (collide_p1_p2) {
        vertex_idx(0, 1) = idx_y_up - 1;
        vertex_idx(1, 1) = idx_y_up - 1;
      } else {
        vertex_idx(0, 1) = idx_y_up;
        vertex_idx(1, 1) = idx_y_up;
      }
    }

    // P4 -P3   Y- direction expands
    if (!collide_p3_p4) {
      for (int i = vertex_idx(3, 1) - 1; i >= idx_y_lo; --i) {
        for (int j = vertex_idx(3, 0); j <= vertex_idx(2, 0); ++j) {
          if (astar_.isOccupied(j, i)) {
            collide_p3_p4 = true;
            break;
          }
        }
      }
      if (collide_p3_p4) {
        vertex_idx(2, 1) = idx_y_lo + 1;
        vertex_idx(3, 1) = idx_y_lo + 1;
      } else {
        vertex_idx(2, 1) = idx_y_lo;
        vertex_idx(3, 1) = idx_y_lo;
      }
    }
    // P3 -P2 X+ direction
    if (!collide_p2_p3) {
      for (int i = vertex_idx(2, 0) + 1; i <= idx_x_up; ++i) {
        for (int j = vertex_idx(1, 1); j >= vertex_idx(2, 1); --j) {
          if (astar_.isOccupied(i, j)) {
            collide_p2_p3 = true;
            break;
          }
        }
      }
      if (collide_p2_p3) {
        vertex_idx(1, 0) = idx_x_up - 1;
        vertex_idx(2, 0) = idx_x_up - 1;
      } else {
        vertex_idx(1, 0) = idx_x_up;
        vertex_idx(2, 0) = idx_x_up;
      }
    }

    // P4 -P1 X- direction
    if (!collide_p1_p4) {
      for (int i = vertex_idx(0, 0) - 1; i >= idx_x_lo; --i) {
        for (int j = vertex_idx(0, 1); j >= vertex_idx(3, 1); --j) {
          if (astar_.isOccupied(i, j)) {
            collide_p1_p4 = true;
            break;
          }
        }
      }
      if (collide_p1_p4) {
        vertex_idx(0, 0) = idx_x_lo + 1;
        vertex_idx(3, 0) = idx_x_lo + 1;
      } else {
        vertex_idx(0, 0) = idx_x_lo;
        vertex_idx(3, 0) = idx_x_lo;
      }
    }
    // if the size of corrdidor doesn't change,  break from while loop
    if (vertex_idx_last == vertex_idx)
      break;

    vertex_idx_last = vertex_idx;
    // std::cout << "vertex_idx"  << vertex_idx << std::endl;
    for (int i = 0; i < 4; ++i) {
      int vertex_idx_x = vertex_idx(i, 0);
      int vertex_idx_y = vertex_idx(i, 1);

      Vector2i index(vertex_idx_x, vertex_idx_y);
      Vector2d coord = astar_.grid2coor(index);
      vertex_coord.row(i) = coord;
    }
    Cube_max.setVertex(vertex_coord, map_resolution_);
    if(isConstains(lastCube , Cube_max)&& iter == 3)
//     if(isConstains(lastCube , Cube_max))   //
////     之前没加入iter=3的限制导致了生成的corrido太过稀疏，公共部分太小导致
      return make_pair(lastCube , false);
    iter++;
  }
  return make_pair(Cube_max, true);
}
void SafeMoveCorridor::simplify(vector<Cube>& corridor) {
  Cube cube_self, cube;
  auto temp = corridor;
  int idx = 0;
  int length = (int)temp.size();
  corridor.clear();
  corridor.push_back(temp[idx]);
  for (int i = 1; i < length; ++i) {
    cube_self = temp[idx];
    cube = temp[i];
    if (!isOverlap(cube_self, cube)) {
      idx = i - 1;
      corridor.push_back(temp[idx]);
      // if(isOverlap(temp[idx] , temp[length - 1]))
      // break;
    }
  }
  corridor.push_back(temp[length - 1]);
}

bool SafeMoveCorridor::isConstains(Cube& Cube1, Cube& Cube2) {
  if (Cube1.vertex(0, 0) <= Cube2.vertex(0, 0) &&
      Cube1.vertex(0, 1) >= Cube2.vertex(0, 1) &&
      Cube1.vertex(1, 0) >= Cube2.vertex(1, 0) &&
      Cube1.vertex(1, 1) >= Cube2.vertex(1, 1) &&
      Cube1.vertex(2, 0) >= Cube2.vertex(2, 0) &&
      Cube1.vertex(2, 1) <= Cube2.vertex(2, 1) &&
      Cube1.vertex(3, 0) <= Cube2.vertex(3, 0) &&
      Cube1.vertex(3, 1) <= Cube2.vertex(3, 1))
    return true;
  else
    return false;
}

bool SafeMoveCorridor::isOverlap(Cube& cube_self, Cube& cube) {
  if (cube_self.vertex(0, 1) <= cube.vertex(2, 1) ||
      cube_self.vertex(2, 1) >= cube.vertex(0, 1) ||
      cube_self.vertex(0, 0) >= cube.vertex(2, 0) ||
      cube_self.vertex(2, 0) <= cube.vertex(0, 0))
    return false;
  else
    return true;
}
/*
    初始传进去的pt是A*找到的waypoints的物理坐标
*/
Cube SafeMoveCorridor::generateCube(Vector2d pt) {
  Cube Cube;
  double x_u, x_l, y_u, y_l;
  x_u = x_l = pt(0);
  y_u = y_l = pt(1);
  Cube.vertex.row(0) = Vector2d(x_l, y_u);
  Cube.vertex.row(1) = Vector2d(x_u, y_u);
  Cube.vertex.row(2) = Vector2d(x_u, y_l);
  Cube.vertex.row(3) = Vector2d(x_l, y_l);
  Cube.center = pt;
  return Cube;
}

void SafeMoveCorridor::genCorridor(
    const Vector2d& start_ps, const Vector2d& target_ps,
    const Vector2d& start_vel, const vector<Vector2d>& path) {
  if (!safe_corridor_.empty()) {
    safe_corridor_.clear();
  }

  Cube lastCube, newCube;

  Vector2d waypoint;
  // std::cout << path.size() <<std::endl;
  for (int i = 0; i < (int)path.size(); ++i) {
    waypoint = path[i];
    newCube = generateCube(waypoint);
    auto is_saveCube = expandCube(newCube, lastCube);
    if (!is_saveCube.second)
      continue;
    // std::cout << "waypoints is :" << waypoit << std::endl;
    newCube = is_saveCube.first;
    // std::cout << "Cube center is :" << newCube.center << std::endl;
    lastCube = newCube;
    safe_corridor_.push_back(newCube);
  }
  simplify(safe_corridor_);
  visCorridor(safe_corridor_);
  vector<Vector2d> corridor_center_list = getCenterPt();
  timeAllocation(start_ps, target_ps, start_vel, corridor_center_list);
  visCenter(corridor_center_list);
}

vector<Vector2d> SafeMoveCorridor::getCenterPt() {
  vector<Vector2d> center_pt_list;
  Vector2d center_pt;
  if(safe_corridor_.empty()){
    ROS_WARN("the safe corridor generates fail!..");
    return {};
  }
  for (int i = 0; i < (int)safe_corridor_.size() - 1; ++i) {
    double x_low =
        max(safe_corridor_[i].vertex(0, 0), safe_corridor_[i + 1].vertex(0, 0));
    double x_up =
        min(safe_corridor_[i].vertex(2, 0), safe_corridor_[i + 1].vertex(2, 0));
    double y_low =
        max(safe_corridor_[i].vertex(2, 1), safe_corridor_[i + 1].vertex(2, 1));
    double y_up =
        min(safe_corridor_[i].vertex(0, 1), safe_corridor_[i + 1].vertex(0, 1));
    center_pt(0) = (x_low + x_up) / 2.0;
    center_pt(1) = (y_low + y_up) / 2.0;
    center_pt_list.push_back(center_pt);

    // std::cout << "center_pt_x :" << center_pt(0) << std::endl;
    // std::cout << "center_pt_y :" << center_pt(1) << std::endl;
  }
  return center_pt_list;
}

void SafeMoveCorridor::visCorridor(vector<Cube>& SafeMovingCorridor) {
  for (auto& mk : cube_vis.markers)
    mk.action = visualization_msgs::Marker::DELETE;  // 删除上一次的cube

  cube_vis.markers.clear();

  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp = ros::Time::now();
  mk.ns = "corridor";
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;

  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.a = 0.6;
  mk.color.r = 0.0;
  mk.color.g = 1.0;
  mk.color.b = 1.0;

  int idx = 0;
  std::cout << SafeMovingCorridor.size() << std::endl;
  for (int i = 0; i < SafeMovingCorridor.size(); ++i) {
    mk.id = i;

    mk.pose.position.x = (SafeMovingCorridor[i].vertex(0, 0) +
                          SafeMovingCorridor[i].vertex(1, 0)) /
                         2.0;
    mk.pose.position.y = (SafeMovingCorridor[i].vertex(0, 1) +
                          SafeMovingCorridor[i].vertex(3, 1)) /
                         2.0;
    mk.pose.position.z = 0.0;  // 二维

    mk.scale.x =
        (SafeMovingCorridor[i].vertex(1, 0) -
         SafeMovingCorridor[i].vertex(0, 0));
    mk.scale.y =
        (SafeMovingCorridor[i].vertex(0, 1) -
         SafeMovingCorridor[i].vertex(3, 1));
    mk.scale.z = 0.05;

    idx++;
    cube_vis.markers.push_back(mk);
  }
  corridor_pub_.publish(cube_vis);
}
void SafeMoveCorridor::visCenter(vector<Vector2d>& center_pt_list) {
  if(center_pt_list.empty()){
    ROS_WARN("the center point list is empty!..");
    return;
  }
  visualization_msgs::Marker center_vis;
  center_vis.header.frame_id = "map";
  center_vis.header.stamp = ros::Time::now();

  center_vis.type = visualization_msgs::Marker::CUBE_LIST;
  center_vis.action = visualization_msgs::Marker::ADD;
  center_vis.id = 0;

  center_vis.pose.orientation.x = 0.0;
  center_vis.pose.orientation.y = 0.0;
  center_vis.pose.orientation.z = 0.0;
  center_vis.pose.orientation.w = 1.0;

  center_vis.color.a = 1.0;
  center_vis.color.r = 1.0;
  center_vis.color.g = 0.0;
  center_vis.color.b = 0.0;

  center_vis.scale.x = 0.05;
  center_vis.scale.y = 0.05;
  center_vis.scale.z = 0.05;

  geometry_msgs::Point pt;
  int length = (int)center_pt_list.size();
  for (int i = 0; i < length; ++i) {
    Vector2d coord = center_pt_list[i];
    // std::cout << "nodes[i]" <<  nodes[i] << std::endl;
    pt.x = coord(0);
    pt.y = coord(1);
    pt.z = 0;

    center_vis.points.push_back(pt);
  }
  center_pt_publisher.publish(center_vis);
}

void SafeMoveCorridor::timeAllocation(
    const Vector2d& start_ps, const Vector2d& target_ps,
    const Vector2d& start_vel, const vector<Vector2d>& centerPtList) {
  vector<Vector2d> points;
  int size = static_cast<int>(centerPtList.size());
  double v_max = max_vel_;
  double a_max = max_acc_;
  double v_start = start_vel.norm();
  double acct = v_max / a_max;
  double dcct = v_max / a_max;
  double act_dist = 0.0;
  Vector2i firstPindex = astar_.coor2grid(centerPtList[0]);
  Vector2i startPindex = astar_.coor2grid(start_ps);
  Vector2i endPindex = astar_.coor2grid(centerPtList[size - 1]);
  Vector2i lastPindex = astar_.coor2grid(target_ps);
  points.push_back(start_ps);

  for (int i = 0; i < (int)centerPtList.size(); i++)
    points.push_back(centerPtList[i]);

  points.push_back(target_ps);

  double dist;
  for (int k = 0; k < (int)points.size() - 1; k++) {
    double t;
    Vector2d p0 = points[k];
    Vector2d p1 = points[k + 1];
    Vector2d d = p1 - p0;
    dist = d.norm();
    if (k == 0 & v_start != 0) {
      acct = (v_max - v_start) / a_max;
      act_dist = (v_start + v_max) * acct * 0.5 + v_max * dcct * 0.5;
    } else {
      act_dist = (acct + dcct) * v_max * 0.5;
    }
    if (act_dist <= dist)
      t = (dist - act_dist) / v_max + acct + dcct;
    else
      t = acct + dcct;
    safe_corridor_[k].t = t;
  }
}

bool SafeMoveCorridor::IsTargetValid(double x, double y) const {
  if (x < x_low_bound_ || x > x_up_bound_ || y < y_low_bound_ ||
      y > y_up_bound_)
    return false;
  return true;
}

void SafeMoveCorridor::setObs(double x, double y) {
  astar_.setObs(x, y);
}

void SafeMoveCorridor::updateLocalMap(const Eigen::Vector2d& local_min,
                                      const Eigen::Vector2d& local_max) {
  astar_.resetLocalGridmap(local_min, local_max);
}

void SafeMoveCorridor::frontEndPathFinder(
    const Vector2d& start_ps, const Vector2d& target_ps,
    const Vector2d& start_vel) {
  astar_.AstarFinder(start_ps, target_ps);
  auto grid_path = astar_.getPath();
  astar_.visGridPath(grid_path);
  astar_.resetGridNodemap();
  ros::Time time_before_gen_corridor = ros::Time::now();
  genCorridor(start_ps, target_ps, start_vel, grid_path);
  ros::Time time_after_gen_corridor = ros::Time::now();
  ROS_WARN(
      "Time consume in corridor generation is %f  s",
      (time_after_gen_corridor - time_before_gen_corridor).toSec());
}

bool SafeMoveCorridor::isOccupy(const Vector2i& index) {
  return astar_.isOccupied(index);
}

Vector2i SafeMoveCorridor::getGridmapIndex(const Vector2d& coor) {
  return astar_.coor2grid(coor);
}

void SafeMoveCorridor::visGridMap() {
  astar_.visGridMap();
}
void SafeMoveCorridor::inflateObs() {
  astar_.inflateObs();
}