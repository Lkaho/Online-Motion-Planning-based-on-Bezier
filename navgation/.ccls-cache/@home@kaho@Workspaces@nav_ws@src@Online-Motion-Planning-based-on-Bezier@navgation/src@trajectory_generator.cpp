#include "navigation/trajectory_generator.h"

using namespace std;
using namespace Eigen;

TrajectoryGenerator::TrajectoryGenerator(
    const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const MatrixXd& MQM,
    const VectorXd& CList, const VectorXd& CvList)
    : nh_(nh),
      nh_private_(pnh),
      MQM_(MQM),
      Clist_(CList),
      CvList_(CvList),
      gen_traj_num_(0),
      has_target_(false),
      has_odom_(false),
      has_cloudmap_(false),
      has_traj_(false),
      target_frame_("map"),
      safe_corridor_(nh, pnh),
      odom_vel_(0.0, 0.0),
      odom_pos_(0.0, 0.0),
      robot_state_(STATE::INIT) {
  nh_private_.param("planning/traj_order", traj_order_, 5);
  nh_private_.param("planning/min_order", min_order_, 2.5);
  nh_private_.param("planning/margin", margin_, 0.0);
  nh_private_.param("planning/max_acc", max_acc_, 1.0);
  nh_private_.param("planning/max_vel", max_vel_, 1.0);
  nh_private_.param("planning/is_limit_acc", isLimitAcc_, true);
  nh_private_.param("planning/is_limit_vel", isLimitVel_, true);
  nh_private_.param("planning/target_frame", target_frame_, string("map"));
  nh_private_.param("planning/arrive_thresh", arrive_thresh_, 0.1);
  nh_private_.param("planning/no_replan_thresh", no_replan_thresh_, 0.1);
  nh_private_.param("planning/update_range", update_range_, 4.0);

  // Subscriber
  target_sub_ = nh_.subscribe(
      "move_base_simple/goal", 1, &TrajectoryGenerator::targetCallback, this);
  odom_sub_ =
      nh_.subscribe("odom", 1, &TrajectoryGenerator::odomCallback, this);
  exec_timer_ = nh_.createTimer(
      ros::Duration(0.01), &TrajectoryGenerator::execTimerCallback, this);
  cloudmap_sub_.subscribe(nh_, "correct_pointcloud", 100);
  tf_cloudmap_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(
      cloudmap_sub_, tf_listener_, target_frame_, 10);
  tf_cloudmap_filter_->registerCallback(
      boost::bind(&TrajectoryGenerator::laserToMapCallback, this, _1));
  // Publisher
  traj_pub_ = nh_private_.advertise<btraj_msgs::trajectory>("trajToctrl", 1);
  traj_vis_pub =
      nh_private_.advertise<visualization_msgs::Marker>("traj_vis", 1);
}
void TrajectoryGenerator::execTimerCallback(const ros::TimerEvent& e) {
  static int num = 0;
  num++;
  if (num == 100) {
    printState();
    if (!has_odom_)
      cout << "no odom." << endl;
    if (!has_target_)
      cout << "wait for goal." << endl;
    if (!has_cloudmap_)
      cout << "no map." << endl;
    num = 0;
  }

  switch (robot_state_) {
    case INIT: {
      if (!has_odom_)
        return;
      changeState(WAIT_TARGET, "STATE");
      break;
    }

    case WAIT_TARGET: {
      if (!has_cloudmap_)
        return;
      if (!has_target_)
        return;
      else
        changeState(GEN_NEW_TRAJ, "STATE");
      break;
    }

    case GEN_NEW_TRAJ: {
      bool success = trajPlanning();
      if (success)
        changeState(EXEC_TRAJ, "STATE");
      else {
        // ROS_WARN("The taget is not arrivable, Pleas reset target!");
        changeState(GEN_NEW_TRAJ, "STATE");
      }
      break;
    }

    case EXEC_TRAJ: {
      if ((target_pos_ - odom_pos_).norm() <= arrive_thresh_) {
        has_target_ = false;
        changeState(WAIT_TARGET, "STATE");
        return;
      } else if ((target_pos_ - odom_pos_).norm() < no_replan_thresh_) {
        return;
      }
      break;
    }
    case REPLAN_TRAJ: {
      bool success = trajPlanning();
      if (success)
        changeState(EXEC_TRAJ, "STATE");
      else
        changeState(GEN_NEW_TRAJ, "STATE");
      break;
    }
  }
}
void TrajectoryGenerator::targetCallback(
    const geometry_msgs::PoseStampedConstPtr& target) {
  double x = target->pose.position.x;
  double y = target->pose.position.y;
  final_yaw_ = tf::getYaw(target->pose.orientation);
  if (!safe_corridor_.IsTargetValid(x, y)) {
    ROS_WARN("The target is not in the valid region, please reset.");
    has_target_ = false;
    return;
  }
  target_pos_ = Vector2d(x, y);
  has_target_ = true;
  ROS_INFO(
      "\033[1;32m----> Planner has received target point at, %f,%f \033[0m", x,
      y);
  if (robot_state_ == WAIT_TARGET) {
    changeState(GEN_NEW_TRAJ, "STATE");
  } else if (robot_state_ == EXEC_TRAJ) {
    changeState(REPLAN_TRAJ, "STATE");
  }
}

void TrajectoryGenerator::odomCallback(const nav_msgs::OdometryConstPtr& odom) {
  double current_x = odom->pose.pose.position.x;
  double current_y = odom->pose.pose.position.y;
  if (isnan(current_x) || isnan(current_y)) {
    has_odom_ = false;
    ROS_WARN("Something wrong with odom, please check it!");
    return;
  }
  has_odom_ = true;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(odom->pose.pose.orientation, quat);
  quat.normalize();
  double linear_vel, roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  odom_pos_(0) = current_x;
  odom_pos_(1) = current_y;
  odom_yaw_ = yaw;
  odom_vel_(0) = cos(odom_yaw_) * linear_vel;  // x velocity
  odom_vel_(1) = sin(odom_yaw_) * linear_vel;
  linear_vel = (double)odom->twist.twist.linear.x;
}

void TrajectoryGenerator::laserToMapCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloudmap) {
  pcl::PointCloud<pcl::PointXYZ> local_pointcloud;
  pcl::PointCloud<pcl::PointXYZ> local_pointcloud_filtered;
  pcl::PointCloud<pcl::PointXYZ> global_pointcloud;
  pcl::fromROSMsg(*cloudmap, local_pointcloud);
  if (local_pointcloud.points.empty()) {
    has_cloudmap_ = false;
    ROS_WARN("Planner node does not receive the map!");
    return;
  }
  has_cloudmap_ = true;
  for(int i = 0; i < (int)local_pointcloud.points.size(); i++) {
    if(local_pointcloud.points[i].x <= 0.05 && local_pointcloud.points[i].y <= 0.05)
      continue;
    local_pointcloud_filtered.points.push_back(local_pointcloud.points[i]);
  }
  // transform to global frame
  tf::StampedTransform laserToMap_transform;
  try {
    tf_listener_.lookupTransform(
        target_frame_, cloudmap->header.frame_id, cloudmap->header.stamp,
        laserToMap_transform);
  } catch (tf::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  Eigen::Matrix4f  sensorToWorld;
  pcl_ros::transformAsMatrix(laserToMap_transform, sensorToWorld);
  pcl::transformPointCloud(local_pointcloud_filtered, global_pointcloud, sensorToWorld);
  // update the map
  Eigen::Vector2d local_min(odom_pos_(0) - update_range_ / 2.0,
                            odom_pos_(1) - update_range_ / 2.0);
  Eigen::Vector2d local_max(odom_pos_(0) + update_range_ / 2.0,
                            odom_pos_(1) + update_range_ / 2.0);
  safe_corridor_.updateLocalMap(local_min, local_max);
  for (int i = 0; i < (int)global_pointcloud.points.size(); ++i) {
    double x = global_pointcloud.points[i].x;
    double y = global_pointcloud.points[i].y;

    safe_corridor_.setObs(x, y);
  }
  safe_corridor_.inflateObs();
  safe_corridor_.visGridMap();

  if (!trajCheck()) {
    ROS_WARN("Current trajectory meets obstacles, needs replanning!");
    changeState(REPLAN_TRAJ, "STATE");
  }
}

void TrajectoryGenerator::changeState(STATE new_state, const string& pos_call) {
  string state_str[5] = {
      "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ", "REPLAN_TRAJ"};
  int pre_s = int(robot_state_);
  robot_state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " +
              state_str[int(new_state)]
       << endl;
}

void TrajectoryGenerator::printState() {
  string state_str[5] = {
      "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ", "REPLAN_TRAJ"};
  cout << "[Clock]: state: " + state_str[int(robot_state_)] << endl;
}

int TrajectoryGenerator::BezierPloyCoeffGeneration(
    const vector<Cube>& corridor, const MatrixXd& pos, const MatrixXd& vel,
    const MatrixXd& acc) {
#define ENFORCE_VEL isLimitVel_
#define ENFORCE_ACC isLimitAcc_

  double first_cube_time = corridor.front().t;
  double last_cube_time = corridor.back().t;

  int segment_num = corridor.size();
  int d1_ctrl_point_num =
      traj_order_ + 1;  // one dimentional control points number
  int ctrl_point_num = 2 * d1_ctrl_point_num;  // number of coeff each seg

  //  dimention of beq
  int equ_con_s_num = 2 * 3;  // p, v, a in x, y axis at the start point
  int equ_con_e_num = 2 * 3;  // p, v, a in x, y axis at the end point
  int equ_con_continuity_num =
      2 * 3 * (segment_num - 1);  // continous constrain p , v ,a
  int equ_con_num = equ_con_s_num + equ_con_e_num +
                    equ_con_continuity_num;  // p, v, a in x, y axis in each
                                             // segment's joint position

  int vel_ctrl_num = 2 * traj_order_ * segment_num;
  int acc_ctrl_num = 2 * (traj_order_ - 1) * segment_num;

  if (!ENFORCE_VEL)
    vel_ctrl_num = 0;

  if (!ENFORCE_ACC)
    acc_ctrl_num = 0;

  int high_order_con_num = vel_ctrl_num + acc_ctrl_num;

  int ctrlP_num = segment_num * ctrl_point_num;
  // csc矩阵
  /*
      1 0 4
      0 3 5
      2 0 6
      indptr = [0 , 2 , 3 , 6]
      indices = [0 , 2 , 1 , 0 , 1 , 2]
      data = [1 , 2, 3 , 4 , 5 ,6]
      indptr[i + 1]
      表示系数矩阵的第i列(包含第i列)之前一共有多少个非零元素(列序号从1开始)
      indices 表示非零元素所在行(行序号从0开始)
      data 表示非零元素
  */
  vector<c_float> A_data;
  vector<c_int> A_indices;
  vector<c_int> A_indptr;

  vector<c_float> lower_bounds;
  vector<c_float> upper_bounds;

  /*** constraints bound: bieq and beq ***/

  // bieq
  if (ENFORCE_VEL) {
    /***  Stack the bounding value for the linear inequality for the velocity
     * constraints  ***/
    for (int i = 0; i < vel_ctrl_num; i++) {
      lower_bounds.push_back(-max_vel_);
      upper_bounds.push_back(+max_vel_);
    }
  }
  // std::cout << lower_bounds.size();

  if (ENFORCE_ACC) {
    /***  Stack the bounding value for the linear inequality for the
     * acceleration constraints  ***/
    for (int i = 0; i < acc_ctrl_num; i++) {
      lower_bounds.push_back(-max_acc_);
      upper_bounds.push_back(+max_acc_);
    }
  }
  // std::cout << lower_bounds.size();
  // beq matrix
  for (int i = 0; i < equ_con_num; ++i) {
    double beq_i;
    if (i < 2)
      beq_i = pos(0, i);
    else if (i >= 2 && i < 4)
      beq_i = vel(0, i - 2);
    else if (i >= 4 && i < 6)
      beq_i = acc(0, i - 4);
    else if (i >= 6 && i < 8)
      beq_i = pos(1, i - 6);
    else if (i >= 8 && i < 10)
      beq_i = vel(1, i - 8);
    else if (i >= 10 && i < 12)
      beq_i = acc(1, i - 10);
    else
      beq_i = 0.0;

    lower_bounds.push_back(beq_i);
    upper_bounds.push_back(beq_i);
  }

  // std::cout << lower_bounds.size();

  /*** construct linear matrix A (varibles) ***/
  vector<vector<pair<c_int, c_float>>> variables(ctrlP_num);
  // col[i]:(row ,values)
  int constraint_index = 0;  // row index

  if (ENFORCE_VEL) {
    for (int k = 0; k < segment_num; ++k) {
      for (int i = 0; i < 2; i++) {  // x , y axis
        for (int p = 0; p < traj_order_; ++p) {
          int nnz = 2;
          
          int col_index[nnz];  // record the column index  of non zero
                               // components of matrix
          double val[nnz];

          val[0] = -1.0 * traj_order_;
          val[1] = 1.0 * traj_order_;

          col_index[0] = k * ctrl_point_num + i * d1_ctrl_point_num + p;
          col_index[1] = k * ctrl_point_num + i * d1_ctrl_point_num + p + 1;
          variables[col_index[0]].emplace_back(constraint_index, val[0]);
          variables[col_index[1]].emplace_back(constraint_index, val[1]);
          constraint_index++;
        }
      }
    }
  }

  if (ENFORCE_ACC) {
    for (int k = 0; k < segment_num; ++k) {
      for (int i = 0; i < 2; i++) {  // x , y axis
        for (int p = 0; p < traj_order_ - 1; ++p) {
          int nnz = 3;
          int col_index[nnz];  // record the column index of non zero
                               // components of matrix
          double val[nnz];

          val[0] = 1.0 * traj_order_ * (traj_order_ - 1) / corridor[k].t;
          val[1] = -2.0 * traj_order_ * (traj_order_ - 1) / corridor[k].t;
          val[2] = 1.0 * traj_order_ * (traj_order_ - 1) / corridor[k].t;

          col_index[0] = k * ctrl_point_num + i * d1_ctrl_point_num + p;
          col_index[1] = k * ctrl_point_num + i * d1_ctrl_point_num + p + 1;
          col_index[2] = k * ctrl_point_num + i * d1_ctrl_point_num + p + 2;
          variables[col_index[0]].emplace_back(constraint_index, val[0]);
          variables[col_index[1]].emplace_back(constraint_index, val[1]);
          variables[col_index[2]].emplace_back(constraint_index, val[2]);
          constraint_index++;
        }
      }
    }
  }

  /*   Start position  */
  {
    // position :
    for (int i = 0; i < 2; i++) {  // loop for x, y
      int nnz = 1;
      int asub[nnz];
      double aval[nnz];
      aval[0] = 1.0 * first_cube_time;
      asub[0] = i * d1_ctrl_point_num;

      variables[asub[0]].emplace_back(constraint_index, aval[0]);

      constraint_index++;
    }
    // velocity :
    for (int i = 0; i < 2; i++) {  // loop for x, y
      int nnz = 2;
      int asub[nnz];
      double aval[nnz];
      aval[0] = -1.0 * traj_order_;
      aval[1] = 1.0 * traj_order_;
      asub[0] = i * d1_ctrl_point_num;
      asub[1] = i * d1_ctrl_point_num + 1;

      variables[asub[0]].emplace_back(constraint_index, aval[0]);
      variables[asub[1]].emplace_back(constraint_index, aval[1]);

      constraint_index++;
    }
    // acceleration :
    for (int i = 0; i < 2; i++) {  // loop for x, y
      int nzi = 3;
      int asub[nzi];
      double aval[nzi];
      aval[0] = 1.0 * traj_order_ * (traj_order_ - 1) / first_cube_time;
      aval[1] = -2.0 * traj_order_ * (traj_order_ - 1) / first_cube_time;
      aval[2] = 1.0 * traj_order_ * (traj_order_ - 1) / first_cube_time;
      asub[0] = i * d1_ctrl_point_num;
      asub[1] = i * d1_ctrl_point_num + 1;
      asub[2] = i * d1_ctrl_point_num + 2;

      variables[asub[0]].emplace_back(constraint_index, aval[0]);
      variables[asub[1]].emplace_back(constraint_index, aval[1]);
      variables[asub[2]].emplace_back(constraint_index, aval[2]);
      constraint_index++;
    }
  }

  /*   End position  */
  // ROS_WARN(" end position");
  {
    // position :
    for (int i = 0; i < 2; i++) {  // loop for x, y
      int nzi = 1;
      int asub[nzi];
      double aval[nzi];
      asub[0] = ctrlP_num - 1 - (1 - i) * d1_ctrl_point_num;
      aval[0] = 1.0 * last_cube_time;
      // r = MSK_putarow(task, row_idx, nzi, asub, aval);
      // row_idx ++;
      variables[asub[0]].emplace_back(constraint_index, aval[0]);
      constraint_index++;
    }
    // velocity :
    for (int i = 0; i < 2; i++) {
      int nzi = 2;
      int asub[nzi];
      double aval[nzi];
      asub[0] = ctrlP_num - 1 - (1 - i) * d1_ctrl_point_num - 1;
      asub[1] = ctrlP_num - 1 - (1 - i) * d1_ctrl_point_num;
      aval[0] = -1.0 * traj_order_;
      aval[1] = 1.0 * traj_order_;
      // r = MSK_putarow(task, row_idx, nzi, asub, aval);
      // row_idx ++;
      variables[asub[0]].emplace_back(constraint_index, aval[0]);
      variables[asub[1]].emplace_back(constraint_index, aval[1]);
      constraint_index++;
    }
    // acceleration :
    for (int i = 0; i < 2; i++) {
      int nzi = 3;
      int asub[nzi];
      double aval[nzi];
      asub[0] = ctrlP_num - 1 - (1 - i) * d1_ctrl_point_num - 2;
      asub[1] = ctrlP_num - 1 - (1 - i) * d1_ctrl_point_num - 1;
      asub[2] = ctrlP_num - 1 - (1 - i) * d1_ctrl_point_num;
      aval[0] = 1.0 / last_cube_time * traj_order_ * (traj_order_ - 1);
      aval[1] = -2.0 / last_cube_time * traj_order_ * (traj_order_ - 1);
      aval[2] = 1.0 / last_cube_time * traj_order_ * (traj_order_ - 1);
      // r = MSK_putarow(task, row_idx, nzi, asub, aval);
      // row_idx ++;
      variables[asub[0]].emplace_back(constraint_index, aval[0]);
      variables[asub[1]].emplace_back(constraint_index, aval[1]);
      variables[asub[2]].emplace_back(constraint_index, aval[2]);
      constraint_index++;
    }
  }

  /*   continuous constraints */
  {
    int sub_shift = 0;
    double val0, val1;
    for (int k = 0; k < (segment_num - 1); k++) {
      double time_this = corridor[k].t;
      double time_next = corridor[k + 1].t;
      // position continuous constraints
      val0 = time_this;
      val1 = time_next;
      for (int i = 0; i < 2; i++) {  // loop for x, y
        int nzi = 2;
        int asub[nzi];
        double aval[nzi];

        // This segment's last control point
        aval[0] = 1.0 * val0;
        asub[0] = sub_shift + (i + 1) * d1_ctrl_point_num - 1;

        // Next segment's first control point
        aval[1] = -1.0 * val1;
        asub[1] = sub_shift + ctrl_point_num + i * d1_ctrl_point_num;

        variables[asub[0]].emplace_back(constraint_index, aval[0]);
        variables[asub[1]].emplace_back(constraint_index, aval[1]);
        constraint_index++;
      }
      // velocity
      for (int i = 0; i < 2; i++) {
        int nzi = 4;
        int asub[nzi];
        double aval[nzi];

        // This segment's last velocity control point
        aval[0] = -1.0 * traj_order_;
        aval[1] = 1.0 * traj_order_;
        asub[0] = sub_shift + (i + 1) * d1_ctrl_point_num - 2;
        asub[1] = sub_shift + (i + 1) * d1_ctrl_point_num - 1;
        // Next segment's first velocity control point
        aval[2] = 1.0 * traj_order_;
        aval[3] = -1.0 * traj_order_;

        asub[2] = sub_shift + ctrl_point_num + i * d1_ctrl_point_num;
        asub[3] = sub_shift + ctrl_point_num + i * d1_ctrl_point_num + 1;

        variables[asub[0]].emplace_back(constraint_index, aval[0]);
        variables[asub[1]].emplace_back(constraint_index, aval[1]);
        variables[asub[2]].emplace_back(constraint_index, aval[2]);
        variables[asub[3]].emplace_back(constraint_index, aval[3]);
        constraint_index++;
      }
      // acceleration :
      val0 = 1.0 / time_this;
      val1 = 1.0 / time_next;
      for (int i = 0; i < 2; i++) {
        int nzi = 6;
        int asub[nzi];
        double aval[nzi];

        // This segment's last velocity control point
        aval[0] = 1.0 * val0 * traj_order_ * (traj_order_ - 1);
        aval[1] = -2.0 * val0 * traj_order_ * (traj_order_ - 1);
        aval[2] = 1.0 * val0 * traj_order_ * (traj_order_ - 1);
        asub[0] = sub_shift + (i + 1) * d1_ctrl_point_num - 3;
        asub[1] = sub_shift + (i + 1) * d1_ctrl_point_num - 2;
        asub[2] = sub_shift + (i + 1) * d1_ctrl_point_num - 1;
        // Next segment's first velocity control point
        aval[3] = -1.0 * val1 * traj_order_ * (traj_order_ - 1);
        aval[4] = 2.0 * val1 * traj_order_ * (traj_order_ - 1);
        aval[5] = -1.0 * val1 * traj_order_ * (traj_order_ - 1);
        asub[3] = sub_shift + ctrl_point_num + i * d1_ctrl_point_num;
        asub[4] = sub_shift + ctrl_point_num + i * d1_ctrl_point_num + 1;
        asub[5] = sub_shift + ctrl_point_num + i * d1_ctrl_point_num + 2;

        variables[asub[0]].emplace_back(constraint_index, aval[0]);
        variables[asub[1]].emplace_back(constraint_index, aval[1]);
        variables[asub[2]].emplace_back(constraint_index, aval[2]);
        variables[asub[3]].emplace_back(constraint_index, aval[3]);
        variables[asub[4]].emplace_back(constraint_index, aval[4]);
        variables[asub[5]].emplace_back(constraint_index, aval[5]);
        constraint_index++;
      }

      sub_shift += ctrl_point_num;
    }
  }
  /****  safe constraint *****/
  for (int k = 0; k < segment_num; k++) {
    Cube cube_ = corridor[k];
    double time = cube_.t;
    // std::cout << "time is " << time << std::endl;

    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < d1_ctrl_point_num; j++) {
        int asub;
        double aval;
        double lo_bound, up_bound;
        asub = k * ctrl_point_num + i * d1_ctrl_point_num + j;
        aval = 1.0;
        if (k > 0) {
          lo_bound =
              (cube_.box[i].first) / time;  // margin 为安全值,等效为障碍物膨胀
          up_bound = (cube_.box[i].second) / time;
        } else {
          lo_bound = (cube_.box[i].first) / time;
          up_bound = (cube_.box[i].second) / time;
        }
        variables[asub].emplace_back(constraint_index, aval);
        lower_bounds.push_back(lo_bound);
        upper_bounds.push_back(up_bound);
        constraint_index++;
      }
      // std:: cout << "lo_bound of cube  is " <<
      // cube_.box[i].first<<std::endl; std:: cout << "up_bound of cube  is "
      // << cube_.box[i].second<<std::endl;
    }
  }
  // std::cout << lower_bounds.size();

  /** construct CSC Matrix to populate the parameter to OSQP ****/
  int ind_p = 0;
  for (int i = 0; i < ctrlP_num; ++i)  // i : column index of linear matrix A
  {
    A_indptr.push_back(ind_p);
    for (const auto& nz_val :
         variables[i])  // loop for  data for every  row of matrix A
    {
      // non zero valuable
      A_data.push_back(nz_val.second);
      // row index of non zero valuable
      A_indices.push_back(nz_val.first);
      ++ind_p;
    }
  }

  // for(int i = 0 ; i  < A_data.size(); ++i)
  // {
  //    std::cout << "A_Data  " << A_data[i] << std::endl;
  // }

  A_indptr.push_back(ind_p);  // total number of non zero date of A Matirx

  int min_order_l = floor(min_order_);  //向小取整
  int min_order_u = ceil(min_order_);   //向大取整

  int MQM_nnz = 0;  // non zero numbers of MQM 只记录了下三角的元素个数
                    // osqp的官网列子也是这样做的
  for (int i = 0; i < segment_num; ++i) {
    int MQM_nnz_blk = (traj_order_ + 1);
    MQM_nnz += 2 * MQM_nnz_blk * (MQM_nnz_blk + 1) / 2;
  }

  vector<c_float> MQM_data;
  vector<c_int> MQM_indices;
  vector<c_int> MQM_indptr;
  {
    int sub_shift = 0;
    int idx = 0;
    for (int k = 0; k < segment_num; k++) {
      double scale_k = corridor[k].t;
      for (int p = 0; p < 2; p++)  // loop for  x, y
      {
        for (int j = 0; j < d1_ctrl_point_num; j++)  // column index
        {
          MQM_indptr.push_back(idx);
          for (int i = 0; i < d1_ctrl_point_num; i++)  // i : row index
          {
            if (j >= i)  //只填入下三角矩阵内的元素
            {
              MQM_indices.push_back(
                  sub_shift + p * d1_ctrl_point_num + i);  // row_idx
              if (min_order_l == min_order_u)
                MQM_data.push_back(
                    MQM_(i, j) / (double)pow(scale_k, 2 * min_order_u - 3));
              else
                MQM_data.push_back(
                    ((min_order_ - min_order_l) /
                         (double)pow(scale_k, 2 * min_order_u - 3) +
                     (min_order_u - min_order_) /
                         (double)pow(scale_k, 2 * min_order_l - 3)) *
                    MQM_(i, j));
              idx++;
            }
          }
        }
      }

      sub_shift += ctrl_point_num;
    }
    MQM_indptr.push_back(idx);
  }
  vector<c_float> q;
  for (int i = 0; i < ctrlP_num; i++) {
    q.push_back(0.0);
  }

  // OSQP Solver setting
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  OSQPWorkspace* work;
  OSQPSettings* settings = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
  size_t kernal_dim = ctrlP_num;
  size_t cons_bd_dim = lower_bounds.size();

  data->n = kernal_dim;
  data->m = cons_bd_dim;
  data->P = csc_matrix(
      kernal_dim, kernal_dim, MQM_nnz, CopyData(MQM_data),
      CopyData(MQM_indices), CopyData(MQM_indptr));

  data->q = CopyData(q);
  data->A = csc_matrix(
      cons_bd_dim, kernal_dim, A_data.size(), CopyData(A_data),
      CopyData(A_indices), CopyData(A_indptr));
  data->l = CopyData(lower_bounds);
  data->u = CopyData(upper_bounds);

  ros::Time befor_sol = ros::Time::now();
  c_int exitflag = 0;
  if (settings)
    osqp_set_default_settings(settings);

  // Setup workspace
  exitflag = osqp_setup(&work, data, settings);

  // Solve problem
  osqp_solve(work);

  auto status = work->info->status_val;  // whether solve succesfully

  // solve fail
  if (status < 0 || (status != 1 && status != 2)) {
    osqp_cleanup(work);
    FreeData(data);
    c_free(settings);
    return -1;
  } else if (work->solution == nullptr) {
    osqp_cleanup(work);
    FreeData(data);
    c_free(settings);
    return -1;
  }
  // solve success
  ros::Time time_end2 = ros::Time::now();
  ROS_WARN("time consume in optimize is :");
  std::cout << (time_end2 - befor_sol) << std::endl;

  VectorXd ctrlP_sol(ctrlP_num);  // save solution of control points
  for (int i = 0; i < ctrlP_num; ++i) {
    ctrlP_sol[i] = work->solution->x[i];
  }

  polyCoeff_ = MatrixXd::Zero(segment_num, 2 * (traj_order_ + 1));
  int sol_index = 0;
  for (int i = 0; i < segment_num; ++i) {
    for (int j = 0; j < 2 * (traj_order_ + 1); ++j) {
      polyCoeff_(i, j) = ctrlP_sol(sol_index);
      sol_index++;
      // std::cout <<polyCoeff(i,j) << std::endl;
    }
  }

  osqp_cleanup(work);
  FreeData(data);
  c_free(settings);
  return true;
}

OSQPSettings* TrajectoryGenerator::SolverDefaultSettings() {
  // Define Solver default settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->polish = true;
  settings->verbose = false;
  settings->scaled_termination = true;
  return settings;
}
void TrajectoryGenerator::FreeData(OSQPData* data) {
  delete[] data->q;
  delete[] data->l;
  delete[] data->u;

  delete[] data->P->i;
  delete[] data->P->p;
  delete[] data->P->x;

  delete[] data->A->i;
  delete[] data->A->p;
  delete[] data->A->x;
}

void TrajectoryGenerator::traj_vis(VectorXd time) {
  {
    visualization_msgs::Marker traj_vis;

    traj_vis.header.stamp = ros::Time::now();
    traj_vis.header.frame_id = "map";

    traj_vis.ns = "trajectory/trajectory";
    traj_vis.id = 0;
    traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;

    traj_vis.action = visualization_msgs::Marker::DELETE;

    traj_vis.action = visualization_msgs::Marker::ADD;
    traj_vis.scale.x = 0.05;
    traj_vis.scale.y = 0.05;
    traj_vis.scale.z = 0.05;
    traj_vis.pose.orientation.x = 0.0;
    traj_vis.pose.orientation.y = 0.0;
    traj_vis.pose.orientation.z = 0.0;
    traj_vis.pose.orientation.w = 1.0;
    traj_vis.color.r = 1.0;
    traj_vis.color.g = 0.0;
    traj_vis.color.b = 0.0;
    traj_vis.color.a = 0.6;

    double traj_len = 0.0;
    int count = 0;
    Vector2d cur, pre;
    cur.setZero();
    pre.setZero();

    traj_vis.points.clear();  // 清空以前的轨迹点

    Vector2d state;
    geometry_msgs::Point pt;

    int segment_num = (int)polyCoeff_.rows();
    for (int i = 0; i < segment_num; i++) {
      // 因为是标准的贝塞尔曲线, 所以需考虑缩放系数
      for (double t = 0.0; t < 1.0; t += 0.05 / time[i], count++) {
        state =
            getPosFromBezier(polyCoeff_, t, i);  // 从标准的贝塞尔曲线得到pos
        cur(0) = pt.x =
            time(i) * state(0);  // 这里的time(i)是比例系数,将坐标放大到真实值
        cur(1) = pt.y = time(i) * state(1);
        traj_vis.points.push_back(pt);

        if (count)
          traj_len += (pre - cur).norm();  // 轨迹长度
        pre = cur;
      }
    }
    // std::cout << "number of points is :" <<
    // traj_vis.points.size()<<std::endl;
    ROS_INFO("[GENERATOR] The length of the trajectory; %.3lfm.", traj_len);
    traj_vis_pub.publish(traj_vis);
  }
}

void TrajectoryGenerator::pub_traj2ctrl(
    const MatrixXd polyCoeff, const VectorXd time, const double final_yaw) {
  if (polyCoeff.size() == 0 || time.size() == 0) {
    ROS_WARN(
        "[trajectory_generator_waypoint] empty trajectory, nothing to "
        "publish.");
    return;
  }

  unsigned int poly_number;

  static int count = 1;

  btraj_msgs::trajectory traj_msg;

  traj_msg.header.seq = count;  // recored the time of trajectory generation
  traj_msg.header.stamp = ros::Time::now();
  traj_msg.header.frame_id = std::string("map");
  traj_msg.trajectory_id = count;
  traj_msg.time_sum = 0.0;
  traj_msg.num_order = traj_order_;  // the order of polynomial
  traj_msg.num_segment = time.size();
  traj_msg.final_yaw = final_yaw;
  poly_number = traj_msg.num_order + 1;
  for (unsigned int i = 0; i < traj_msg.num_segment; i++) {
    for (unsigned int j = 0; j < poly_number; j++) {
      traj_msg.coef_x.push_back(polyCoeff(i, j));
      traj_msg.coef_y.push_back(polyCoeff(i, poly_number + j));
    }
    traj_msg.time.push_back(time(i));
    traj_msg.order.push_back(traj_msg.num_order);
    traj_msg.time_sum += time(i);
  }
  traj_msg.mag_coeff = 1;

  count++;
  ROS_WARN("[traj..gen...node] traj_msg publish");
  traj_pub_.publish(traj_msg);
}
Vector2d TrajectoryGenerator::getPosFromBezier(
    const MatrixXd& polyCoeff, double t_now, int seg_now) {
  Vector2d ret = VectorXd::Zero(2);  // x, y,的pos
  VectorXd ctrl_now = polyCoeff.row(seg_now);
  int ctrl_num1D = polyCoeff.cols() / 2;

  for (int i = 0; i < 2; i++)  // x, y
    for (int j = 0; j < ctrl_num1D; j++)
      ret(i) += Clist_(j) * ctrl_now(i * ctrl_num1D + j) * pow(t_now, j) *
                pow((1 - t_now), (traj_order_ - j));

  return ret;
}
Vector2d TrajectoryGenerator::getVelFromBezier(
    const MatrixXd& polyCoeff, double t_now, int seg_now) {
  Vector2d vel = VectorXd::Zero(2);
  VectorXd ctrl_now = polyCoeff.row(seg_now);
  int ctrl_num1D = polyCoeff.cols() / 2;

  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < ctrl_num1D - 1; ++j) {
      vel(i) +=
          traj_order_ *
          (ctrl_now(i * ctrl_num1D + j + 1) - ctrl_now(i * ctrl_num1D + j)) *
          CvList_(j) * pow(t_now, j) * pow((1 - t_now), (traj_order_ - 1) - j);
    }
  return vel;
}

bool TrajectoryGenerator::trajPlanning() {
  if (!ref_traj_.empty())
    ref_traj_.clear();
  safe_corridor_.frontEndPathFinder(odom_pos_, target_pos_, odom_vel_);
  MatrixXd pos = MatrixXd::Zero(2, 2);
  MatrixXd vel = MatrixXd::Zero(2, 2);
  MatrixXd acc = MatrixXd::Zero(2, 2);
  pos.row(0) = odom_pos_;
  pos.row(1) = target_pos_;
  if (gen_traj_num_ == 0) {
    vel(0, 0) = 0.2;
    vel(0, 1) = 0.001;
  } else
    vel.row(0) = odom_vel_;
  ros::Time time_bef_opt = ros::Time::now();
  auto corridor = safe_corridor_.getCorridor();
  if (BezierPloyCoeffGeneration(corridor, pos, vel, acc) == - 1) {
    ROS_WARN(
        "Cannot find a feasible and optimal solution, somthing wrong with "
        "the OSQP solver");
    has_traj_ = false;
    return false;
  } else {
    has_traj_ = true;
    traj_num_ = (int)corridor.size();
    seg_time_.resize(traj_num_);
    for (int i = 0; i < traj_num_; i++)
      seg_time_(i) = corridor[i].t;
    traj_vis(seg_time_);
    pub_traj2ctrl(polyCoeff_, seg_time_, final_yaw_);
    ++gen_traj_num_;
  }
  ros::Time traj_start_time = ros::Time::now();
  ROS_WARN(
      "The time consumation of the program is %f",
      (traj_start_time - time_bef_opt).toSec());
  for (int i = 0; i < traj_num_; ++i)
    for (double t = 0.0; t < 1.0; t += 0.05 / seg_time_[i]) {
      Vector2d ref_pt = seg_time_[i] * getPosFromBezier(polyCoeff_, t, i);
      ref_traj_.push_back(ref_pt);
    }
  return true;
}

bool TrajectoryGenerator::trajCheck() {
  double x, y;
  int size = (int)ref_traj_.size();
  for (int i = 0; i < size; ++i) {
    x = ref_traj_[i](0);
    y = ref_traj_[i](1);
    Vector2d coord(x, y);
    Vector2i index = safe_corridor_.getGridmapIndex(coord);
    if (safe_corridor_.isOccupy(index)) {
      return false;
    }
  }
  return true;
}