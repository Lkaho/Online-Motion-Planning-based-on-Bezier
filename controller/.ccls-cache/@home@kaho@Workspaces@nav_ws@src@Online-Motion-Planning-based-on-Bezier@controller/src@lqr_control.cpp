#include "controller/lqr_control.h"

void LQRController::register_pub(ros::NodeHandle &nh) {
  cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
}
void LQRController::setparams(double sample_t, double L, double thresh,
                              double iter_max) {
  _sample_t = sample_t;
  _L = L;
  _thresh = thresh;
  _iter_max = iter_max;
}

int LQRController::find_nearest_pt(vector<Vector4d> &ref, Vector4d current_ps) {
  int min_idx = 0;
  double min_dist = sqrt(pow((ref[0](0) - current_ps(0)), 2) +
                         pow((ref[0](1) - current_ps(1)), 2));
  for (int i = 1; i < ref.size(); ++i) {

    double dist = sqrt(pow((ref[i](0) - current_ps(0)), 2) +
                       pow((ref[i](1) - current_ps(1)), 2));
    if (dist < min_dist) {
      min_idx = i;
      min_dist = dist;
    }
  }
  return min_idx;
}

MatrixXd LQRController::computeK() {
  MatrixXd P_old = Q;
  MatrixXd P_new;
  for (int i = 0; i < _iter_max; ++i) {
    P_new = A.transpose() * P_old * A -
            (A.transpose() * P_old * B) *
                (R + B.transpose() * P_old * B).inverse() *
                (B.transpose() * P_old * A) +
            Q;
    if (compare(P_old, P_new))
      break;
    else
      P_old = P_new;
  }
  MatrixXd P = P_new;
  MatrixXd K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
  return K;
}

bool LQRController::compare(Matrix<double, 3, 3> P_old,
                            Matrix<double, 3, 3> P_new) {
  double res;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      res = fabs(P_new(i, j) - P_old(i, j));
      if (res > _thresh)
        return false;
    }
  return true;
}

void LQRController::updateMatrix(double heading_ref, double vel_ref) {
  A << 1, 0, -vel_ref * sin(heading_ref) * _sample_t, 0, 1,
      vel_ref * cos(heading_ref) * _sample_t, 0, 0, 1;

  B << cos(heading_ref) * _sample_t, 0, sin(heading_ref) * _sample_t, 0, 0,
      _sample_t;
}

geometry_msgs::Twist LQRController::LQRsolver(Vector4d ref_status,
                                              Vector4d cur_status) {
  geometry_msgs::Twist cmd;
  double x_ref = ref_status(0);
  double y_ref = ref_status(1);
  double heading_ref = ref_status(2);
  double v_ref = ref_status(3);

  double cur_x = cur_status(0);
  double cur_y = cur_status(1);
  double cur_heading = cur_status(2);
  double v_cur = cur_status(3);

  double x_error = cur_x - x_ref;
  double y_error = cur_y - y_ref;
  double theta_error = cur_heading - heading_ref;
  Vector3d state_error(x_error, y_error, theta_error);

  updateMatrix(heading_ref, 0.3);

  MatrixXd K = computeK();

  Vector2d st_error = -K * state_error;

  cmd.linear.x = 0.3 + st_error(0);
  cmd.angular.z = st_error(1);

  return cmd;
}

void LQRController::pub_cmd(geometry_msgs::Twist &command) {
  cmd_pub.publish(command);
}