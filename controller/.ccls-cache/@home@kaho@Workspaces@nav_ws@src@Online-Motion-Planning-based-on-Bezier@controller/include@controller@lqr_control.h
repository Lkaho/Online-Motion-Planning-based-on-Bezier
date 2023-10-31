#ifndef _CONTROLLER_LQR_CONTROL_H
#define _CONTROLLER_LQR_CONTROL_H
#include "ros/ros.h"
#include "Eigen/Eigen"
#include "btraj_msgs/trajectory.h"
#include "geometry_msgs/Twist.h"
#include <vector>
using namespace Eigen;
using namespace std;

class LQRController
{
private:
    ros::Publisher cmd_pub;
    double _sample_t;
    double _L;
    double _thresh;
    double _iter_max;
    MatrixXd A = MatrixXd::Zero(3 , 3);
    MatrixXd B = MatrixXd::Zero(3, 2);
    MatrixXd Q = MatrixXd::Identity(3, 3) * 100;
    MatrixXd R = MatrixXd::Identity(2 , 2) * 50;
public:
    void register_pub(ros::NodeHandle& nh);
    int find_nearest_pt(vector<Vector4d>& ref  , Vector4d current_ps);
    void setparams(double sample_t , double L , double thresh , double iter_max);
    void updateMatrix(double heading_ref , double vel_ref);
    bool compare(Matrix<double , 3 ,3> P_old , Matrix<double , 3 ,3> P_new);
    geometry_msgs::Twist  LQRsolver( Vector4d ref_status , Vector4d cur_status );
    void pub_cmd(geometry_msgs::Twist& command);
    MatrixXd computeK();
    LQRController(/* args */){};
    ~LQRController(){};
};

#endif