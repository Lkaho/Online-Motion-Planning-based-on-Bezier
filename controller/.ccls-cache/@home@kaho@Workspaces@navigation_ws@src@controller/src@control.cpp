#include "ros/ros.h"
#include "controller/lqr_control.h"
#include "navigation/trajectory_generator.h"
#include "ros/console.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "Eigen/Eigen"
#include <iostream>
#include "navigation/bezier.h"
#include <vector>
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "btraj_msgs/trajectory.h"

using namespace std;
using namespace Eigen;
#define PI  3.14159
int num1 = 0;
int record = 0;
double ref_theta_last = 0.0;

//useful global object
LQRController myController;
btraj_msgs::trajectory cur_traj;
//geometry_msgs::Twist cmd;
double replan_t = 3.0;
double _minimize_order;
int _traj_order;
VectorXd _C, _Cv;
vector<Vector4d> cur_ref;
Vector4d current_st;
Vector2d _odom_ps , _odom_vel;
MatrixXd coef;
bool _has_odom = false;
bool _has_traj = false;
ros::Publisher ctrl_traj_vis;
double final_yaw;

void traj_vis(MatrixXd polyCoeff, vector<double>  time);
/*
        getXPosFromBezier , getXVelFromBezier , getYPosFromBezier ,getYVelFromBezier
        
        to get the refference postion and velocity from the trajectory
*/
Vector2d getPosFromBezier(const MatrixXd& _coef, double t_now , int seg_now)
{
    Vector2d ret = VectorXd::Zero(2); // x, y,的pos
    int ctrl_num1D = _traj_order + 1;
    VectorXd ctrl_now = _coef.row(seg_now);
    
    for (int i = 0; i < 2; i++) // x, y
        for (int j = 0; j < ctrl_num1D; j++)
            ret(i) += _C(j) * ctrl_now(i * ctrl_num1D + j) * pow(t_now, j) * pow((1 - t_now), (_traj_order- j));
 
    return ret;
}
Vector2d getVelFromBezier(const MatrixXd& _coef , double t_now , int seg_now)
{
    Vector2d vel = VectorXd::Zero(2); // x, y,的vel
    int ctrl_num1D = _traj_order + 1;
    VectorXd ctrl_now = _coef.row(seg_now);

     for(int i = 0; i < 2; ++i)
     {
        for(int j  = 0; j <  ctrl_num1D-1; ++j)
        {
            vel(i)  +=  _Cv(j) * _traj_order  * (ctrl_now(i * ctrl_num1D + j + 1) - ctrl_now(i * ctrl_num1D + j))  * pow(t_now , j) * pow((1- t_now), (_traj_order-1 - j));
            
        }
       // std::cout << " v is :" << vel(i) << std::endl;
     }
        return vel;
}

void ctrlCallback(const ros::TimerEvent& event)
{
    if(!_has_traj || !_has_odom) return;
    num1++;
    geometry_msgs::Twist cmd;
    int num = cur_ref.size();
    int min_idx = myController.find_nearest_pt(cur_ref , current_st);
    Vector2d terminal_pt(cur_ref[num-1](0) , cur_ref[num-1](1));
    Vector4d ref_st = cur_ref[min_idx];
    Vector2d cur_ref_pt(ref_st(0) , ref_st(1));
    if((cur_ref_pt - _odom_ps).norm() > 0.3 )
    {
        cmd.angular.z = 0.0;
        cmd.linear.x     = 0.0;
        myController.pub_cmd(cmd);
        return ;
    }

    if((terminal_pt - _odom_ps).norm() < 0.1 )
    {
        //std::cout << "car has already arrive the target"<<(terminal_pt - _odom_ps).norm() << std::endl;
        cmd.angular.z = 0.0;
        cmd.linear.x     = 0.0;
        //std::cout << "vel is :" << cmd.linear.x  << std::endl;
        //std::cout << "w is :" << cmd.angular.z << std::endl;
        myController.pub_cmd(cmd);
        return ;
    }

        cmd = myController.LQRsolver(ref_st , current_st);
        if(cmd.angular.z > 0.5)
            cmd.angular.z = 0.5;
        if(cmd.angular.z < -0.5)
            cmd.angular.z = -0.5;

        cmd.linear.x = 0.35;
        /*
        if(num == 20)
        {
            std::cout << " current t is: " << t << std::endl;
            std::cout << "ref_ps is :" << ref_ps << std::endl;
            std::cout << "cur_x is " <<current_st(0) << std::endl;
            std::cout << "cur_y is " <<current_st(1) << std::endl;
            std::cout << "ref_vel is :" << ref_vel << std::endl;
            std::cout << "ref_theta is :" << ref_theta << std::endl;
            std::cout << " cur_theta is : " << current_st(2) << std::endl;
            std::cout << "cmd vel is : " << cmd.linear.x << std::endl;
            std::cout << "cmd angular is : " << cmd.angular.z << std::endl;
            num=0;
        }
        */
       /*
       if(num1 == 5)
       {
            std::cout << "target point is" << terminal_pt << std::endl;
            std::cout << "target point index is" << min_idx << std::endl;
            std::cout << "number of points is :" << num << std::endl;
            num1=0;
       }
       */
       myController.pub_cmd(cmd);
       //std::cout << "exec the traj ! " << std::endl;
       return ;

}

void odom2Callback(const nav_msgs::Odometry& odom)
{
    _has_odom = true;
    //std::cout << " has receive odom data !" << std::endl; 
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom.pose.pose.orientation , quat);
    double linear_vel , roll , pitch , yaw;
    tf::Matrix3x3(quat).getRPY(roll , pitch , yaw);
    _odom_ps(0) = current_st(0) = odom.pose.pose.position.x;
    _odom_ps(1) = current_st(1) = odom.pose.pose.position.y;
    current_st(2) = yaw;
    current_st(3) = odom.twist.twist.linear.x;
}
void trajCallback(const btraj_msgs::trajectory& traj)
{
    _has_traj = true;
    std::cout << " has receive trajectory infomation !" << std::endl; 
    cur_ref.clear();
    cur_traj = traj;
    int seg_num = cur_traj.num_segment;
    int d1_ctrlP_num = cur_traj.num_order + 1;
    double time = 0.0;
    final_yaw = cur_traj.final_yaw;
    coef = MatrixXd::Zero(seg_num , 2 * d1_ctrlP_num);
    vector<double> time_list = cur_traj.time;
    for(int i = 0 ; i < seg_num ; ++i)
    {
        for(int j  = 0;  j <  d1_ctrlP_num * 2 ;++j)
            {
                if(j < d1_ctrlP_num)
                    coef(i , j) = cur_traj.coef_x[i * d1_ctrlP_num + j];
                else
                    coef(i, j ) = cur_traj.coef_y[i * d1_ctrlP_num + j - d1_ctrlP_num];
                
            }
    }
        traj_vis(coef , time_list);
    //record the refference points: x , y , theta
    for(int i = 0; i < cur_traj.num_segment; ++i)
        for(double t = 0.0; t < 1.0; t += 0.05 / cur_traj.time[i])
            {
                Vector2d ref_ps = cur_traj.time[i] * getPosFromBezier(coef , t , i);
                Vector2d ref_vel = getVelFromBezier(coef , t , i);
                double v_ref = ref_vel.norm();
                double ref_x = ref_ps(0);
                double ref_y = ref_ps(1);
                double v_x = ref_vel(0);
                double v_y = ref_vel(1);
                double ref_theta = atan2(v_y , v_x);
                Vector4d ref_st(ref_ps(0) , ref_ps(1) , ref_theta , v_ref);
                cur_ref.push_back(ref_st);
            }
            Vector4d target_pt_st = cur_ref.back();
            Vector4d modify_yaw(target_pt_st(0) , target_pt_st(1) , final_yaw , target_pt_st(3));
            cur_ref.pop_back();
            cur_ref.push_back(modify_yaw);
            //std::cout << cur_ref.back() << std::endl;
            //std::cout <<  "total point is :" << cur_ref.size() << std::endl; 
}
int main(int argc, char ** argv)
{
    ros::init(argc , argv , "control_node");
    ros::NodeHandle nh1;
    ros::Subscriber  traj_sub = nh1.subscribe("btraj_node/trajToctrl" , 10 , trajCallback);
    ctrl_traj_vis = nh1.advertise<visualization_msgs::Marker>("ctrl_traj" , 10);
    ros::Subscriber ctrl_odom_sub = nh1.subscribe("odom" , 10 , odom2Callback);
    ros::Timer ctrl_timer = nh1.createTimer(ros::Duration(0.03) , ctrlCallback);
    double sample_t , thresh , L ;
    int iter_max;
    nh1.param("control/thresh" , thresh , 0.01);
    nh1.param("control/iter_max" , iter_max , 200);
    nh1.param("control/car_len" , L , 0.5);
    nh1.param("control/sample_t" , sample_t , 0.05);
    nh1.param("control/traj_order" ,  _traj_order , 5);
    nh1.param("control/min_order" , _minimize_order , 2.5);



    myController.setparams(sample_t , L , thresh , iter_max);
    myController.register_pub(nh1);
    Bernstein _bernstein;

    if(_bernstein.setParam(3 ,12 ,_minimize_order) == -1)
        ROS_ERROR(" The trajectory order is set beyond the library's scope, please re-set "); 
    
    _C   = _bernstein.getC()[_traj_order];
    _Cv  = _bernstein.getC_v()[_traj_order];
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spin();
        loop_rate.sleep();
    }
    

    return 0;
}

void traj_vis(MatrixXd polyCoeff, vector<double>  time)
{
    {   
    visualization_msgs::Marker traj_vis;

    traj_vis.header.stamp       = ros::Time::now();
    traj_vis.header.frame_id    = "map";

    traj_vis.ns = "control/trajectory";
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
    traj_vis.color.r = 0.0;
    traj_vis.color.g = 1.0;
    traj_vis.color.b = 0.0;
    traj_vis.color.a = 0.6;

    double traj_len = 0.0;
    int count = 0;
    Vector2d cur, pre;
    cur.setZero();
    pre.setZero();
    
    traj_vis.points.clear(); // 清空以前的轨迹点

    Vector2d state;
    geometry_msgs::Point pt;

    int segment_num = polyCoeff.rows();
    for (int i = 0; i < segment_num; i++)
    {
        // 因为是标准的贝塞尔曲线, 所以需考虑缩放系数
        for (double t = 0.0; t < 1.0; t += 0.05/ time[i], count++)
        {
            state = getPosFromBezier(polyCoeff, t, i); // 从标准的贝塞尔曲线得到pos
            cur(0) = pt.x = time[i] * state(0); // 这里的time(i)是比例系数,将坐标放大到真实值
            cur(1) = pt.y = time[i] * state(1);
            traj_vis.points.push_back(pt);

            if (count)
                traj_len += (pre - cur).norm(); // 轨迹长度
            pre = cur;
        }
        
    }
   // std::cout << "number of points is :" << traj_vis.points.size()<<std::endl;
    //ROS_INFO("[GENERATOR] The length of the trajectory; %.3lfm.", traj_len);
    ctrl_traj_vis.publish(traj_vis);
}
}