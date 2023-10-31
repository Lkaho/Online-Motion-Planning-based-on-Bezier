# A Robust and Efficent Navigation Framework for  Indoor Wheel-based Robot

In this project, we designed a robust and efficent navigation framework for indoor mobile robot in unknown environments. For the motion planning part, based on the onboard state estimation and environment perception, we implemented A* for front-end path searching and generated back-end corridor-based trajectory by using Bezier curves. For the control part, we implemented path tracking controller by using LQR and MPC algorithm respectively.

## 1. Main contribution

- We implement A-star method for front-end path searching.(see navigation/src/astar.cpp)
- We implement corridor generator for extracting safe region from occupancy map for mobile robot. (see navigation/src/safe_corridor_generator.cpp)
- We ues OSQP_Interface to solve QP problem to generate Bezier curve. Thanks to the Hodograph property of Bezier curves,, we are able to bound the position within safe regions and limit the higher order dynamics of the trajectory within feasible regions.(see navigation/src/trajectory_generator.cpp)
- We implement LQR path tracking controller to track the path.(see controller/src)

## 2. Installation

### Dependencies

This software is built on the Robotic Operating System ([ROS](http://www.ros.org/)), which needs to be [installed](http://wiki.ros.org/) first. Additionally, the Navigation Framework depends on following software:

- [Point Cloud Library (PCL)](http://pointclouds.org/) (point cloud processing),
- [Eigen](http://eigen.tuxfamily.org/) (linear algebra library).
- [catkin_simple](https://github.com/catkin/catkin_simple) 
- [gflag_catkin](https://github.com/ethz-asl/gflags_catkin.git) 
- [OSQP](https://osqp.org/) 

### Building

In order to install the Navigation Framework, clone the latest version from this repository into your catkin workspace and compile the package using ROS.

```
cd catkin_workspace/src
git clone https://github.com/Lkaho/Online-Motion-Planning-based-on-Bezier.git
cd Online-Motion-Planning-based-on-Bezier
git submodule init
git submodule update
cd ../../
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

## Basic Usage

### TurtleBot3 Waffle Simulation

A running example is provided, making use of the Turtlebot3 simulation environment. 

To start with, the Turtlebot3 simulation dependencies need to be installed:

```
sudo apt install ros-noetic-turtlebot3*
```

Launch the turtlebot3 gazebo simulation:

```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

Run turtlebot3 slam for visulization:

```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

Run navigation system:

```
cd your_catkin_workspace/
source devel/setup.bash
roslaunch navigation start_sim.launch
```

Then, the launch file will launch Bezier trajectory generation, LQR controler modules.

Click '2D goal' on the toolbar to set the destination to robot.

## 3. Simulation demo

![img](https://github.com/Lkaho/Online-Motion-Planning-based-on-Bezier/blob/main/sim.gif)	

## 4. Real World Experiment

- ![img](https://github.com/Lkaho/Online-Motion-Planning-based-on-Bezier/blob/main/real_world_test1.gif)

- ![img](https://github.com/Lkaho/Online-Motion-Planning-based-on-Bezier/blob/main/real_world_test2.gif)

