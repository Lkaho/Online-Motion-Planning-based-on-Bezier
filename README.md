# A Robust and Efficent Navigation Framework for  Indoor Wheel-based Robot

In this project, we designed a robust and efficent navigation framework for indoor mobile robot in unknown environments. For the motion planning part, based on the onboard state estimation and environment perception, we implemented A* for front-end path searching and generated back-end corridor-based trajectory by using Bezier curves. For the control part, we implemented path tracking controller by using LQR and MPC algorithm respectively.

## 1. Main contribution

- We implement A-star method for front-end path searching.(see navigation/src/astar.cpp)
- We implement corridor generator for extracting safe region from occupancy map for mobile robot. (see navigation/src/safe_corridor_generator.cpp)
- We ues OSQP_Interface to solve QP problem to generate Bezier curve. Thanks to the Hodograph property of Bezier curves,, we are able to bound the position within safe regions and limit the higher order dynamics of the trajectory within feasible regions.(see navigation/src/trajectory_generator.cpp)
- We implement LQR path tracking controller to track the path.(see controller/src)

## 2. Simulation demo

![img](https://github.com/Lkaho/Online-Motion-Planning-based-on-Bezier/blob/main/sim.gif)	

## 3. Real World test

- ![real1](/home/kaho/Workspaces/navigation_ws/src/real1.GIF)

- ![real2](/home/kaho/Workspaces/navigation_ws/src/real2.GIF)

