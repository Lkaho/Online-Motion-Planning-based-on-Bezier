 cmake_minimum_required(VERSION 3.0.2)
 project(navigation)

 ## Compile as C++11, supported in ROS Kinetic and newer
 # add_compile_options(-std=c++11)
 find_package(catkin_simple REQUIRED)
 catkin_simple(ALL_DEPS_REQUIRED)

 set(CMAKE_CXX_STANDARD 17)
 find_package(Eigen3 REQUIRED)
 find_package(osqp REQUIRED)
 find_package(PCL REQUIRED)
 find_package(tf2_ros)
 # set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
 # set(OSQP_INCLUDE_DIRS ${OSQP_INCLUDE_DIR})


 message(STATUS ${OSQP_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include/navigation)
 include_directories(
         include
         ${Eigen3_INCLUDE_DIRS}
         ${OSQP_INCLUDE_DIRS}
         ${PCL_INCLUDE_DIRS})


 cs_add_library(${PROJECT_NAME}
         src/astar.cpp
         src/safe_corridor_generator.cpp
         src/bezier.cpp
         src/trajectory_generator.cpp
         src/backward.cpp)
 target_link_libraries(${PROJECT_NAME}
         ${PCL_LIBRARIES}
         osqp::osqp
         )

 cs_add_executable(btraj_node src/btraj_node.cpp)
 target_link_libraries(btraj_node
         ${PROJECT_NAME}
         )


 cs_add_executable(perception_node src/perception_node.cpp)
 target_link_libraries(perception_node
         ${PCL_LIBRARIES}
         ${Eigen3_INCLUDE_DIRS})

 cs_add_executable(joy_ctrl src/joy_ctrl.cpp)

 cs_install()
 cs_export()