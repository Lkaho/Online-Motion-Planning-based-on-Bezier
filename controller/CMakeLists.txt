cmake_minimum_required(VERSION 3.0.2)
project(controller)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})


include_directories(${Eigen3_INCLUDE_DIRS})


cs_add_executable(controller
src/control.cpp
src/lqr_control.cpp)

target_link_libraries(controller
${Eigen3_INCLUDE_DIRS})

cs_install()
cs_export()