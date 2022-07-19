cmake_minimum_required(VERSION 3.0.2)
project(btraj_msgs)

find_package(catkin_simple REQUIRED )
catkin_simple(ALL_DEPS_REQUIRED)

# add_message_files(
#    FILES
#    trajectory.msg
#  )



# ## Generate added messages and services with any dependencies listed here
#  generate_messages(
#    DEPENDENCIES
#    geometry_msgs  std_msgs
#  )

cs_install()
cs_export()
