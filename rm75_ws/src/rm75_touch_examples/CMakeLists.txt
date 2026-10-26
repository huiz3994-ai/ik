cmake_minimum_required(VERSION 3.0.2)
project(rm75_touch_examples)

add_compile_options(-std=c++14 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs moveit_ros_planning_interface roscpp trajectory_msgs)

catkin_package(CATKIN_DEPENDS
  geometry_msgs moveit_ros_planning_interface roscpp trajectory_msgs)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(rm75_get_end_effector_pose examples/rm75_get_end_effector_pose.cpp)
target_link_libraries(rm75_get_end_effector_pose ${catkin_LIBRARIES})
add_executable(rm75_initialize_forward_pose examples/rm75_initialize_forward_pose.cpp)
target_link_libraries(rm75_initialize_forward_pose ${catkin_LIBRARIES})

install(TARGETS rm75_get_end_effector_pose rm75_initialize_forward_pose
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY reference DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
