cmake_minimum_required(VERSION 3.0.2)
project(rm75_touch_teleop)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_core
  moveit_ros_planning
  rm_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs moveit_core moveit_ros_planning roscpp
                 rm_msgs sensor_msgs std_msgs std_srvs trajectory_msgs
  DEPENDS EIGEN3
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(rm75_touch_teleop_node src/rm75_touch_teleop_node.cpp)
target_link_libraries(rm75_touch_teleop_node ${catkin_LIBRARIES})

add_dependencies(rm75_touch_teleop_node ${catkin_EXPORTED_TARGETS})
add_executable(sampling_soil_sim_node src/sampling_soil_sim_node.cpp)
target_link_libraries(sampling_soil_sim_node ${catkin_LIBRARIES})
add_dependencies(sampling_soil_sim_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(hybrid_force_control_test test/hybrid_force_control_test.cpp)
  if(TARGET hybrid_force_control_test)
    target_link_libraries(hybrid_force_control_test ${catkin_LIBRARIES})
  endif()
  add_rostest(test/hybrid_force_control.test)
  add_rostest(test/haptic_contract.test)
  catkin_add_gtest(mapping_test test/mapping_test.cpp)
  if(TARGET mapping_test)
    target_link_libraries(mapping_test ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(haptic_renderer_test test/haptic_renderer_test.cpp)
  if(TARGET haptic_renderer_test)
    target_link_libraries(haptic_renderer_test ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(damped_differential_ik_test test/damped_differential_ik_test.cpp)
  if(TARGET damped_differential_ik_test)
    target_link_libraries(damped_differential_ik_test ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS rm75_touch_teleop_node sampling_soil_sim_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/check_touch_teleop.sh scripts/check_rm75_real_6f.sh
                 scripts/gzclient_as_desktop_user.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
