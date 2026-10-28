cmake_minimum_required(VERSION 3.0.2)
project(rm75_vertical_press)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_core
  moveit_ros_planning
  rm75_touch_teleop
  rm_msgs
  roscpp
  sensor_msgs
  std_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs moveit_core moveit_ros_planning
                 rm75_touch_teleop rm_msgs roscpp sensor_msgs std_msgs
  DEPENDS EIGEN3
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(rm75_vertical_press_node src/rm75_vertical_press_node.cpp)
target_link_libraries(rm75_vertical_press_node ${catkin_LIBRARIES})
add_dependencies(rm75_vertical_press_node ${catkin_EXPORTED_TARGETS})

add_executable(rm75_vertical_ik_probe src/rm75_vertical_ik_probe.cpp)
target_link_libraries(rm75_vertical_ik_probe ${catkin_LIBRARIES})
add_dependencies(rm75_vertical_ik_probe ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(vertical_orientation_test test/vertical_orientation_test.cpp)
  if(TARGET vertical_orientation_test)
    target_link_libraries(vertical_orientation_test ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(tool5_feed_test test/tool5_feed_test.cpp)
  if(TARGET tool5_feed_test)
    target_link_libraries(tool5_feed_test ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS rm75_vertical_press_node rm75_vertical_ik_probe
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
