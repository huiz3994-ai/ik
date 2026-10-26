cmake_minimum_required(VERSION 3.0.2)
project(geomagic_myhaptics)

find_package(catkin REQUIRED COMPONENTS
  geomagic_control
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2_ros   
  tf2_eigen
  tf2_geometry_msgs 
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES geomagic_myhaptics
#  CATKIN_DEPENDS geomagic_control geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  # 这里取消注释
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pubforce src/pubforce.cpp)
add_dependencies(pubforce ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pubforce
  ${catkin_LIBRARIES}
)
add_executable(pubforce_single src/pubforce_single.cpp)
add_dependencies(pubforce_single ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pubforce_single
  ${catkin_LIBRARIES}
)

add_executable(pubforce_touch1 src/pubforce_touch1.cpp)
add_dependencies(pubforce_touch1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pubforce_touch1
  ${catkin_LIBRARIES}
)

add_executable(pubforce_touch2 src/pubforce_touch2.cpp)
add_dependencies(pubforce_touch2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pubforce_touch2
  ${catkin_LIBRARIES}
)