cmake_minimum_required(VERSION 3.0.2)
project(geomagic_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs message_generation roscpp rosconsole sensor_msgs std_msgs tf urdf)

add_message_files(FILES DeviceButtonEvent.msg DeviceFeedback.msg)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(CATKIN_DEPENDS
  geometry_msgs message_runtime roscpp rosconsole sensor_msgs std_msgs tf urdf)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(device_node src/device_node.cpp)

add_dependencies(device_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(device_node HD HL HDU rt ncurses ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/lockDoF.py scripts/workspace_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS device_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY cfg launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

