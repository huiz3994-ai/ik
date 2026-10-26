cmake_minimum_required(VERSION 3.0.2)
project(rm75_qr_task)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge geometry_msgs message_filters message_generation moveit_commander moveit_msgs
  rospy sensor_msgs std_msgs std_srvs tf2_geometry_msgs tf2_msgs tf2_ros
)

catkin_python_setup()
add_service_files(FILES TaskCommand.srv)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)
catkin_package(CATKIN_DEPENDS
  geometry_msgs message_runtime moveit_msgs rospy sensor_msgs std_msgs std_srvs tf2_msgs tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/marker_node.py
  scripts/five_target_server.py
  scripts/task.py
  scripts/task_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_pose_manager.py)
  catkin_add_nosetests(test/test_marker_manager.py)
  catkin_add_nosetests(test/test_state_machine.py)
  catkin_add_nosetests(test/test_storage.py)
  catkin_add_nosetests(test/test_five_targets.py)
endif()
