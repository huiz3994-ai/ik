cmake_minimum_required(VERSION 3.0.2)
project(rm75_sampling_gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  control_msgs
  moveit_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/sampling_gui.py
  scripts/joint_control_server.py
  scripts/sampling_web.py
  scripts/validate_web_simulation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/start_sampling_station.sh
  scripts/start_sampling_web.sh
  scripts/start_sampling_web_realtime.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES __init__.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  assets
  docs
  config
  launch
  ros
  state
  task
  ui
  gui
  widgets
  logger
  model
  control
  web/backend
  web/frontend/dist
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/monitor_transport.test)
  catkin_add_nosetests(test/test_field_station.py)
  catkin_add_nosetests(test/test_ros_interface.py)
  catkin_add_nosetests(test/test_state_machine.py)
  catkin_add_nosetests(test/test_task_manager.py)
  catkin_add_nosetests(test/test_joint_digital_twin.py)
  catkin_add_nosetests(test/test_joint_control_server.py)
endif()
