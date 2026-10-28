cmake_minimum_required(VERSION 3.0.2)
project(rm75_motion_planning)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)
find_package(catkin REQUIRED COMPONENTS roscpp rm75_kinematics_optimization
  kdl_parser moveit_core moveit_ros_planning moveit_msgs geometry_msgs
  shape_msgs srdfdom sensor_msgs std_msgs std_srvs trajectory_msgs rm_msgs)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(PythonInterp 3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rm75_motion_planning
  CATKIN_DEPENDS roscpp rm75_kinematics_optimization kdl_parser moveit_core
    moveit_ros_planning moveit_msgs geometry_msgs shape_msgs srdfdom
    sensor_msgs std_msgs std_srvs trajectory_msgs rm_msgs
  DEPENDS EIGEN3 orocos_kdl)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS})

# The linked devel layout gives each package a private prefix. Resolve the
# generated URDF relative to the dependency's exported CMake directory rather
# than relative to this package's CATKIN_DEVEL_PREFIX.
get_filename_component(RM75_KINEMATICS_SHARE
  ${rm75_kinematics_optimization_DIR}/.. ABSOLUTE)
set(RM75_6FB_URDF ${RM75_KINEMATICS_SHARE}/config/rm75_6fb.generated.urdf)

add_library(rm75_motion_planning
  src/cartesian_jog.cpp
  src/se3_interpolator.cpp
  src/cartesian_path_generator.cpp
  src/continuous_ik_solver.cpp
  src/persistent_ipopt_fallback.cpp
  src/trajectory_parameterizer.cpp
  src/trajectory_validator.cpp
  src/moveit_collision_checker.cpp
  src/real_execution_metrics.cpp
  src/runtime_guard.cpp
  src/command_source_manager.cpp)
target_link_libraries(rm75_motion_planning ${catkin_LIBRARIES})

add_executable(rm75_motion_benchmark benchmark/motion_benchmark.cpp)
target_compile_definitions(rm75_motion_benchmark PRIVATE RM75_DEFAULT_URDF="${RM75_6FB_URDF}")
target_link_libraries(rm75_motion_benchmark rm75_motion_planning
  ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(rm75_fallback_smoke scripts/fallback_smoke.cpp)
target_compile_definitions(rm75_fallback_smoke PRIVATE RM75_DEFAULT_URDF="${RM75_6FB_URDF}")
target_link_libraries(rm75_fallback_smoke rm75_motion_planning ${catkin_LIBRARIES})

add_executable(rm75_live_state_audit scripts/live_state_audit.cpp)
target_compile_definitions(rm75_live_state_audit PRIVATE
  RM75_DEFAULT_URDF="${RM75_6FB_URDF}"
  RM75_DEFAULT_SRDF="${CMAKE_CURRENT_SOURCE_DIR}/../rm_robot/rm_moveit_config/rm_75_moveit_config/config/rm_75.srdf")
target_link_libraries(rm75_live_state_audit rm75_motion_planning ${catkin_LIBRARIES})

add_executable(rm75_low_speed_commissioning scripts/low_speed_commissioning.cpp)
target_compile_definitions(rm75_low_speed_commissioning PRIVATE
  RM75_DEFAULT_URDF="${RM75_6FB_URDF}"
  RM75_DEFAULT_SRDF="${CMAKE_CURRENT_SOURCE_DIR}/../rm_robot/rm_moveit_config/rm_75_moveit_config/config/rm_75.srdf")
target_link_libraries(rm75_low_speed_commissioning rm75_motion_planning
  ${catkin_LIBRARIES})

add_executable(rm75_cartesian_commissioning scripts/cartesian_commissioning.cpp)
target_compile_definitions(rm75_cartesian_commissioning PRIVATE
  RM75_DEFAULT_URDF="${RM75_6FB_URDF}"
  RM75_DEFAULT_SRDF="${CMAKE_CURRENT_SOURCE_DIR}/../rm_robot/rm_moveit_config/rm_75_moveit_config/config/rm_75.srdf")
target_link_libraries(rm75_cartesian_commissioning rm75_motion_planning
  ${catkin_LIBRARIES})

add_executable(rm75_cartesian_jog_server scripts/cartesian_jog_server.cpp)
target_compile_definitions(rm75_cartesian_jog_server PRIVATE
  RM75_DEFAULT_URDF="${RM75_6FB_URDF}"
  RM75_DEFAULT_SRDF="${CMAKE_CURRENT_SOURCE_DIR}/../rm_robot/rm_moveit_config/rm_75_moveit_config/config/rm_75.srdf")
target_link_libraries(rm75_cartesian_jog_server rm75_motion_planning
  ${catkin_LIBRARIES})

add_executable(rm75_gazebo_shadow_execution scripts/gazebo_shadow_execution.cpp)
target_compile_definitions(rm75_gazebo_shadow_execution PRIVATE
  RM75_DEFAULT_URDF="${RM75_6FB_URDF}"
  RM75_DEFAULT_SRDF="${CMAKE_CURRENT_SOURCE_DIR}/../rm_robot/rm_moveit_config/rm_75_moveit_config/config/rm_75.srdf")
target_link_libraries(rm75_gazebo_shadow_execution rm75_motion_planning
  ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(rm75_motion_planning_test test/motion_planning_test.cpp)
  if(TARGET rm75_motion_planning_test)
    target_compile_definitions(rm75_motion_planning_test PRIVATE
      RM75_TEST_URDF="${RM75_6FB_URDF}"
      RM75_TEST_SRDF="${CMAKE_CURRENT_SOURCE_DIR}/../rm_robot/rm_moveit_config/rm_75_moveit_config/config/rm_75.srdf"
      RM75_FAKE_IPOPT_SERVER="${CMAKE_CURRENT_SOURCE_DIR}/test/fake_ipopt_server.py")
    target_link_libraries(rm75_motion_planning_test rm75_motion_planning ${catkin_LIBRARIES})
  endif()
  add_test(NAME rm75_real_benchmark_report_test
    COMMAND ${PYTHON_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/real_benchmark_report_test.py)
  add_test(NAME rm75_stage4_characterization_test
    COMMAND ${PYTHON_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/stage4_characterization_test.py)
endif()

install(TARGETS rm75_motion_planning rm75_motion_benchmark rm75_fallback_smoke
  rm75_live_state_audit rm75_low_speed_commissioning
  rm75_cartesian_commissioning
  rm75_cartesian_jog_server
  rm75_gazebo_shadow_execution
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config docs launch benchmark/real_execution
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/run_benchmark.sh scripts/render_report.py
  scripts/ipopt_motion_benchmark.py scripts/ipopt_fallback_server.py
  scripts/setup_ipopt_fallback.sh
  benchmark/real_execution/analyze_real_benchmark.py
  benchmark/real_execution/analyze_characterization.py
  benchmark/real_execution/record_real_trial.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES requirements-ipopt.txt DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
