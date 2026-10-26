cmake_minimum_required(VERSION 3.0.2)
project(rm75_kinematics_optimization)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)
find_package(catkin REQUIRED COMPONENTS roscpp kdl_parser urdf)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio>=2.6)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rm75_kinematics
  CATKIN_DEPENDS roscpp kdl_parser urdf
  DEPENDS EIGEN3 orocos_kdl
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS} ${PINOCCHIO_INCLUDE_DIRS})
link_directories(${PINOCCHIO_LIBRARY_DIRS})
add_definitions(${PINOCCHIO_CFLAGS_OTHER})

set(RM75_6FB_XACRO ${CMAKE_CURRENT_SOURCE_DIR}/../rm_robot/rm_description/urdf/RM75_6FB/rm_75_6fb_description.urdf.xacro)
set(RM75_GENERATED_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
set(RM75_6FB_URDF ${RM75_GENERATED_DIR}/rm75_6fb.generated.urdf)
file(MAKE_DIRECTORY ${RM75_GENERATED_DIR})
execute_process(
  COMMAND ${CATKIN_ENV} xacro ${RM75_6FB_XACRO} link7_type:=Link7_6fb
  OUTPUT_FILE ${RM75_6FB_URDF}
  RESULT_VARIABLE XACRO_RESULT)
if(NOT XACRO_RESULT EQUAL 0)
  message(FATAL_ERROR "Failed to expand the RM75-6FB xacro")
endif()

add_library(rm75_kinematics
  src/kinematics_model.cpp
  src/adaptive_dls_ik.cpp
  src/nullspace_optimizer.cpp)
target_link_libraries(rm75_kinematics ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES})

add_executable(rm75_ik_benchmark src/ik_benchmark.cpp)
target_compile_definitions(rm75_ik_benchmark PRIVATE RM75_DEFAULT_URDF="${RM75_6FB_URDF}")
target_link_libraries(rm75_ik_benchmark rm75_kinematics ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_executable(rm75_fk_cli src/fk_cli.cpp)
target_link_libraries(rm75_fk_cli rm75_kinematics ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(rm75_kinematics_test test/kinematics_test.cpp)
  if(TARGET rm75_kinematics_test)
    target_compile_definitions(rm75_kinematics_test PRIVATE RM75_TEST_URDF="${RM75_6FB_URDF}")
    target_link_libraries(rm75_kinematics_test rm75_kinematics ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
  endif()
endif()

install(TARGETS rm75_kinematics rm75_ik_benchmark rm75_fk_cli
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/plot_benchmark.py scripts/plot_scenario_summary.py scripts/optimization_ik.py scripts/optimization_ik_benchmark.py scripts/summarize_by_scenario.py
  scripts/setup_casadi_venv.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES requirements-casadi.txt DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
