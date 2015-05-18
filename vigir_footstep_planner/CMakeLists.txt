cmake_minimum_required(VERSION 2.8.3)
project(vigir_footstep_planner)

if(CMAKE_VERSION VERSION_GREATER "2.8.12")
  cmake_policy(SET CMP0022 OLD)
endif()

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs nav_msgs visualization_msgs cv_bridge pcl_conversions vigir_footstep_planning_msgs vigir_footstep_planning_lib vigir_foot_pose_transformer vigir_terrain_classifier
             map_server angles tf)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(PCL REQUIRED)

#find_package(sbpl REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
link_directories(${SBPL_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vigir_footstep_planner
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs visualization_msgs cv_bridge pcl_conversions vigir_footstep_planning_msgs vigir_footstep_planning_lib vigir_foot_pose_transformer vigir_terrain_classifier
                 map_server angles tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${SBPL_INCLUDE_DIRS})

## Specify additional locations of header files
set(HEADERS
  include/${PROJECT_NAME}/environment_parameters.h
  include/${PROJECT_NAME}/footstep_planner_environment.h
  include/${PROJECT_NAME}/footstep_planner_node.h
  include/${PROJECT_NAME}/footstep_planner.h
  include/${PROJECT_NAME}/heuristics/dynamics_heuristic.h
  include/${PROJECT_NAME}/heuristics/euclidean_heuristic.h
  include/${PROJECT_NAME}/heuristics/heuristic.h
  include/${PROJECT_NAME}/heuristics/hot_map_heuristic.h
  include/${PROJECT_NAME}/heuristics/occupancy_grid_map_heuristic.h
  include/${PROJECT_NAME}/heuristics/path_cost_heuristic.h
  include/${PROJECT_NAME}/heuristics/step_cost_heuristic.h
  include/${PROJECT_NAME}/heuristics/travel_time_heuristic.h
  include/${PROJECT_NAME}/post_processor/post_processor.h
  include/${PROJECT_NAME}/post_processor/step_dynamics_post_process.h
  include/${PROJECT_NAME}/robot_model/dynamics_reachability.h
  include/${PROJECT_NAME}/robot_model/reachability_polygon.h
  include/${PROJECT_NAME}/robot_model/robot_model.h
  include/${PROJECT_NAME}/state_space/footstep.h
  include/${PROJECT_NAME}/state_space/planning_state.h
  include/${PROJECT_NAME}/state_space/state_space.h
  include/${PROJECT_NAME}/step_cost_estimators/boundary_step_cost_estimator.h
  include/${PROJECT_NAME}/step_cost_estimators/const_step_cost_estimator.h
  include/${PROJECT_NAME}/step_cost_estimators/dynamics_step_cost_estimator.h
  include/${PROJECT_NAME}/step_cost_estimators/euclidean_step_cost_estimator.h
  include/${PROJECT_NAME}/step_cost_estimators/ground_contact_step_cost_estimator.h
  include/${PROJECT_NAME}/step_cost_estimators/map_step_cost_estimator.h
  include/${PROJECT_NAME}/step_cost_estimators/step_cost_estimator.h
  include/${PROJECT_NAME}/step_cost_estimators/step_cost_key.h
  include/${PROJECT_NAME}/step_cost_estimators/travel_time_step_cost_estimator.h
  include/${PROJECT_NAME}/threading/expand_state_job.h
  include/${PROJECT_NAME}/world_model/foot_grid_map_model.h
  include/${PROJECT_NAME}/world_model/grid_map_model.h
  include/${PROJECT_NAME}/world_model/grid_map_2d.h
  include/${PROJECT_NAME}/world_model/terrain_model.h
  include/${PROJECT_NAME}/world_model/upper_body_grid_map_model.h
  include/${PROJECT_NAME}/world_model/world_model.h
)

set(SOURCES
  src/environment_parameters.cpp
  src/footstep_planner_environment.cpp
  src/footstep_planner_node.cpp
  src/footstep_planner.cpp
  src/heuristics/dynamics_heuristic.cpp
  src/heuristics/euclidean_heuristic.cpp
  src/heuristics/heuristic.cpp
  src/heuristics/hot_map_heuristic.cpp
  src/heuristics/occupancy_grid_map_heuristic.cpp
  src/heuristics/path_cost_heuristic.cpp
  src/heuristics/step_cost_heuristic.cpp
  src/heuristics/travel_time_heuristic.cpp
  src/post_processor/post_processor.cpp
  src/post_processor/step_dynamics_post_process.cpp
  src/robot_model/dynamics_reachability.cpp
  src/robot_model/reachability_polygon.cpp
  src/robot_model/robot_model.cpp
  src/state_space/footstep.cpp
  src/state_space/planning_state.cpp
  src/state_space/state_space.cpp
  src/step_cost_estimators/boundary_step_cost_estimator.cpp
  src/step_cost_estimators/const_step_cost_estimator.cpp
  src/step_cost_estimators/dynamics_step_cost_estimator.cpp
  src/step_cost_estimators/euclidean_step_cost_estimator.cpp
  src/step_cost_estimators/ground_contact_step_cost_estimator.cpp
  src/step_cost_estimators/map_step_cost_estimator.cpp
  src/step_cost_estimators/step_cost_estimator.cpp
  src/step_cost_estimators/step_cost_key.cpp
  src/step_cost_estimators/travel_time_step_cost_estimator.cpp
  src/threading/expand_state_job.cpp
  src/world_model/foot_grid_map_model.cpp
  src/world_model/grid_map_2d.cpp
  src/world_model/grid_map_model.cpp
  src/world_model/terrain_model.cpp
  src/world_model/upper_body_grid_map_model.cpp
  src/world_model/world_model.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

## Declare a cpp executable
add_executable(footstep_planner_node src/footstep_planner_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} ${boost_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${SBPL_LIBRARIES})
target_link_libraries(footstep_planner_node ${PROJECT_NAME})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} footstep_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vigir_footstep_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
