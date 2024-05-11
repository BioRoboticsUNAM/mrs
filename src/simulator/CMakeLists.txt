cmake_minimum_required(VERSION 2.8.3)
project(simulator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  std_srvs
  sensor_msgs
##action_planner_msgs
  nav_msgs
  geometry_msgs
  tf
  message_generation
)


find_package(CGAL REQUIRED COMPONENTS Core)
set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "Don't override flags")
message(CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS})
include(${CGAL_USE_FILE})
message(CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS})

#find_package(Eigen3 REQUIRED)
#include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
file(GLOB MSG_FILES RELATIVE
  ${CMAKE_CURRENT_SOURCE_DIR}/msg
  ${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg
)
if( "${MSG_FILES}" STREQUAL "" )
  message("No messages in ${PROJECT_NAME}")
else()
 add_message_files(FILES
   ${MSG_FILES}
 )
endif()


## Generate services in the 'srv' folder
file(GLOB SRV_FILES RELATIVE
  ${CMAKE_CURRENT_SOURCE_DIR}/srv
  ${CMAKE_CURRENT_SOURCE_DIR}/srv/*.srv
)
if( "${SRV_FILES}" STREQUAL "" )
  message("No services in ${PROJECT_NAME}")
else()
  add_service_files(FILES
    ${SRV_FILES}
  )
endif()

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  ##action_planner_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS src
#  LIBRARIES simulator
#  CATKIN_DEPENDS roscpp rospy std_msgs clips
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Build the CLIPS library libclips.a
add_subdirectory(src/clips60)

## Build the CLIPS library bridge.a
add_subdirectory(src/bridge)

## Build the main node
add_subdirectory(src/rosclips)

## Build the clipscontrol node
add_subdirectory(src/clipscontrol)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/simulator.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/simulator_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")



## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_simulator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_library(simrep src/simrep/SimuladorRepresentation.cpp)
target_link_libraries(simrep ${catkin_LIBRARIES})
add_dependencies(simrep
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
link_libraries(simrep)

add_executable(motion_planner_node src/motion_planner/motion_planner_node.cpp)
target_link_libraries(motion_planner_node ${catkin_LIBRARIES})
add_dependencies(motion_planner_node simulator_generate_messages_cpp)

add_executable(base_node src/simulator_physics/base_node.cpp)
target_link_libraries(base_node ${catkin_LIBRARIES})
add_dependencies(base_node simulator_generate_messages_cpp)

add_executable(laser_node src/simulator_physics/laser_node.cpp)
target_link_libraries(laser_node ${catkin_LIBRARIES})
add_dependencies(laser_node simulator_generate_messages_cpp)

add_executable(light_server src/simulator_physics/light_server.cpp)
target_link_libraries(light_server ${catkin_LIBRARIES})
add_dependencies(light_server simulator_generate_messages_cpp)

add_executable(light_node src/simulator_physics/light_node.cpp)
target_link_libraries(light_node ${catkin_LIBRARIES})
add_dependencies(light_node simulator_generate_messages_cpp)

add_executable(move_turtlebot_node src/turtlebot/move_turtlebot_node.cpp)
target_link_libraries(move_turtlebot_node ${catkin_LIBRARIES})
add_dependencies(move_turtlebot_node simulator_generate_messages_cpp)

add_executable(simple_move_node src/turtlebot/simple_move_node.cpp)
target_link_libraries(simple_move_node ${catkin_LIBRARIES})
add_dependencies(simple_move_node simulator_generate_messages_cpp)

add_executable(map_rviz_node src/visualization/map_rviz_node.cpp)
target_link_libraries(map_rviz_node ${catkin_LIBRARIES})
add_dependencies(map_rviz_node simulator_generate_messages_cpp)

add_executable(objs_viz_node src/visualization/objs_viz_node.cpp)
target_link_libraries(objs_viz_node ${catkin_LIBRARIES})
add_dependencies(objs_viz_node simulator_generate_messages_cpp)

add_executable(find_obj_node src/simulator_physics/find_obj_node.cpp)
target_link_libraries(find_obj_node ${catkin_LIBRARIES})
add_dependencies(find_obj_node simulator_generate_messages_cpp)

add_executable(manipulator_node src/simulator_physics/manipulator_node.cpp)
target_link_libraries(manipulator_node ${catkin_LIBRARIES})
add_dependencies(manipulator_node simulator_generate_messages_cpp)

add_executable(action_planner_node src/action_planner/action_planner_node.cpp)
target_link_libraries(action_planner_node ${catkin_LIBRARIES})
add_dependencies(action_planner_node simulator_generate_messages_cpp)

add_executable(natural_language_node src/natural_language/natural_language_node.cpp)
target_link_libraries(natural_language_node ${catkin_LIBRARIES})
add_dependencies(natural_language_node simulator_generate_messages_cpp)

add_executable(speech_generation_node src/speech_generation/speech_generation_node.cpp)
target_link_libraries(speech_generation_node ${catkin_LIBRARIES})
add_dependencies(speech_generation_node simulator_generate_messages_cpp)


