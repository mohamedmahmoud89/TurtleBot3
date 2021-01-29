cmake_minimum_required(VERSION 2.8.3)
project(turtleBot3)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ros/common
  ros/hmi
  ros/opencrComm
  ros/odom
  ros/ldsd
  core/common
  core/odom
  core/lfx
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/robot_consumer.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(hmi_node ros/hmi/hmiMain.cpp)
add_executable(opencr_node ros/opencrComm/opencrCommMain.cpp)
add_executable(odom_node ros/odom/odomMain.cpp)
add_executable(lds_node ros/ldsd/ldsMain.cpp ros/ldsd/ldsDriver.cpp)
add_executable(lfx_node ros/lfx/lfxMain.cpp)
add_executable(maze_node ros/mazeRunner/mazeRunnerMain.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(
   hmi_node
   ${catkin_LIBRARIES}
 )
  target_link_libraries(
   opencr_node
   ${catkin_LIBRARIES}
 )
  target_link_libraries(
   odom_node
   ${catkin_LIBRARIES}
 )
 target_link_libraries(
   lds_node
   ${catkin_LIBRARIES}
 )
 target_link_libraries(
   lfx_node
   ${catkin_LIBRARIES}
 )
 target_link_libraries(
   maze_node
   ${catkin_LIBRARIES}
 )