cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# find Eigen library and headers
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf2
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pure_pursuit
#  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(purepursuit_node src/purepursuit.cpp)
target_link_libraries(purepursuit_node ${catkin_LIBRARIES})

add_executable(online_trajectory_publish_node src/online_trajectory_publish.cpp)
target_link_libraries(online_trajectory_publish_node ${catkin_LIBRARIES})