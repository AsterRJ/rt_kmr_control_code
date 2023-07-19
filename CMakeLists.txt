cmake_minimum_required(VERSION 3.0.2)
project(kmr_iiwa_carlos)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
  visualization_msgs
  message_generation
  tf
  urdf
  kdl_parser
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kmr_iiwa
  #CATKIN_DEPENDS roscpp
)
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)
include_directories (/usr/local/include/eigen3)

add_library(kmr_iiwa src/kmr_iiwa/core/kmr_iiwa.cpp)
target_link_libraries(kmr_iiwa  ${catkin_LIBRARIES})

add_executable(kmr_iiwa_diff_ik_server src/kmr_iiwa/redundancy_server/kmr_iiwa_diff_ik_server.cpp)
target_link_libraries(kmr_iiwa_diff_ik_server ${catkin_LIBRARIES} kmr_iiwa)



