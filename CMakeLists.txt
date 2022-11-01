cmake_minimum_required(VERSION 3.0.2)
project(stereo_vis_odom)

set(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  rospy  
  roscpp
  # libraries
  cv_bridge
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

find_package(OpenCV)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include 
  DEPENDS OpenCV

  CATKIN_DEPENDS
  cv_bridge 
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs 
)

# include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_libraries(
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# src
add_executable(${PROJECT_NAME}_KITTI KITTI/src/KITTI.cpp)
target_link_libraries(${PROJECT_NAME}_KITTI ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_StereoSystem StereoSystem/src/StereoSystem.cpp)
target_link_libraries(${PROJECT_NAME}_StereoSystem ${catkin_LIBRARIES})