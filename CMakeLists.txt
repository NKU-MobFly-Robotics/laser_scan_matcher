cmake_minimum_required(VERSION 2.8.3)
project(laser_scan_matcher)

# List C++ dependencies on ros packages
set(ROS_CXX_DEPENDENCIES
  roscpp
  sensor_msgs
  tf
  geometry_msgs
  nav_msgs
)

# Find catkin and all required ROS components
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})

# Find csm project
find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)

# Set include directories
include_directories(include ${catkin_INCLUDE_DIRS} ${csm_INCLUDE_DIRS})
link_directories(${csm_LIBRARY_DIRS})

# Declare info that other packages need to import library generated here
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laser_scan_matcher
  CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
)

#Create node
add_executable(laser_scan_matcher_node src/laser_scan_matcher.cpp)
target_link_libraries(laser_scan_matcher_node ${catkin_LIBRARIES} ${csm_LIBRARIES})