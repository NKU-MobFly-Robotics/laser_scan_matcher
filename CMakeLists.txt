cmake_minimum_required(VERSION 2.8.3)
project(laser_scan_matcher)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  geometry_msgs
  nav_msgs
)

## Find csm project
find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)

## Set include directories
include_directories(include ${catkin_INCLUDE_DIRS} ${csm_INCLUDE_DIRS})
link_directories(${csm_LIBRARY_DIRS})

## Declare info that other packages need to import library generated here
catkin_package()

## Create node
add_executable(${PROJECT_NAME}_node src/laser_scan_matcher.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${csm_LIBRARIES})
