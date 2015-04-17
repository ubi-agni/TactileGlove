cmake_minimum_required (VERSION 2.6)
project (tactile_glove)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin QUIET COMPONENTS roscpp std_msgs)
find_package(Boost 1.38 REQUIRED COMPONENTS thread program_options)

add_subdirectory (lib)
add_subdirectory (console)
add_subdirectory (GUI)

if(catkin_FOUND)
  include_directories(${catkin_INCLUDE_DIRS})
else()
  message(WARNING "ROS catkin not found, building without ROS support")
endif()
