cmake_minimum_required (VERSION 2.8.12)
project (tactile_glove)

set(CMAKE_CXX_STANDARD 14)

#catkin_lint: ignore_once missing_required
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin QUIET COMPONENTS roscpp std_msgs sensor_msgs tactile_msgs)
find_package(Boost 1.38 REQUIRED COMPONENTS thread program_options regex)
find_package(tactile_filters REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
find_package(Qt54 COMPONENTS Core Gui Svg Xml SerialPort)

if(catkin_FOUND)
	# declare this as a catkin package, allows to rosrun our applications
	# installs package.xml, *Config.cmake, and *.pc files
	# defines catkin_destinations()
	catkin_package(
		CATKIN_DEPENDS roscpp std_msgs sensor_msgs tactile_msgs
		SKIP_CMAKE_CONFIG_GENERATION
		SKIP_PKG_CONFIG_GENERATION
	)
else()
  message(WARNING "ROS catkin not found, building without ROS support")
endif()

# add sub libraries *after* calling catking_package()
add_subdirectory (libio)
add_subdirectory (console)
if(QT_FOUND)
	add_subdirectory (lib)
	add_subdirectory (GUI)
endif()

if(catkin_FOUND)
  install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  catkin_install_python(
    PROGRAMS
    scripts/tactile_to_js.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()
