cmake_minimum_required(VERSION 2.6)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
find_package(Qt54 REQUIRED COMPONENTS Core Gui Svg)

set(CMAKE_AUTOMOC ON)

file(GLOB SOURCES "*.[hc]*")
file(GLOB ROS_SRC "ROSInput.[hc]*")
file(GLOB FORMS "*.ui")
qt_wrap_ui(FORM_SRC ${FORMS})
qt_add_resources(RESOURCES resources.qrc)

if(catkin_FOUND)
   add_definitions(-DHAVE_ROS=1)
else()
   list(REMOVE_ITEM SOURCES ${ROS_SRC})
endif()

add_definitions(${QT_DEFINITIONS})
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${QT_INCLUDES})

add_executable(GloveViz ${SOURCES} ${FORM_SRC} ${RESOURCES})

target_link_libraries(GloveViz
   ${QT_LIBRARIES}
   ${Boost_PROGRAM_OPTIONS_LIBRARY_RELEASE}
   ${roscpp_LIBRARIES})
