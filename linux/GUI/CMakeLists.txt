cmake_minimum_required(VERSION 2.6)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
find_package(Qt54 REQUIRED COMPONENTS Core Gui Svg Xml)
find_package(Boost 1.38 REQUIRED COMPONENTS thread program_options regex)

set(CMAKE_AUTOMOC ON)

file(GLOB SOURCES "RandomInput.*" "SerialThread.*" "ROSInput.*" "*.h" "*.cpp")
file(GLOB ROS_SRC "ROSInput.*")
file(GLOB FORMS "*.ui")
qt_wrap_ui(FORM_SRC ${FORMS})
qt_add_resources(RESOURCES resources.qrc)

if(catkin_FOUND)
   add_definitions(-DHAVE_ROS=1)
else()
   list(REMOVE_ITEM SOURCES ${ROS_SRC})
endif()

add_definitions(${QT_DEFINITIONS})
include_directories(${CMAKE_CURRENT_BINARY_DIR}
                    ${QT_INCLUDES}
                    ${catkin_INCLUDE_DIRS}
                    ${tactile_filters_INCLUDE_DIRS})

list(APPEND CMAKE_INSTALL_RPATH ${tactile_filters_DIR}/../lib)

set(PROG_NAME GloveViz)
add_executable(${PROG_NAME} ${SOURCES} ${FORM_SRC} ${RESOURCES})
target_link_libraries(${PROG_NAME}
   ${PROJECT_NAME}
   ${QT_LIBRARIES}
   ${Boost_PROGRAM_OPTIONS_LIBRARY_RELEASE}
   ${Boost_REGEX_LIBRARY_RELEASE}
   ${roscpp_LIBRARIES}
   ${tactile_filters_LIBRARIES})

install(TARGETS ${PROG_NAME} RUNTIME DESTINATION bin)
if(catkin_FOUND)
  install(TARGETS ${PROG_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
