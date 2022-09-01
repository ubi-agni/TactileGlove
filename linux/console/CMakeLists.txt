find_package(Curses QUIET)

if(CURSES_FOUND)
   add_definitions(-DHAVE_CURSES=1)
endif()

if(catkin_FOUND)
   include_directories(${catkin_INCLUDE_DIRS})
   add_definitions(-DHAVE_ROS=1)
endif()

include_directories(
   ../libio
   ${CURSES_INCLUDE_PATH}
   ${Boost_INCLUDE_DIRS}
   ${tactile_filters_INCLUDE_DIRS})

list(APPEND CMAKE_INSTALL_RPATH ${tactile_filters_DIR}/../lib)

set(PROG_NAME gloveConsole)
add_executable(${PROG_NAME} main.cpp)
target_link_libraries(${PROG_NAME}
   ${PROJECT_NAME}_io
   ${CURSES_NCURSES_LIBRARY}
   ${Boost_PROGRAM_OPTIONS_LIBRARY_RELEASE}
   ${roscpp_LIBRARIES}
   ${tactile_filters_LIBRARIES})

install(TARGETS ${PROG_NAME} RUNTIME DESTINATION bin)

if(catkin_FOUND)
   install(TARGETS ${PROG_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
