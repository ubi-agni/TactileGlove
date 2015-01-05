cmake_minimum_required (VERSION 2.6)

find_package(Curses QUIET)
if(CURSES_FOUND)
   add_definitions(-DHAVE_CURSES=1)
endif()
if(catkin_FOUND)
   add_definitions(-DHAVE_ROS=1)
endif()

include_directories(${CURSES_INCLUDE_PATH} ${Boost_INCLUDE_DIRS})

add_executable(console console.cpp)
target_link_libraries(console
   ${CURSES_NCURSES_LIBRARY}
   ${Boost_PROGRAM_OPTIONS_LIBRARY_RELEASE}
   ${roscpp_LIBRARIES})

install(TARGETS console RUNTIME DESTINATION bin)
