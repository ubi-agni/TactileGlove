cmake_minimum_required (VERSION 2.6)

find_package(Curses)
include_directories(${CURSES_INCLUDE_PATH} ${Boost_INCLUDE_DIRS})

add_executable(console console.cpp)
target_link_libraries(console ${CURSES_NCURSES_LIBRARY})