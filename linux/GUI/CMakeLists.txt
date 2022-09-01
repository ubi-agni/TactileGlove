set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES
  InputInterface.h
  main.cpp
  MainWindow.cpp MainWindow.h
  MappingDialog.cpp MappingDialog.h
  FileExistsDialog.cpp FileExistsDialog.h
  RandomInput.cpp RandomInput.h
  QSerialInput.cpp QSerialInput.h
  TaxelSelector.cpp TaxelSelector.h)
qt_wrap_ui(FORM_SRC MainWindow.ui MappingDialog.ui FileExistsDialog.ui)

if(catkin_FOUND)
  add_definitions(-DHAVE_ROS=1)
  list(APPEND SOURCES ROSInput.cpp ROSInput.h)
endif()

add_definitions(${QT_DEFINITIONS})
include_directories(../lib
  ${QT_INCLUDES}
  ${catkin_INCLUDE_DIRS}
  ${tactile_filters_INCLUDE_DIRS})

list(APPEND CMAKE_INSTALL_RPATH ${tactile_filters_DIR}/../lib)

set(PROG_NAME GloveViz)
add_executable(${PROG_NAME} ${SOURCES} ${FORM_SRC})
target_link_libraries(${PROG_NAME}
  ${PROJECT_NAME}_io ${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${Boost_PROGRAM_OPTIONS_LIBRARY_RELEASE}
  ${Boost_REGEX_LIBRARY_RELEASE}
  ${roscpp_LIBRARIES}
  ${tactile_filters_LIBRARIES})

install(TARGETS ${PROG_NAME} RUNTIME DESTINATION bin)

if(catkin_FOUND)
  install(TARGETS ${PROG_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
