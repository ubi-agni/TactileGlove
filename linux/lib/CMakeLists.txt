set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES
  ColorMap.cpp ColorMap.h
  GloveWidget.cpp GloveWidget.h
  TaxelMapping.cpp TaxelMapping.h
  resources.qrc)

add_definitions(${QT_DEFINITIONS})

include_directories(${QT_INCLUDES}
                    ${tactile_filters_INCLUDE_DIRS})

add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION bin)
if(catkin_FOUND)
  install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endif()
