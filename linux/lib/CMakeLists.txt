set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES
  ColorMap.cpp ColorMap.h
  GloveWidget.cpp GloveWidget.h
  TaxelMapping.cpp TaxelMapping.h
  TaxelMappingTools.cpp TaxelMappingTools.h
)
qt_add_resources(RESOURCES resources.qrc)

add_definitions(${QT_DEFINITIONS})

include_directories(${QT_INCLUDES}
                    ${tactile_filters_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${RESOURCES})
target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)
set_target_properties(${PROJECT_NAME} PROPERTIES
  PUBLIC_HEADER "ColorMap.h;GloveWidget.h;TaxelMapping.h;TaxelMappingTools.h"
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}
)

## cmake package config
# Add all targets to the build-tree export set
export(TARGETS ${PROJECT_NAME}
  FILE "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")

configure_file(${PROJECT_NAME}Config.cmake.in
               ${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake
               @ONLY)

configure_file(${PROJECT_NAME}ConfigVersion.cmake.in
               ${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
               @ONLY)
# install the *Config.cmake and *ConfigVersion.cmake
install(FILES
  ${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION share/${PROJECT_NAME}/cmake)

# install the export set for use with the install-tree
install(EXPORT ${PROJECT_NAME}Targets
  DESTINATION share/${PROJECT_NAME}/cmake)

## pkgconfig file
configure_file(${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc
        DESTINATION lib/pkgconfig)
