set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTORCC ON) # only works with cmake 3.x
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11)

set(SOURCES
  ColorMap.cpp ColorMap.h
  GloveWidget.cpp GloveWidget.h
  TaxelMapping.cpp TaxelMapping.h
  TaxelMappingTools.cpp TaxelMappingTools.h
)
# fallback for cmake < 3.x
qt_add_resources(RESOURCES resources.qrc)

add_definitions(${QT_DEFINITIONS})

include_directories(${QT_INCLUDES}
                    ${tactile_filters_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${RESOURCES})
target_link_libraries(${PROJECT_NAME}
  PRIVATE ${QT_LIBRARIES}
  PRIVATE ${Boost_PROGRAM_OPTIONS_LIBRARY}
)
# specify which headers to install with this target
set_target_properties(${PROJECT_NAME} PROPERTIES
  PUBLIC_HEADER "ColorMap.h;GloveWidget.h;TaxelMapping.h;TaxelMappingTools.h"
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  # specify where libs and binaries should go to
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  # specify where to install our headers
  PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}
  # specify where includes were installed (for package export)
  INCLUDES DESTINATION include
)

## cmake package config
include(CMakePackageConfigHelpers)
configure_file("${PROJECT_NAME}Config.cmake.in"
               "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
               @ONLY)

write_basic_package_version_file(
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION "0.1.0"
  COMPATIBILITY AnyNewerVersion
  )

# install the export set for use in the install tree
install(EXPORT ${PROJECT_NAME}Targets
  DESTINATION share/${PROJECT_NAME}/cmake)

#catkin_lint: ignore_once missing_file
# install the *Config.cmake and *ConfigVersion.cmake
install(FILES
  ${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION share/${PROJECT_NAME}/cmake)

## pkgconfig file
configure_file(${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc
        DESTINATION lib/pkgconfig)
