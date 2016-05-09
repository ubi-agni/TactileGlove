file(GLOB SOURCES *.h *.cpp)
add_library(${PROJECT_NAME}_io STATIC ${SOURCES})
