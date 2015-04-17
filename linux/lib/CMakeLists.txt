file(GLOB SOURCES *.h *.cpp)
add_library(${PROJECT_NAME} STATIC ${SOURCES})
