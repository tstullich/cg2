CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE_DIRECTORIES(include)

# Some extra compile flags. Could use this for linking too
SET(GCC_COVERAGE_COMPILE_FLAGS "-Wall -O3")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")

# Our source files. New files will need to be added for CMake to pick them up
SET(SOURCES src/main.cpp include/cg2_framework.hpp include/parser.hpp include/point.hpp include/face.hpp src/cg2_framework.cpp src/parser.cpp)

ADD_EXECUTABLE(main ${SOURCES})
