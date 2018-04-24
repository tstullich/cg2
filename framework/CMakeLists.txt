CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
INCLUDE_DIRECTORIES(include)
SET(SOURCES src/main.cpp include/cg2_framework.h include/parser.hpp include/point.hpp include/face.hpp src/cg2_framework.cpp src/parser.cpp)
ADD_EXECUTABLE(main ${SOURCES})
