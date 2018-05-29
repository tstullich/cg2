CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

FIND_PATH(GLM_INCLUDE_DIR glm/glm.hpp HINTS /usr/include /usr/local/include /opt/local/include)
INCLUDE_DIRECTORIES(${GLM_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})

SET(CMAKE_CXX_STANDARD 14)
SET(GCC_COVERAGE_COMPILE_FLAGS "-Wall -03 -ldl")

INCLUDE_DIRECTORIES(${CG2_SOURCE_DIR}/framework/include)
ADD_LIBRARY(framework point.cpp kdtree.cpp parser.cpp)
