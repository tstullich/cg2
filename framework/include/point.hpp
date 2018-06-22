#ifndef POINT_HPP
#define POINT_HPP

#include <algorithm>
#include <cmath>
#include <limits>

#include <glm/glm.hpp>

// Forward declaration so we can use Node class
struct Node;

/**
 * Quick mockup of a Point according to the OFF file
 * format. All of the members are public for now for
 * easy data access, but we can refine this class later on.
 */
class Point {
public:
  Point(float x, float y, float z);
  Point(glm::vec3 v);

  float fetchPointValue(int axis);
  float distPoint(const Point &b) const;
  float distNode(const Node &b) const;
  glm::vec3 toVec3();

  float x;
  float y;
  float z;
  float f = 0;
  float dist;
  glm::vec3 normal;
};
#endif // POINT_HPP
