#ifndef POINT_HPP
#define POINT_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <glm/glm.hpp>

enum pointType {originalPoint, positivePoint, negativePoint};

// Forward declaration so we can use Node class
struct Node;

/**
 * Quick mockup of a Point according to the OFF file
 * format. All of the members are public for now for
 * easy data access, but we can refine this class later on.
 */
class Point {
public:
  Point() : x(0.0), y(0.0), z(0.0), dist(0.0), normal(glm::vec3(0.0, 0.0, 0.0)) {}
  Point(float x, float y, float z);
  Point(glm::vec3 v);

  float fetchPointValue(int axis);
  float distPoint(const Point &b) const;
  float distNode(const Node &b) const;
  glm::vec3 toVec3() const;
  void addVertex(glm::uint id);

  float x;
  float y;
  float z;
  float f = 0;
  pointType type = originalPoint;
  std::shared_ptr<Point> positivePoint;
  std::shared_ptr<Point> negativePoint;

  float dist;
  glm::vec3 normal;
  glm::vec3 weightedNormal;

  std::vector<glm::uint> adjacentFaces;
  std::vector<glm::uint> adjacentVertices;
};
#endif // POINT_HPP
