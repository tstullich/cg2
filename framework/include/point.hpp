#ifndef POINT_HPP
#define POINT_HPP

#include <algorithm>
#include <cmath>
#include <limits>

// Forward declaration so we can use Node class
class Node;

/**
 * Quick mockup of a Point according to the OFF file
 * format. All of the members are public for now for
 * easy data access, but we can refine this class later on.
 */
class Point {
public:
  Point(float x, float y, float z) : x(x), y(y), z(z) {}

  float fetchPointValue(int axis);
  float distPoint(const Point &b) const;
  float distNode(const Node &b) const;

  float x;
  float y;
  float z;
  float dist;
};
#endif // POINT_HPP
