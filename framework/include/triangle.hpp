#ifndef TRIANGLE_HPP
#define TRIANGLE_HPP

#include <iostream>
#include <vector>

#include "point.hpp"

/**
 * Basic class to describe a Triange according
 * to the OFF file format. The underlying
 * structure to this class is a simple vector
 * and the individual elements can be accessed
 * via the [] operator.
 */
class Triangle {
public:
  Triangle(const Point &v0, const Point &v1, const Point &v2)  {
    vertices.push_back(v0);
    vertices.push_back(v1);
    vertices.push_back(v2);
  }

  /**
   * This lets you add a vertex index to the
   * Face object. The element will be added to
   * the back of the vector, so if order matters
   * we might need to find another solution to
   * adding new vertices
   */
  void add(const Point &vertex) { vertices.push_back(vertex); }

  /**
   * Returns the number of vertices in the current
   * vector
   */
  int numVertices() { return vertices.size(); }

  /**
   * An overloaded operator to allow direct indexng
   * of face indices. Does no bounds-checking so that
   * might be useful later
   */
  Point &operator[](int idx) { return vertices[idx]; }

  /**
   * An overloaded operator to allow direct indexng
   * of face indices. Does no bounds-checking so that
   * might be useful later
   */
  const Point &operator[](int idx) const { return vertices[idx]; }

private:
  std::vector<Point> vertices;
};
#endif // TRIANGLES_HPP
