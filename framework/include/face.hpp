#ifndef FACE_HPP
#define FACE_HPP

#include <iostream>
#include <vector>

#include "point.hpp"

/**
 * Basic class to describe a Face according
 * to the OBJ file format. The underlying
 * structure to this class is a simple vector
 * and the individual elements can be accessed
 * via the [] operator.
 */
class Face {
public:
  Face(const glm::uint &v0, const glm::uint &v1, const glm::uint &v2)  {
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
  void add(const glm::uint &vertex) { vertices.push_back(vertex); }

  /**
   * Returns the number of vertices in the current
   * vector
   */
  glm::uint numVertices() { return vertices.size(); }

  /**
   * An overloaded operator to allow direct indexng
   * of face indices. Does no bounds-checking so that
   * might be useful later
   */
  glm::uint &operator[](int idx) { return vertices[idx]; }

  /**
   * An overloaded operator to allow direct indexng
   * of face indices. Does no bounds-checking so that
   * might be useful later
   */
  const glm::uint &operator[](int idx) const { return vertices[idx]; }

  glm::vec3 normal;
  float area;

private:
  std::vector<glm::uint> vertices;
};
#endif // FACES_HPP
