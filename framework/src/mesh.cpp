#include "mesh.hpp"

Mesh::Mesh(std::vector<Point> _vertices, std::vector<Face> _faces) :
    vertices(std::move(_vertices)), faces(std::move(_faces)) {
  assert(vertices.size() > 0);
  // calculate center of vertex cloud
  for (glm::uint i = 0; i < vertices.size(); ++i) {
    center += vertices[i].toVec3();
  }
  // determine lagest distance from center to vertex
  center /= vertices.size();
  for (glm::uint i = 0; i < vertices.size(); ++i) {
    float tmp = glm::length(vertices[i].toVec3() - center);
    boundingRadius = (tmp > boundingRadius)? tmp : boundingRadius;
  }
}

glm::vec3 Mesh::getCenter() {
  return center;
}

float Mesh::getBoundingRadius() {
  return boundingRadius;
}

std::vector<Point> Mesh::getVertices() {
  return vertices;
}

std::vector<Face> Mesh::getFaces() {
  return faces;
}
