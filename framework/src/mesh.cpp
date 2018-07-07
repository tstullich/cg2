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
  // add face ids to vertices and compute face normals
  for (glm::uint i = 0; i < faces.size(); ++i) {
    for (glm::uint j = 0; j < faces[i].numVertices(); ++j) {
      vertices[faces[i][j]].adjacentFaces.push_back(i);
    }
    if(faces[i].numVertices() == 3) {
      faces[i].normal = glm::cross(vertices[faces[i][1]].toVec3() - vertices[faces[i][0]].toVec3(),
          vertices[faces[i][2]].toVec3() - vertices[faces[i][0]].toVec3());
      faces[i].normal /= glm::length(faces[i].normal);
    }
  }
}

void Mesh::computeUnweightedNormals() {
  for (glm::uint i = 0; i < vertices.size(); ++i) {
    glm::vec3 v(0.0, 0.0, 0.0);
    for (glm::uint j = 0; j < vertices[i].adjacentFaces.size(); ++j) {
      v += faces[vertices[i].adjacentFaces[j]].normal;
    }
    v /= vertices[i].adjacentFaces.size();
    vertices[i].normal = v;
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
