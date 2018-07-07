#ifndef MESH_HPP
#define MESH_HPP

#include <iostream>
#include <vector>

#include "point.hpp"
#include "face.hpp"

class Mesh {
  public:
    Mesh(std::vector<Point> vertices, std::vector<Face> faces);

    void computeUnweightedNormals();

    glm::vec3 getCenter();
    float getBoundingRadius();

    std::vector<Point> getVertices();
    std::vector<Face> getFaces();
  private:
    glm::vec3 center = glm::vec3(0.0, 0.0, 0.0);
    float boundingRadius = 0.0;

    std::vector<Point> vertices;
    std::vector<Face> faces;
};

#endif // MESH_HPP
