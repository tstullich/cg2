#ifndef MESH_HPP
#define MESH_HPP

#include <iostream>
#include <vector>
#include <cmath>

#include "point.hpp"
#include "face.hpp"

#include <Eigen/Core>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <Spectra/SymEigsSolver.h>
#include <Spectra/MatOp/SparseSymMatProd.h>

using namespace Eigen;
using namespace Spectra;

class Mesh {
  public:
    Mesh(std::vector<Point> vertices, std::vector<Face> faces);

    void computeUnweightedNormals();
    void computeWeightedNormals();
    void computeUniformLaplacian(uint numEigenVectors);
    void computeExplicitCotan(double stepSize, uint numEigenVectors);
    void computeImplicitCotan(double stepSize, uint numEigenVectors);
    void resetCotanLaplace();

    float getSurroundingArea(Point &P);

    glm::vec3 getCenter();
    float getBoundingRadius();

    std::vector<Point> getVertices();
    std::vector<Face> getFaces();

    std::vector<Point> verticesUniformLaplacian;
    std::vector<Point> verticesExplicitLaplace;
    std::vector<Point> verticesImplicitLaplace;
  private:
    /**
     * calculate area weighted face normal vector
     */
    glm::vec3 computeAreaNormal(Face face);

    /**
     * Compute the Laplacian matrix L
     */
    SparseMatrix<double> computeLMatrix(uint n);

    glm::vec3 center = glm::vec3(0.0, 0.0, 0.0);
    float boundingRadius = 0.0;

    std::vector<Point> vertices;
    std::vector<Face> faces;
};

#endif // MESH_HPP
