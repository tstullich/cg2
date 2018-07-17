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
    void computeUniformLaplacian(glm::uint numEigenVectors);
    void resetUniformLaplace();
    void computeExplicitCotan(double stepSize, glm::uint numEigenVectors);
    void computeImplicitCotan(double stepSize, glm::uint numEigenVectors);
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
    float computeFaceArea(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3);
    /**
     * calculate area weighted face normal vector
     */
    glm::vec3 computeAreaNormal(Face face);

    /**
     * takes vertex indices of edge and computes sum of cot-values
     * at the remaining vertices
     */
    float computeCotSum(glm::uint i, glm::uint j);

    /**
     * Compute the Laplacian matrix L for cotan and uniform
     */
    SparseMatrix<double> computeCotanLMatrix(glm::uint n);
    SparseMatrix<double> computeUniformLMatrix(glm::uint n);

    glm::vec3 center = glm::vec3(0.0, 0.0, 0.0);
    float boundingRadius = 0.0;

    std::vector<Point> vertices;
    std::vector<Face> faces;
};

#endif // MESH_HPP
