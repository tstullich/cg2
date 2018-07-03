#ifndef IMPLICITSURFACE_HPP
#define IMPLICITSURFACE_HPP

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

#include "kdtree.hpp"
#include "marching_cubes.hpp"
#include "triangle.hpp"

using namespace Eigen;

class ImplicitSurface {
  public:
    ImplicitSurface(std::shared_ptr<KDTree> kdtree, unsigned int nSubdivision,
        float r);

    void setGrid(unsigned int nSubdivision);
    void setRadius(float r);

    PointPointerList getOriginalPoints();
    PointPointerList getPositivePoints();
    PointPointerList getNegativePoints();
    std::vector<std::vector<std::vector<std::shared_ptr<Point>>>> getImplicitGridPoints();
    std::vector<Triangle> getMarchingCubesMesh();

    void computeImplicitGridPoints();

    float computeMLS(const Point &p);
    float evaluateImplicitFunction(const Point &p);
    void computeImplicitFunction();
    void computeMarchingCubes();

  private:
    void createAdditionalPoints();


    bool computeCoefficients(const Point &p);

    std::shared_ptr<KDTree> kdtree;

    // The Five Vectors
    PointPointerList points;
    PointPointerList positivePoints;
    PointPointerList negativePoints;
    std::vector<double> positiveFunctionValues;
    std::vector<double> negativeFunctionValues;

    std::vector<std::vector<std::vector<std::shared_ptr<Point>>>> implicitGridPoints;
    std::vector<VectorXf> implicitFunction;
    std::vector<Triangle> marchingCubesMesh;

    unsigned int gridSubdivision;
    float radius;

    unsigned int basisPolynomDegree = 1;
};

#endif // IMPLICITSURFACE_HPP
