#ifndef IMPLICITSURFACE_HPP
#define IMPLICITSURFACE_HPP

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

#include <kdtree.hpp>

using namespace Eigen;

class ImplicitSurface {
  public:
    ImplicitSurface(std::shared_ptr<KDTree> kdtree, unsigned int nX, unsigned int nY, unsigned int nZ,
        float r);

    void setGrid(unsigned int nX, unsigned int nY, unsigned int nZ);
    void setRadius(float r);

    std::vector<std::vector<std::vector<std::shared_ptr<Point>>>> getImplicitGridPoints();

    void computeImplicitGridPoints();

  private:
    void createAdditionalPoints();

    float computeMLS(const Point &p);

    std::shared_ptr<KDTree> kdtree;

    PointPointerList points;
    PointPointerList positivePoints;
    PointPointerList negativePoints;

    std::vector<double> positiveFunctionValues;
    std::vector<double> negativeFunctionValues;

    std::vector<std::vector<std::vector<std::shared_ptr<Point>>>> implicitGridPoints;

    unsigned int gridNX;
    unsigned int gridNY;
    unsigned int gridNZ;
    float radius;

    //TODO maybe trigger by gui
    unsigned int basisPolynomDegree = 0;
};

#endif // IMPLICITSURFACE_HPP
