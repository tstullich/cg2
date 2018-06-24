#ifndef IMPLICITSURFACE_HPP
#define IMPLICITSURFACE_HPP

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include <map>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

#include <kdtree.hpp>

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

    void computeImplicitGridPoints();

    float computeMLS(const Point &p);
    float evaluteImplicitFunction(const Point &p);
    void computeImplicitFunction();

  private:
    void createAdditionalPoints();


    bool computeCoefficients(const Point &p, VectorXf &coefficients);

    std::shared_ptr<KDTree> kdtree;

    // The Five Vectors
    PointPointerList points;
    PointPointerList positivePoints;
    PointPointerList negativePoints;
    std::vector<double> positiveFunctionValues;
    std::vector<double> negativeFunctionValues;

    std::vector<std::vector<std::vector<std::shared_ptr<Point>>>> implicitGridPoints;
    std::vector<VectorXf> implicitFunction;

    unsigned int gridSubdivision;
    float radius;

    //TODO maybe trigger by gui
    unsigned int basisPolynomDegree = 0;
};

#endif // IMPLICITSURFACE_HPP
