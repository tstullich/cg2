#ifndef IMPLICITSURFACE_HPP
#define IMPLICITSURFACE_HPP

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include <kdtree.hpp>

class ImplicitSurface {
  public:
    ImplicitSurface(std::shared_ptr<KDTree> kdtree);

  private:
    void createAdditionalPoints();

    std::shared_ptr<KDTree> kdtree;

    PointPointerList points;
    PointPointerList positivePoints;
    PointPointerList negativePoints;

    std::vector<double> positiveFunctionValue;
    std::vector<double> negativeFunctionValue;
};

#endif // IMPLICITSURFACE_HPP
