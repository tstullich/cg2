#include "ImplicitSurface.hpp"

ImplicitSurface::ImplicitSurface(std::shared_ptr<KDTree> kdtree)
  : kdtree(std::move(kdtree)) {

  std::cout << "Implicit Surface!" << std::endl;
  this->points = this->kdtree->getRootnode()->plist;
  this->createAdditionalPoints();
}

void ImplicitSurface::createAdditionalPoints() {
  Borders outerBox = kdtree->getRootnode()->borders;

  Point p1(outerBox.xMin, outerBox.yMin, outerBox.zMin);
  Point p2(outerBox.xMax, outerBox.yMax, outerBox.zMax);
  double epsilon = 0.01 * p1.distPoint(p2);

  for(unsigned int i = 0; i < this->points.size(); i++) {
    std::shared_ptr<Point> p = this->points[i];
    double eps = epsilon;
    while(true) {
      std::shared_ptr<Point> newPositivePoint = std::make_shared<Point>(
          p->x+eps*p->normal[0], p->y+eps*p->normal[1], p->z+eps*p->normal[2]);
      if(kdtree->isClosestPoint(*newPositivePoint, *p)) {
        kdtree->addToTree(newPositivePoint);
        positivePoints.push_back(newPositivePoint);
        positiveFunctionValue.push_back(eps);
        break;
      }
      eps /= 2;
    }
    eps = -epsilon;
    while(true) {
      std::shared_ptr<Point> newNegativePoint = std::make_shared<Point>(
          p->x+eps*p->normal[0], p->y+eps*p->normal[1], p->z+eps*p->normal[2]);
      if(kdtree->isClosestPoint(*newNegativePoint, *p)) {
        kdtree->addToTree(newNegativePoint);
        negativePoints.push_back(newNegativePoint);
        negativeFunctionValue.push_back(eps);
        break;
      }
      eps /= 2;
    }
  }
}
