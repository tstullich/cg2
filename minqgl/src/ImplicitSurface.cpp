#include "ImplicitSurface.hpp"

ImplicitSurface::ImplicitSurface(std::shared_ptr<KDTree> kdtree, unsigned int nX, unsigned int nY,
    unsigned int nZ, float r)
  : kdtree(std::move(kdtree)), gridNX(nX), gridNY(nY), gridNZ(nZ), radius(r) {

  this->points = this->kdtree->getRootnode()->plist;
  this->createAdditionalPoints();
}

void ImplicitSurface::setGrid(unsigned int nX, unsigned int nY, unsigned int nZ) {
  this->gridNX = nX;
  this->gridNY = nY;
  this->gridNZ = nZ;
  computeImplicitGridPoints();
}

void ImplicitSurface::setRadius(float r) {
  this->radius = r;
  computeImplicitGridPoints();
}

std::vector<std::vector<std::vector<std::shared_ptr<Point>>>> ImplicitSurface::getImplicitGridPoints() {
  return implicitGridPoints;
}

void ImplicitSurface::computeImplicitGridPoints() {
  this->implicitGridPoints.clear();

  Borders outerBox = kdtree->getRootnode()->borders;
  float xDelta = (outerBox.xMax - outerBox.xMin) / gridNX;
  float yDelta = (outerBox.yMax - outerBox.yMin) / gridNY;
  float zDelta = (outerBox.zMax - outerBox.zMin) / gridNZ;

  float x, y, z;
  for(unsigned int i=0; i <= gridNX; i++) {
    this->implicitGridPoints.push_back(std::vector<std::vector<std::shared_ptr<Point>>>());
    x = outerBox.xMin + (i * xDelta);
    for(unsigned int j=0; j <= gridNY; j++) {
      this->implicitGridPoints[i].push_back(std::vector<std::shared_ptr<Point>>());
      y = outerBox.yMin + (j * yDelta);
      for(unsigned int k=0; k <= gridNZ; k++) {
        z = outerBox.zMin + (k * zDelta);
        Point p(x, y, z);
        p.f = this->computeMLS(p);
        this->implicitGridPoints[i][j].push_back(std::make_shared<Point>(p));
      }
    }
  }
}

float ImplicitSurface::computeMLS(const Point &p) {

  PointPointerList points = kdtree->collectInRadius(p, this->radius);

  unsigned int n;

  if(this->basisPolynomDegree == 0)
    n = 1;
  else if(this->basisPolynomDegree == 1)
    n = 4;
  else if(this->basisPolynomDegree == 2)
    n = 10;

  //TODO maybe change
  if (points.size() < 3*n) {
    //std::cerr << "No points in radius " << this->radius << std::endl;
    return std::numeric_limits<float>::max();
  }

  MatrixXf A(n, n);
  VectorXf a(n), b(n);

  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      A(i, j) = 0.0;
    }
    b(i) = 0.0;
  }

  for (unsigned int i = 0; i < points.size(); i++) {
    std::shared_ptr<Point> p_i = points[i];
    float distance = p.distPoint(*p_i) / this->radius;
    // wendland component
    float theta = pow(1.0 - distance, 4.0) * (4.0 * distance + 1);

    if(this->basisPolynomDegree == 0)
      a << 1.0;
    else if(this->basisPolynomDegree == 1)
      a << 1.0, p_i->x, p_i->y, p_i->z;
    else if(this->basisPolynomDegree == 2)
      a << 1.0, p_i->x, p_i->y, p_i->z, p_i->x * p_i->y, p_i->x * p_i->z, p_i->y * p_i->z,
          pow(p_i->x, 2.0), pow(p_i->y, 2.0), pow(p_i->z, 2.0);

    for (unsigned int j = 0; j < n; j++) {
      for (unsigned int k = 0; k < n; k++) {
        A(j, k) += a[j] * a[k] * theta;
      }
      b[j] += a[j] * theta * p_i->f;
    }
  }

  MatrixXf A_inv = A.inverse();
  VectorXf X = A_inv * b;

  // evalute polynom
  VectorXf c(n);
  if(this->basisPolynomDegree == 0)
    c << 1.0;
  if(this->basisPolynomDegree == 1)
    c << 1.0, p.x, p.y, p.z;
  if(this->basisPolynomDegree == 2)
    c << 1.0, p.x, p.y, p.z, p.x * p.y, p.x * p.z, p.y * p.z,
        pow(p.x, 2.0), pow(p.y, 2.0), pow(p.z, 2.0);

  float v;
  for (unsigned int i = 0; i < n; i++) {
    v = c.dot(X);
  }
  return v;
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
        newPositivePoint->f = eps;
        kdtree->addToTree(newPositivePoint);
        positivePoints.push_back(newPositivePoint);
        positiveFunctionValues.push_back(eps);
        break;
      }
      eps /= 2;
    }
    eps = -epsilon;
    while(true) {
      std::shared_ptr<Point> newNegativePoint = std::make_shared<Point>(
          p->x+eps*p->normal[0], p->y+eps*p->normal[1], p->z+eps*p->normal[2]);
      if(kdtree->isClosestPoint(*newNegativePoint, *p)) {
        newNegativePoint->f = eps;
        kdtree->addToTree(newNegativePoint);
        negativePoints.push_back(newNegativePoint);
        negativeFunctionValues.push_back(eps);
        break;
      }
      eps /= 2;
    }
  }
}
