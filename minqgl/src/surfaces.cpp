#include "surfaces.hpp"

Surfaces::Surfaces(std::shared_ptr<KDTree> kdtree, int m, int n, float r)
    : kdtree(std::move(kdtree)), M(m), N(n), radius(r) {
  std::cout << "kdtree:" << this->kdtree << std::endl;
  std::cout << "m:" << this->M << std::endl;
  std::cout << "n:" << this->N << std::endl;
  std::cout << "r:" << this->radius << std::endl;
}

PointPointerList Surfaces::getControlPoints() { return controlPoints; }

PointPointerList Surfaces::getSurfaceMLS() { return surfaceMLS; }

PointPointerList Surfaces::getSurfaceBTPS() { return surfaceBTPS; }

void Surfaces::setGrid(int m, int n) {
  this->M = m;
  this->N = n;
}

void Surfaces::setRadius(float r) {
  this->radius = r;
}

void Surfaces::updateControlPoints() {
  // first empty the container
  controlPoints.clear();

  Borders borders = kdtree->getRootnode()->borders;
  float xMin = borders.xMin;
  float xMax = borders.xMax;
  float yMin = borders.yMin;
  float yMax = borders.yMax;
  float mDelta = double(yMax - yMin) / M;
  float nDelta = double(xMax - xMin) / N;

  // for each grid intersection compute new point
  for (int m = 0; m <= M; ++m) {
    for (int n = 0; n <= N; ++n) {
      float x = xMin + (n * nDelta);
      float y = yMin + (m * mDelta);
      float z = computeMLS(x, y);
      controlPoints.push_back(std::make_shared<Point>(x, y, z));
    }
  }
}

void Surfaces::updateSurfacesMLS(int k) {
  std::cout << "updateSurfacesMLS()" << std::endl;

  // first empty the container
  surfaceMLS.clear();

  int subdivM = k * M;
  int subdivN = k * N;

  Borders borders = kdtree->getRootnode()->borders;
  float xMin = borders.xMin;
  float xMax = borders.xMax;
  float yMin = borders.yMin;
  float yMax = borders.yMax;
  float mDelta = double(yMax - yMin) / subdivM;
  float nDelta = double(xMax - xMin) / subdivN;

  // for each grid intersection compute new point
  for (int m = 0; m <= subdivM; ++m) {
    for (int n = 0; n <= subdivN; ++n) {
      float x = xMin + (n * nDelta);
      float y = yMin + (m * mDelta);
      float z = computeMLS(x, y);
      surfaceMLS.push_back(std::make_shared<Point>(x, y, z));
    }
  }
}

float Surfaces::computeMLS(float x, float y) {
  std::shared_ptr<Point> p = std::make_shared<Point>(x, y, 0);
  PointPointerList points = kdtree->collectInRadius(*p, this->radius);

  if(points.size() == 0)
    return 0;

  unsigned int n = 6;
  MatrixXf A(n, n);
  VectorXf a(n), b(n);
  A *= 0;
  b *= 0;

  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      A(i, j) = 0.0;
    }
      b(i) = 0.0;
  }

  for (unsigned int i = 0; i < points.size(); i++) {
    std::shared_ptr<Point> p_i = points[i];
    float distance = p->distPoint(*p_i) / this->radius;
    // wendland component
    float theta = pow(1.0 - distance, 4.0) * (4.0 * distance + 1);
    a << 1.0, p_i->x, p_i->y, pow(p_i->x, 2.0), p_i->x * p_i->y, pow(p_i->y, 2.0);
    for (unsigned int j = 0; j < n; j++) {
      for (unsigned int k = 0; k < n; k++) {
        A(j, k) += a[j] * a[k] * theta;
      }
      b[j] += a[j] * theta * p_i->z;
    }
  }

  MatrixXf A_inv = A.inverse();
  VectorXf X = A_inv * b;

  // evalute polynom
  VectorXf c(n);
  c << 1.0, p->x, p->y, pow(p->x, 2.0), p->x * p->y, pow(p->y, 2.0);
  float z = 0.0;
  for (unsigned int i = 0; i < n; i++) {
    z = c.dot(X);
  }
  return z;
}

PointPointerList Surfaces::getControlPointsAtM(int m) {
  PointPointerList points;
  for (int n = 0; n <= N; ++n) {
    points.push_back(controlPoints[M*m + n]);
  }
  return points;
}

PointPointerList Surfaces::getControlPointsAtN(int n) {
  PointPointerList points;
  for (int m = 0; m <= M; ++m) {
    points.push_back(controlPoints[M*m + n]);
  }
  return points;
}

void Surfaces::updateSurfacesBTPS(int k) {
  PointPointerList points = getControlPointsAtM(1);
  for (int i = 0; i < points.size(); ++i) {
    std::cout << i << ": (" << points[i]->x << ",\t" << points[i]->y << ")" << std::endl;
  }
}
