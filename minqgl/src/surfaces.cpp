#include "surfaces.hpp"

Surfaces::Surfaces(std::shared_ptr<KDTree> kdtree, int m, int n, float r)
    : kdtree(std::move(kdtree)), M(m), N(n), radius(r) {
  std::cout << "kdtree:" << this->kdtree << std::endl;
  std::cout << "m:" << this->M << std::endl;
  std::cout << "n:" << this->N << std::endl;
  std::cout << "r:" << this->radius << std::endl;
}

PointPointerList Surfaces::getSurfaceMLS() {
  return surfaceMLS;
}

PointPointerList Surfaces::getSurfaceBTPS() {
  return surfaceBTPS;
}

void Surfaces::setGrid(int m, int n) {
  this->M = m;
  this->N = n;
  updateSurfacesMLS();
}

void Surfaces::setRadius(float r) {
  this->radius = r;
  updateSurfacesMLS();
}

void Surfaces::updateSurfacesMLS() {
  std::cout << "updateSurfacesMLS()" << std::endl;

  // first empty the container
  surfaceMLS.clear();

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
      surfaceMLS.push_back(std::make_shared<Point>(x, y, z));
    }
  }
}

// TODO wrong -> fix
float Surfaces::computeMLS(float x, float y) {

  std::shared_ptr<Point> p = std::make_shared<Point>(x, y, 0);
  PointPointerList points = kdtree->collectInRadius(*p, this->radius);

  unsigned int n = 6;
  MatrixXf A(n, n);
  VectorXf a(n), b(n);
  b *= 0;
  b[0] = 1;
  for(unsigned int i = 0; i < points.size(); i++) {
    std::shared_ptr<Point> p_i = points[i];
    a <<  1,                p_i->x,        p_i->y,
          pow(p_i->x, 2.0), p_i->x*p_i->y, pow(p_i->y, 2.0);
    for (unsigned int j = 1; j < n; j++) {
        b[j] += a[j];
    }
  }

  for(unsigned int i = 0; i < points.size(); i++) {
    std::shared_ptr<Point> p_i = points[i];
    float distance = p->distPoint(*p_i) / this->radius;
    float theta = pow(1.0 - distance, 4.0) * (4*distance + 1);
    a <<  1,                p_i->x,        p_i->y,
          pow(p_i->x, 2.0), p_i->x*p_i->y, pow(p_i->y, 2.0);
    for (unsigned int j = 0; j < n; j++) {
      for (unsigned int k = 0; k < n; k++) {
        A(j, k) += a[j]*a[k];
      }
      //add theta -> wendland component
      A(j, 0) += theta*a[j];
    }
  }

  std::cout << "A: " << A << "\n" << std::endl;
  std::cout << "b: " << b << "\n" << std::endl;
  MatrixXf A_inv = A.inverse();
  VectorXf X = A_inv*b;
  std::cout << "A^(-1): " << A_inv << "\n" << std::endl;
  std::cout << "x: " << X << "\n" << std::endl;

  // evalute polynom
  VectorXf c(n);
  c <<  1,              p->x,    p->y,
        pow(p->x, 2.0), p->x*p->y, pow(p->y, 2.0);
  float z = 0;
  for (unsigned int i = 0; i < n; i++) {
    z += c[i]*X[i];
  }
  return z;
}

void Surfaces::updateSurfacesBTPS() {
  std::cout << "updateSurfacesBTPS()" << std::endl;
}
