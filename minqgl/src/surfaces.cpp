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

// TODO implement
float Surfaces::computeMLS(float x, float y) {
  PointPointerList points = kdtree->collectInRadius(*std::make_shared<Point>(x, y, 0), this->radius);
  return 0;
}

void Surfaces::updateSurfacesBTPS() {
  std::cout << "updateSurfacesBTPS()" << std::endl;
}
