#ifndef SURFACES_HPP
#define SURFACES_HPP

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

#include "kdtree.hpp"

using namespace Eigen;

class Surfaces {
public:
  Surfaces(std::shared_ptr<KDTree> kdtree, int m, int n, float r);

  PointPointerList getSurfaceMLS();
  PointPointerList getSurfaceBTPS();

  /**
   * Update m and n values, no computations.
   */
  void setGrid(int m, int n);

  /**
   * Update radius value, no computations.
   */
  void setRadius(float r);

  /**
   * Compute surface based on MLS method
   */
  void updateSurfacesMLS();

  /*
   * Computes z value at the given point by using the MLS method.
   */
  float computeMLS(float x, float y);

  /**
   * Compute surface based on Bézier Tensor Product Surface method
   */
  void updateSurfacesBTPS();

private:
  PointPointerList surfaceMLS;
  PointPointerList surfaceBTPS;
  std::shared_ptr<KDTree> kdtree;
  int M;
  int N;
  float radius;
};

#endif // SURFACES_HPP
