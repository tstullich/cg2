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

  PointPointerList getControlPoints();
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
   * compute control points based on kdtree, M and N using MLS
   */
  void updateControlPoints();

  /**
   * Compute surface based on MLS method
   */
  void updateSurfacesMLS(int k);

  /*
   * Computes z value at the given point by using the MLS method.
   */
  float computeMLS(float x, float y);

  /**
   * Compute surface based on Bézier Tensor Product Surface method
   */
  void updateSurfacesBTPS(int k);

  /*
   * Computes z value at the given point by using the BTPS method.
   */
  float computeBTPS(float u, float v);

private:
  PointPointerList getControlPointsAtM(int m);
  PointPointerList getControlPointsAtN(int n);

  PointPointerList controlPoints;
  PointPointerList surfaceMLS;
  PointPointerList surfaceBTPS;
  std::shared_ptr<KDTree> kdtree;
  int M;
  int N;
  float radius;
};

#endif // SURFACES_HPP
