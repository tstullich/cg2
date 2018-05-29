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

struct trianglePrimitiv {
  std::shared_ptr<Point> p0;
  std::shared_ptr<Point> p1;
  std::shared_ptr<Point> p2;
};

struct quadPrimitiv {
  struct trianglePrimitiv t0;
  struct trianglePrimitiv t1;
};

class Surfaces {
public:
  Surfaces(std::shared_ptr<KDTree> kdtree, int m, int n, float r);

  PointPointerList getControlPoints();
  PointPointerList getSurfaceMLS();
  PointPointerList getSurfaceBTPS();
  std::vector<quadPrimitiv> getSurfaceFacesBTPS();

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
  void updateSurfacesFacesBTPS(int k);

  /*
   * Computes z value at the given point by using the BTPS method.
   */
  float computeBTPS(float u, float v);

  glm::vec3 computeNormalBTPS(float u, float v);

private:
  PointPointerList getControlPointsAtM(int m);
  PointPointerList getControlPointsAtN(int n);

  PointPointerList controlPoints;
  PointPointerList surfaceMLS;
  PointPointerList surfaceBTPS;
  std::vector<quadPrimitiv> surfaceFacesBTPS;
  std::shared_ptr<KDTree> kdtree;
  int M;
  int N;
  float radius;
};

#endif // SURFACES_HPP
