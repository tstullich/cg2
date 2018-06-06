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
  glm::vec3 norm;
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
  std::vector<quadPrimitiv> getControlFaces();
  std::vector<quadPrimitiv> getSurfaceFacesMLS();
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
  void updateControlFaces();

  /**
   * Compute surface based on MLS method
   */
  void updateSurfacesMLS(int k);
  void updateSurfacesFacesMLS(int k);

  /**
   * Compute surface based on Bézier Tensor Product Surface method
   */
  void updateSurfacesBTPS(int k);
  void updateSurfacesFacesBTPS(int k);

private:
  /*
   * Computes z value at the given point by using the MLS method.
   */
  float computeMLS(float x, float y);

  /*
   * Computes z value at the given point by using the BTPS method.
   */
  float computeBTPS(float u, float v);

  /*
   * applies deCasteljau twice to calculate the surface normal at (u,v)
   */
  glm::vec3 computeVertxNormalBTPS(float u, float v);

  glm::vec3 triangleNormal(std::shared_ptr<Point> v1,
                           std::shared_ptr<Point> v2,
                           std::shared_ptr<Point> v3);

  /**
   * compute face and vertex normals for given set of primitives
   */
  void computeNormalsMLS(int k);
  void computeNormalsBTPS(int k);
  void computeControlFaceNormals();

  PointPointerList getControlPointsAtM(int m);
  PointPointerList getControlPointsAtN(int n);

  PointPointerList controlPoints;
  PointPointerList surfaceMLS;
  PointPointerList surfaceBTPS;
  std::vector<quadPrimitiv> controlFaces;
  std::vector<quadPrimitiv> surfaceFacesMLS;
  std::vector<quadPrimitiv> surfaceFacesBTPS;
  std::shared_ptr<KDTree> kdtree;
  int M;
  int N;
  float radius;
};

#endif // SURFACES_HPP
