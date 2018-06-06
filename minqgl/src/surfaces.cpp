#include "surfaces.hpp"

Surfaces::Surfaces(std::shared_ptr<KDTree> kdtree, int m, int n, float r)
    : kdtree(std::move(kdtree)), M(m), N(n), radius(r) {
  this->updateControlPoints();
}

PointPointerList Surfaces::getControlPoints() { return controlPoints; }

PointPointerList Surfaces::getSurfaceMLS() { return surfaceMLS; }

PointPointerList Surfaces::getSurfaceBTPS() { return surfaceBTPS; }

std::vector<quadPrimitiv> Surfaces::getControlFaces() { return controlFaces; }

std::vector<quadPrimitiv> Surfaces::getSurfaceFacesMLS() { return surfaceFacesMLS; }

std::vector<quadPrimitiv> Surfaces::getSurfaceFacesBTPS() { return surfaceFacesBTPS; }

void Surfaces::setGrid(int m, int n) {
  this->M = m;
  this->N = n;
  this->updateControlPoints();
}

void Surfaces::setRadius(float r) {
  this->radius = r;
  this->updateControlPoints();
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

void Surfaces::updateControlFaces() {
  // first empty the container
  controlPoints.clear();

  controlFaces = std::vector<quadPrimitiv>(M * N);

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
      std::shared_ptr<Point> newPoint = std::make_shared<Point>(x, y, z);
      controlPoints.push_back(std::make_shared<Point>(x, y, z));

      // 4 quadPrimitiv can be adjacend to this grid point
      // 1: m-1 n-1
      if (m > 0 && n > 0) {
        int quad_index = (m-1) * N + (n-1);
        controlFaces[quad_index].t0.p1 = newPoint;
        controlFaces[quad_index].t1.p0 = newPoint;
      }
      // 2: m-1 n
      if (m > 0 && n < N) {
        int quad_index = (m-1) * N + (n);
        controlFaces[quad_index].t0.p2 = newPoint;
      }
      // 1: m   n-1
      if (m < M && n > 0) {
        int quad_index = (m) * N + (n-1);
        controlFaces[quad_index].t1.p2 = newPoint;
      }
      // 4: m   n
      if (m < M && n < N) {
        int quad_index = (m) * N + (n);
        controlFaces[quad_index].t0.p0 = newPoint;
        controlFaces[quad_index].t1.p1 = newPoint;
      }
    }
  }

  computeControlFaceNormals();

  return;
}

void Surfaces::updateSurfacesMLS(int k) {
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

void Surfaces::updateSurfacesFacesMLS(int k) {
  int subdivM = k * M;
  int subdivN = k * N;

  surfaceFacesMLS = std::vector<quadPrimitiv>(subdivM * subdivN);

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
      std::shared_ptr<Point> newPoint = std::make_shared<Point>(x, y, z);

      // 4 quadPrimitiv can be adjacend to this grid point
      // 1: m-1 n-1
      if (m > 0 && n > 0) {
        int quad_index = (m-1) * subdivN + (n-1);
        surfaceFacesMLS[quad_index].t0.p1 = newPoint;
        surfaceFacesMLS[quad_index].t1.p0 = newPoint;
      }
      // 2: m-1 n
      if (m > 0 && n < subdivN) {
        int quad_index = (m-1) * subdivN + (n);
        surfaceFacesMLS[quad_index].t0.p2 = newPoint;
      }
      // 1: m   n-1
      if (m < subdivM && n > 0) {
        int quad_index = (m) * subdivN + (n-1);
        surfaceFacesMLS[quad_index].t1.p2 = newPoint;
      }
      // 4: m   n
      if (m < subdivM && n < subdivN) {
        int quad_index = (m) * subdivN + (n);
        surfaceFacesMLS[quad_index].t0.p0 = newPoint;
        surfaceFacesMLS[quad_index].t1.p1 = newPoint;
      }
    }
  }

  computeNormalsMLS(k);

  return;
}

float Surfaces::computeMLS(float x, float y) {
  std::shared_ptr<Point> p = std::make_shared<Point>(x, y, 0.0);
  PointPointerList points = kdtree->collectInRadius(*p, this->radius);

  if (points.size() == 0) return 0;

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
    a << 1.0, p_i->x, p_i->y, pow(p_i->x, 2.0), p_i->x * p_i->y,
        pow(p_i->y, 2.0);
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
    points.push_back(controlPoints[(N+1) * m + n]);
  }
  return points;
}

PointPointerList Surfaces::getControlPointsAtN(int n) {
  PointPointerList points;
  for (int m = 0; m <= M; ++m) {
    points.push_back(controlPoints[(N+1) * m + n]);
  }
  return points;
}

void Surfaces::updateSurfacesBTPS(int k) {
  // first empty the container
  surfaceBTPS.clear();

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
      float z = computeBTPS(x, y);
      /* glm::vec3 norm = computeNormalBTPS(x, y); */
      surfaceBTPS.push_back(std::make_shared<Point>(x, y, z));
      /* surfaceBTPS.back()->normal = norm; */
    }
  }
}

void Surfaces::updateSurfacesFacesBTPS(int k) {
  int subdivM = k * M;
  int subdivN = k * N;

  surfaceFacesBTPS = std::vector<quadPrimitiv>(subdivM * subdivN);

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
      float z = computeBTPS(x, y);
      std::shared_ptr<Point> newPoint = std::make_shared<Point>(x, y, z);
      /* newPoint->normal = computeVertxNormalBTPS(x, y); */

      // 4 quadPrimitiv can be adjacend to this grid point
      // 1: m-1 n-1
      if (m > 0 && n > 0) {
        int quad_index = (m-1) * subdivN + (n-1);
        surfaceFacesBTPS[quad_index].t0.p1 = newPoint;
        surfaceFacesBTPS[quad_index].t1.p0 = newPoint;
      }
      // 2: m-1 n
      if (m > 0 && n < subdivN) {
        int quad_index = (m-1) * subdivN + (n);
        surfaceFacesBTPS[quad_index].t0.p2 = newPoint;
      }
      // 1: m   n-1
      if (m < subdivM && n > 0) {
        int quad_index = (m) * subdivN + (n-1);
        surfaceFacesBTPS[quad_index].t1.p2 = newPoint;
      }
      // 4: m   n
      if (m < subdivM && n < subdivN) {
        int quad_index = (m) * subdivN + (n);
        surfaceFacesBTPS[quad_index].t0.p0 = newPoint;
        surfaceFacesBTPS[quad_index].t1.p1 = newPoint;
      }
    }
  }

  computeNormalsBTPS(k);

  return;
}

glm::vec3 deCasteljau(PointPointerList points, float u, int i, int r) {
  if (r == 0) {
    return points[i]->toVec3();
  }

  glm::vec3 p1 = deCasteljau(points, u, i+1, r-1);
  glm::vec3 p2 = deCasteljau(points, u, i, r-1);

  return (u * p1) + ((1.0f - u) * p2);
}

float Surfaces::computeBTPS(float u, float v) {
  // compute bezier points on curves along n (x-axis) direction
  PointPointerList points;
  for (int m = 0; m <= M; ++m) {
    PointPointerList mPoints = getControlPointsAtM(m);
    glm::vec3 bezierPoint = deCasteljau(mPoints, u, 0, mPoints.size()-1);
    points.push_back(std::make_shared<Point>(bezierPoint));
  }

  // compute z from points
  glm::vec3 bezierPoint = deCasteljau(points, v, 0, points.size()-1);

  return bezierPoint[2];
}

glm::vec3 Surfaces::computeVertxNormalBTPS(float u, float v) {
  // compute tangent 1
  PointPointerList mPoints;
  for (int m = 0; m <= M; ++m) {
    PointPointerList tmp = getControlPointsAtM(m);
    glm::vec3 bezierPoint = deCasteljau(tmp, u, 0, tmp.size()-1);
    mPoints.push_back(std::make_shared<Point>(bezierPoint));
  }
  glm::vec3 tangent_11 = glm::normalize(deCasteljau(mPoints, v, 0, mPoints.size()-2));
  glm::vec3 tangent_12 = glm::normalize(deCasteljau(mPoints, v, 1, mPoints.size()-2));
  glm::vec3 tangent_1 = glm::normalize(tangent_11 - tangent_12);

  // compute tangent 2
  PointPointerList nPoints;
  for (int n = 0; n <= N; ++n) {
    PointPointerList tmp = getControlPointsAtN(n);
    glm::vec3 bezierPoint = deCasteljau(tmp, u, 0, tmp.size()-1);
    nPoints.push_back(std::make_shared<Point>(bezierPoint));
  }
  glm::vec3 tangent_21 = glm::normalize(deCasteljau(nPoints, v, 0, nPoints.size()-2));
  glm::vec3 tangent_22 = glm::normalize(deCasteljau(nPoints, v, 1, nPoints.size()-2));
  glm::vec3 tangent_2 = glm::normalize(tangent_21 - tangent_22);

  glm::vec3 norm = glm::normalize(glm::cross(tangent_2, tangent_1));

  return norm;
}

glm::vec3 Surfaces::triangleNormal(std::shared_ptr<Point> v1,
                                   std::shared_ptr<Point> v2,
                                   std::shared_ptr<Point> v3) {
  // Get the cross product of u - v
  glm::vec3 u(v2->x - v1->x, v2->y - v1->y, v2->z - v1->z);
  glm::vec3 v(v3->x - v1->x, v3->y - v1->y, v3->z - v1->z);
  auto normalX = (u.y * v.z) - (u.z * v.y);
  auto normalY = (u.z * v.x) - (u.x * v.z);
  auto normalZ = (u.x * v.y) - (u.y * v.x);

  // Normalize the cross product
  float d = sqrt(normalX * normalX + normalY * normalY + normalZ * normalZ);

  return glm::vec3(normalX / d, normalY / d, normalZ / d);
}

void Surfaces::computeNormalsMLS(int k) {
  for (int i = 0; i < surfaceFacesMLS.size(); ++i) {
    struct trianglePrimitiv t0 = surfaceFacesMLS[i].t0;
    struct trianglePrimitiv t1 = surfaceFacesMLS[i].t1;

    surfaceFacesMLS[i].t0.norm = triangleNormal(t0.p0, t0.p1, t0.p2);
    surfaceFacesMLS[i].t1.norm = triangleNormal(t1.p0, t1.p1, t1.p2);
  }

  int subdivM = k * M;
  int subdivN = k * N;

  for (int m = 0; m <= subdivM; ++m) {
    for (int n = 0; n <= subdivN; ++n) {
      int cntCases = 0;
      glm::vec3 newNorm = glm::vec3(0.0f, 0.0f, 0.0f);
      // first we sum up all face normals
      // 4 quadPrimitiv can be adjacend to this grid point
      // 1: m-1 n-1
      if (m > 0 && n > 0) {
        int quad_index = (m-1) * subdivN + (n-1);
        newNorm += surfaceFacesMLS[quad_index].t0.norm;
        newNorm += surfaceFacesMLS[quad_index].t1.norm;
        cntCases += 2;
      }
      // 2: m-1 n
      if (m > 0 && n < subdivN) {
        int quad_index = (m-1) * subdivN + (n);
        newNorm += surfaceFacesMLS[quad_index].t0.norm;
        cntCases += 1;
      }
      // 1: m   n-1
      if (m < subdivM && n > 0) {
        int quad_index = (m) * subdivN + (n-1);
        newNorm += surfaceFacesMLS[quad_index].t1.norm;
        cntCases += 1;
      }
      // 4: m   n
      if (m < subdivM && n < subdivN) {
        int quad_index = (m) * subdivN + (n);
        newNorm += surfaceFacesMLS[quad_index].t0.norm;
        newNorm += surfaceFacesMLS[quad_index].t1.norm;
        cntCases += 2;
      }
      // second we compute the average and assign it
      newNorm /= cntCases;
      newNorm = glm::normalize(newNorm);
      // 1: m-1 n-1
      if (m > 0 && n > 0) {
        int quad_index = (m-1) * subdivN + (n-1);
        surfaceFacesMLS[quad_index].t0.p1->normal = newNorm;
        surfaceFacesMLS[quad_index].t1.p0->normal = newNorm;
      }
      // 2: m-1 n
      if (m > 0 && n < subdivN) {
        int quad_index = (m-1) * subdivN + (n);
        surfaceFacesMLS[quad_index].t0.p2->normal = newNorm;
      }
      // 1: m   n-1
      if (m < subdivM && n > 0) {
        int quad_index = (m) * subdivN + (n-1);
        surfaceFacesMLS[quad_index].t1.p2->normal = newNorm;
      }
      // 4: m   n
      if (m < subdivM && n < subdivN) {
        int quad_index = (m) * subdivN + (n);
        surfaceFacesMLS[quad_index].t0.p0->normal = newNorm;
        surfaceFacesMLS[quad_index].t1.p1->normal = newNorm;
      }
    }
  }
}

void Surfaces::computeNormalsBTPS(int k) {
  for (int i = 0; i < surfaceFacesBTPS.size(); ++i) {
    struct trianglePrimitiv t0 = surfaceFacesBTPS[i].t0;
    struct trianglePrimitiv t1 = surfaceFacesBTPS[i].t1;

    surfaceFacesBTPS[i].t0.norm = triangleNormal(t0.p0, t0.p1, t0.p2);
    surfaceFacesBTPS[i].t1.norm = triangleNormal(t1.p0, t1.p1, t1.p2);
  }

  int subdivM = k * M;
  int subdivN = k * N;

  for (int m = 0; m <= subdivM; ++m) {
    for (int n = 0; n <= subdivN; ++n) {
      int cntCases = 0;
      glm::vec3 newNorm = glm::vec3(0.0f, 0.0f, 0.0f);
      // first we sum up all face normals
      // 4 quadPrimitiv can be adjacend to this grid point
      // 1: m-1 n-1
      if (m > 0 && n > 0) {
        int quad_index = (m-1) * subdivN + (n-1);
        newNorm += surfaceFacesBTPS[quad_index].t0.norm;
        newNorm += surfaceFacesBTPS[quad_index].t1.norm;
        cntCases += 2;
      }
      // 2: m-1 n
      if (m > 0 && n < subdivN) {
        int quad_index = (m-1) * subdivN + (n);
        newNorm += surfaceFacesBTPS[quad_index].t0.norm;
        cntCases += 1;
      }
      // 1: m   n-1
      if (m < subdivM && n > 0) {
        int quad_index = (m) * subdivN + (n-1);
        newNorm += surfaceFacesBTPS[quad_index].t1.norm;
        cntCases += 1;
      }
      // 4: m   n
      if (m < subdivM && n < subdivN) {
        int quad_index = (m) * subdivN + (n);
        newNorm += surfaceFacesBTPS[quad_index].t0.norm;
        newNorm += surfaceFacesBTPS[quad_index].t1.norm;
        cntCases += 2;
      }
      // second we compute the average and assign it
      newNorm /= cntCases;
      newNorm = glm::normalize(newNorm);
      // 1: m-1 n-1
      if (m > 0 && n > 0) {
        int quad_index = (m-1) * subdivN + (n-1);
        surfaceFacesBTPS[quad_index].t0.p1->normal = newNorm;
        surfaceFacesBTPS[quad_index].t1.p0->normal = newNorm;
      }
      // 2: m-1 n
      if (m > 0 && n < subdivN) {
        int quad_index = (m-1) * subdivN + (n);
        surfaceFacesBTPS[quad_index].t0.p2->normal = newNorm;
      }
      // 1: m   n-1
      if (m < subdivM && n > 0) {
        int quad_index = (m) * subdivN + (n-1);
        surfaceFacesBTPS[quad_index].t1.p2->normal = newNorm;
      }
      // 4: m   n
      if (m < subdivM && n < subdivN) {
        int quad_index = (m) * subdivN + (n);
        surfaceFacesBTPS[quad_index].t0.p0->normal = newNorm;
        surfaceFacesBTPS[quad_index].t1.p1->normal = newNorm;
      }
    }
  }
}

void Surfaces::computeControlFaceNormals() {
  for (int i = 0; i < controlFaces.size(); ++i) {
    struct trianglePrimitiv t0 = controlFaces[i].t0;
    struct trianglePrimitiv t1 = controlFaces[i].t1;

    glm::vec3 normal0 = triangleNormal(t0.p0, t0.p1, t0.p2);
    controlFaces[i].t0.norm = normal0;
    glm::vec3 normal1 = triangleNormal(t1.p0, t1.p1, t1.p2);
    controlFaces[i].t1.norm = normal1;
  }
}
