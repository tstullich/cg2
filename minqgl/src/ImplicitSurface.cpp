#include "ImplicitSurface.hpp"

ImplicitSurface::ImplicitSurface(std::shared_ptr<KDTree> kdtree,
                                 unsigned int nSubdivision, float r)
    : kdtree(std::move(kdtree)), gridSubdivision(nSubdivision), radius(r) {
  this->points = this->kdtree->getRootnode()->plist;
  this->createAdditionalPoints();
}

void ImplicitSurface::setGrid(unsigned int nSubdivision) {
  this->gridSubdivision = nSubdivision;
  computeImplicitGridPoints();
}

void ImplicitSurface::setRadius(float r) {
  this->radius = r;
  computeImplicitGridPoints();
}

PointPointerList ImplicitSurface::getOriginalPoints() {
  return this->points;
}

PointPointerList ImplicitSurface::getPositivePoints() {
  return this->positivePoints;
}

PointPointerList ImplicitSurface::getNegativePoints() {
  return this->negativePoints;
}

std::vector<std::vector<std::vector<std::shared_ptr<Point>>>>
ImplicitSurface::getImplicitGridPoints() {
  return implicitGridPoints;
}

std::vector<Triangle> ImplicitSurface::getMarchingCubesMesh() {
  return marchingCubesMesh;
}

void ImplicitSurface::computeImplicitGridPoints() {
  this->implicitGridPoints.clear();

  Borders outerBox = kdtree->getRootnode()->borders;
  float xDelta = (outerBox.xMax - outerBox.xMin) / gridSubdivision;
  float yDelta = (outerBox.yMax - outerBox.yMin) / gridSubdivision;
  float zDelta = (outerBox.zMax - outerBox.zMin) / gridSubdivision;

  float x, y, z;
  for (unsigned int i = 0; i <= gridSubdivision; i++) {
    this->implicitGridPoints.push_back(
        std::vector<std::vector<std::shared_ptr<Point>>>());
    x = outerBox.xMin + (i * xDelta);
    for (unsigned int j = 0; j <= gridSubdivision; j++) {
      this->implicitGridPoints[i].push_back(
          std::vector<std::shared_ptr<Point>>());
      y = outerBox.yMin + (j * yDelta);
      for (unsigned int k = 0; k <= gridSubdivision; k++) {
        z = outerBox.zMin + (k * zDelta);
        Point p(x, y, z);
        p.f = this->computeMLS(p);
        this->implicitGridPoints[i][j].push_back(std::make_shared<Point>(p));
      }
    }
  }
}

float ImplicitSurface::computeMLS(const Point &p) {
  PointPointerList collectedPoints = kdtree->collectInRadius(p, this->radius);

  unsigned int n = 1;

  if (this->basisPolynomDegree == 0)
    n = 1;
  else if (this->basisPolynomDegree == 1)
    n = 4;
  else if (this->basisPolynomDegree == 2)
    n = 10;

  if (collectedPoints.size() < n) {
    // std::cerr << "No points in radius " << this->radius << std::endl;
    return std::numeric_limits<float>::max();
  }

  MatrixXf A(n, n);
  VectorXf ao(n), ap(n), an(n), b(n);
  for (unsigned i = 0; i < n; ++i) {
    for (unsigned j = 0; j < n; ++j) {
      A(i, j) = 0.0;
    }
    b(i) = 0.0;
  }

  // for each original point fill up the MLS componants
  for (std::shared_ptr<Point> po_i : collectedPoints) {
    // get corresponding positiv and negativ point
    std::shared_ptr<Point> pp_i = po_i->positivePoint;
    std::shared_ptr<Point> np_i = po_i->negativePoint;

    float distance_po = p.distPoint(*po_i) / radius;
    float distance_pp = p.distPoint(*pp_i) / radius;
    float distance_np = p.distPoint(*np_i) / radius;
    float theta_po = pow(1.0 - distance_po, 4.0) * (4.0 * distance_po + 1);
    float theta_pp = pow(1.0 - distance_pp, 4.0) * (4.0 * distance_pp + 1);
    float theta_np = pow(1.0 - distance_np, 4.0) * (4.0 * distance_np + 1);

    // set up basis vector for filling A and b
    if (this->basisPolynomDegree == 0) {
      ao << 1.0;
      ap << 1.0;
      an << 1.0;
    } else if (this->basisPolynomDegree == 1) {
      ao << 1.0, po_i->x, po_i->y, po_i->z;
      ap << 1.0, pp_i->x, pp_i->y, pp_i->z;
      an << 1.0, np_i->x, np_i->y, np_i->z;
    } else if (this->basisPolynomDegree == 2) {
      ao << 1.0, po_i->x, po_i->y, po_i->z, po_i->x * po_i->y,
          po_i->x * po_i->z, po_i->y * po_i->z, pow(po_i->x, 2.0),
          pow(po_i->y, 2.0), pow(po_i->z, 2.0);
      ap << 1.0, pp_i->x, pp_i->y, pp_i->z, pp_i->x * pp_i->y,
          pp_i->x * pp_i->z, pp_i->y * pp_i->z, pow(pp_i->x, 2.0),
          pow(pp_i->y, 2.0), pow(pp_i->z, 2.0);
      an << 1.0, np_i->x, np_i->y, np_i->z, np_i->x * np_i->y,
          np_i->x * np_i->z, np_i->y * np_i->z, pow(np_i->x, 2.0),
          pow(np_i->y, 2.0), pow(np_i->z, 2.0);
    }

    for (unsigned int j = 0; j < n; j++) {
      for (unsigned int k = 0; k < n; k++) {
        A(j, k) += ao[j] * ao[k] * theta_po;
        A(j, k) += ap[j] * ap[k] * theta_pp;
        A(j, k) += an[j] * an[k] * theta_np;
      }
      b[j] += ao[j] * theta_po * po_i->f;
      b[j] += ap[j] * theta_pp * pp_i->f;
      b[j] += an[j] * theta_np * np_i->f;
    }
  }

  MatrixXf A_inv = A.inverse();
  VectorXf X = A_inv * b;

  // evalute polynom
  VectorXf c(n);
  if (this->basisPolynomDegree == 0) c << 1.0;
  if (this->basisPolynomDegree == 1) c << 1.0, p.x, p.y, p.z;
  if (this->basisPolynomDegree == 2)
    c << 1.0, p.x, p.y, p.z, p.x * p.y, p.x * p.z, p.y * p.z, pow(p.x, 2.0),
        pow(p.y, 2.0), pow(p.z, 2.0);

  float v = c.dot(X);

  return v;
}

float ImplicitSurface::evaluateImplicitFunction(const Point &p) {
  if (this->implicitFunction.size() == 0) {
    std::cout << "evaluateImplicitFunction() error 1" << std::endl;
    return std::numeric_limits<float>::max();
  }

  int n = this->implicitFunction[0].size();
  float v = std::numeric_limits<float>::max();

  for (unsigned i = 0; i < this->implicitFunction.size(); ++i) {
    VectorXf c(n);
    if (n == 4) {
      c << 1.0, p.x, p.y, p.z;
    } else if (n == 10) {
      c << 1.0, p.x, p.y, p.z, p.x * p.y, p.x * p.z, p.y * p.z, pow(p.x, 2.0),
          pow(p.y, 2.0), pow(p.z, 2.0);
    } else {
      std::cout << "evaluateImplicitFunction() error 2" << std::endl;
      return std::numeric_limits<float>::max();
    }

    float tmp = c.dot(implicitFunction[i]);
    v = (tmp < v) ? tmp : v;
  }

  return v;
}

void ImplicitSurface::computeImplicitFunction() {
  this->implicitFunction.clear();

  Borders outerBox = kdtree->getRootnode()->borders;
  float xDelta = (outerBox.xMax - outerBox.xMin) / gridSubdivision;
  float yDelta = (outerBox.yMax - outerBox.yMin) / gridSubdivision;
  float zDelta = (outerBox.zMax - outerBox.zMin) / gridSubdivision;

  float x, y, z;
  for (unsigned int i = 0; i <= gridSubdivision; i++) {
    for (unsigned int j = 0; j <= gridSubdivision; j++) {
      for (unsigned int k = 0; k <= gridSubdivision; k++) {
        x = outerBox.xMin + (i * xDelta);
        y = outerBox.yMin + (j * yDelta);
        z = outerBox.zMin + (k * zDelta);
        Point p(x, y, z);
        computeCoefficients(p);
      }
    }
  }
}

bool ImplicitSurface::computeCoefficients(const Point &p) {
  PointPointerList collectedPoints = kdtree->collectInRadius(p, this->radius);

  unsigned int n = 1;

  if (this->basisPolynomDegree == 0)
    n = 1;
  else if (this->basisPolynomDegree == 1)
    n = 4;
  else if (this->basisPolynomDegree == 2)
    n = 10;

  if (collectedPoints.size() < n) {
    // std::cerr << "No points in radius " << this->radius << std::endl;
    return std::numeric_limits<float>::max();
  }

  MatrixXf A(n, n);
  VectorXf ao(n), ap(n), an(n), b(n);
  for (unsigned i = 0; i < n; ++i) {
    for (unsigned j = 0; j < n; ++j) {
      A(i, j) = 0.0;
    }
    b(i) = 0.0;
  }

  // for each original point fill up the MLS componants
  for (std::shared_ptr<Point> po_i : collectedPoints) {
    // get corresponding positiv and negativ point
    std::shared_ptr<Point> pp_i = po_i->positivePoint;
    std::shared_ptr<Point> np_i = po_i->negativePoint;

    float distance_po = p.distPoint(*po_i) / radius;
    float distance_pp = p.distPoint(*pp_i) / radius;
    float distance_np = p.distPoint(*np_i) / radius;
    float theta_po = pow(1.0 - distance_po, 4.0) * (4.0 * distance_po + 1);
    float theta_pp = pow(1.0 - distance_pp, 4.0) * (4.0 * distance_pp + 1);
    float theta_np = pow(1.0 - distance_np, 4.0) * (4.0 * distance_np + 1);

    // set up basis vector for filling A and b
    if (this->basisPolynomDegree == 0) {
      ao << 1.0;
      ap << 1.0;
      an << 1.0;
    } else if (this->basisPolynomDegree == 1) {
      ao << 1.0, po_i->x, po_i->y, po_i->z;
      ap << 1.0, pp_i->x, pp_i->y, pp_i->z;
      an << 1.0, np_i->x, np_i->y, np_i->z;
    } else if (this->basisPolynomDegree == 2) {
      ao << 1.0, po_i->x, po_i->y, po_i->z, po_i->x * po_i->y,
          po_i->x * po_i->z, po_i->y * po_i->z, pow(po_i->x, 2.0),
          pow(po_i->y, 2.0), pow(po_i->z, 2.0);
      ap << 1.0, pp_i->x, pp_i->y, pp_i->z, pp_i->x * pp_i->y,
          pp_i->x * pp_i->z, pp_i->y * pp_i->z, pow(pp_i->x, 2.0),
          pow(pp_i->y, 2.0), pow(pp_i->z, 2.0);
      an << 1.0, np_i->x, np_i->y, np_i->z, np_i->x * np_i->y,
          np_i->x * np_i->z, np_i->y * np_i->z, pow(np_i->x, 2.0),
          pow(np_i->y, 2.0), pow(np_i->z, 2.0);
    }

    for (unsigned int j = 0; j < n; j++) {
      for (unsigned int k = 0; k < n; k++) {
        A(j, k) += ao[j] * ao[k] * theta_po;
        A(j, k) += ap[j] * ap[k] * theta_pp;
        A(j, k) += an[j] * an[k] * theta_np;
      }
      b[j] += ao[j] * theta_po * po_i->f;
      b[j] += ap[j] * theta_pp * pp_i->f;
      b[j] += an[j] * theta_np * np_i->f;
    }
  }

  MatrixXf A_inv = A.inverse();
  VectorXf X = A_inv * b;

  // insert coefficients into function
  this->implicitFunction.push_back(X);

  return true;
}

void ImplicitSurface::createAdditionalPoints() {
  Borders outerBox = kdtree->getRootnode()->borders;

  Point p1(outerBox.xMin, outerBox.yMin, outerBox.zMin);
  Point p2(outerBox.xMax, outerBox.yMax, outerBox.zMax);
  double epsilon = 0.01 * p1.distPoint(p2);

  for (std::shared_ptr<Point> p : this->points) {
    double eps = epsilon;
    while (true) {
      std::shared_ptr<Point> newPositivePoint = std::make_shared<Point>(
          p->x + eps * p->normal[0], p->y + eps * p->normal[1],
          p->z + eps * p->normal[2]);
      newPositivePoint->type = positivePoint;
      if (kdtree->isClosestPoint(*newPositivePoint, *p)) {
        newPositivePoint->f = eps;
        /* kdtree->addToTree(newPositivePoint); */
        positivePoints.push_back(newPositivePoint);
        p->positivePoint = newPositivePoint;
        positiveFunctionValues.push_back(eps);
        break;
      }
      eps /= 2;
    }
    eps = -epsilon;
    while (true) {
      std::shared_ptr<Point> newNegativePoint = std::make_shared<Point>(
          p->x + eps * p->normal[0], p->y + eps * p->normal[1],
          p->z + eps * p->normal[2]);
      newNegativePoint->type = negativePoint;
      if (kdtree->isClosestPoint(*newNegativePoint, *p)) {
        newNegativePoint->f = eps;
        /* kdtree->addToTree(newNegativePoint); */
        p->negativePoint = newNegativePoint;
        negativePoints.push_back(newNegativePoint);
        negativeFunctionValues.push_back(eps);
        break;
      }
      eps /= 2;
    }
  }
}

void ImplicitSurface::computeMarchingCubes() {
  // Clear mesh if marching cubes has been performed already
  if (!marchingCubesMesh.empty()) {
    marchingCubesMesh.clear();
  }

  // Iterate through and build grid cells to compute intersections
  std::mutex meshMutex;
#pragma omp parallel for collapse(3)
  for (glm::uint z = 0; z < gridSubdivision; z++) {
    for (glm::uint y = 0; y < gridSubdivision; y++) {
      for (glm::uint x = 0; x < gridSubdivision; x++) {
        GridCell gridCell;
        // Set points of cube
        gridCell.p[0] = *this->implicitGridPoints[x][y][z + 1];
        gridCell.val[0] = computeMLS(gridCell.p[0]);

        gridCell.p[1] = *this->implicitGridPoints[x + 1][y][z + 1];
        gridCell.val[1] = computeMLS(gridCell.p[1]);

        gridCell.p[2] = *this->implicitGridPoints[x + 1][y][z];
        gridCell.val[2] = computeMLS(gridCell.p[2]);

        gridCell.p[3] = *this->implicitGridPoints[x][y][z];
        gridCell.val[3] = computeMLS(gridCell.p[3]);

        gridCell.p[4] = *this->implicitGridPoints[x][y + 1][z + 1];
        gridCell.val[4] = computeMLS(gridCell.p[4]);

        gridCell.p[5] = *this->implicitGridPoints[x + 1][y + 1][z + 1];
        gridCell.val[5] = computeMLS(gridCell.p[5]);

        gridCell.p[6] = *this->implicitGridPoints[x + 1][y + 1][z];
        gridCell.val[6] = computeMLS(gridCell.p[6]);

        gridCell.p[7] = *this->implicitGridPoints[x][y + 1][z];
        gridCell.val[7] = computeMLS(gridCell.p[7]);

        auto partialMesh = MarchingCubes::polygonise(gridCell, 0.0);
        if (!partialMesh.empty()) {
          // Only lock resource for insertion if we actually have
          // some triangles to be added
          std::lock_guard<std::mutex> lock(meshMutex);
          marchingCubesMesh.insert(marchingCubesMesh.end(), partialMesh.begin(),
                                   partialMesh.end());
        }
      }
    }
  }
}
