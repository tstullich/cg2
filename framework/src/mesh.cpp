#include "mesh.hpp"

Mesh::Mesh(std::vector<Point> _vertices, std::vector<Face> _faces)
    : vertices(std::move(_vertices)), faces(std::move(_faces)) {
  assert(vertices.size() > 0);
  // calculate center of vertex cloud
  for (glm::uint i = 0; i < vertices.size(); ++i) {
    center += vertices[i].toVec3();
  }
  // determine lagest distance from center to vertex
  center /= vertices.size();
  for (glm::uint i = 0; i < vertices.size(); ++i) {
    float tmp = glm::length(vertices[i].toVec3() - center);
    boundingRadius = (tmp > boundingRadius) ? tmp : boundingRadius;
  }
  // add adjacent vertex and face ids to vertices and compute face normals and
  // face area
  for (glm::uint i = 0; i < faces.size(); ++i) {
    for (glm::uint j = 0; j < faces[i].numVertices(); ++j) {
      // add adjacent faces
      vertices[faces[i][j]].adjacentFaces.push_back(i);

      // add adjacent vertices
      if (j == 0)
        vertices[faces[i][j]].addVertex(faces[i][faces[i].numVertices() - 1]);
      else
        vertices[faces[i][j]].addVertex(faces[i][j - 1]);
      if (j == faces[i].numVertices() - 1)
        vertices[faces[i][j]].addVertex(faces[i][0]);
      else
        vertices[faces[i][j]].addVertex(faces[i][j + 1]);
    }

    if (faces[i].numVertices() == 3) {
      // compute face normals and face area
      faces[i].normal = glm::cross(
          vertices[faces[i][1]].toVec3() - vertices[faces[i][0]].toVec3(),
          vertices[faces[i][2]].toVec3() - vertices[faces[i][0]].toVec3());
      // area is half of the magnitude of the crossproduct
      faces[i].area = glm::length(faces[i].normal) / 2.0;
      faces[i].normal /= glm::length(faces[i].normal);
    }
  }
}

void Mesh::computeUnweightedNormals() {
  for (glm::uint i = 0; i < vertices.size(); ++i) {
    glm::vec3 v(0.0, 0.0, 0.0);
    for (glm::uint j = 0; j < vertices[i].adjacentFaces.size(); ++j) {
      v += faces[vertices[i].adjacentFaces[j]].normal;
    }
    vertices[i].normal = glm::normalize(v);
  }
}

glm::vec3 Mesh::computeAreaNormal(Face face) {
  Point p1 = vertices[face[0]];
  Point p2 = vertices[face[1]];
  Point p3 = vertices[face[2]];
  glm::vec3 v1 = p2.toVec3() - p1.toVec3();
  glm::vec3 v2 = p3.toVec3() - p1.toVec3();
  glm::vec3 nv2 = glm::normalize(v2);

  // project v1 onto v2
  glm::vec3 vP = glm::dot(v1, nv2) * nv2;

  // triangle height and base side length
  float h = glm::length(v1 - vP);
  float b = glm::length(v2);

  float area = 0.5 * h * b;

  return face.normal * area;
}

void Mesh::computeWeightedNormals() {
  for (glm::uint i = 0; i < vertices.size(); ++i) {
    glm::vec3 newNormal(0.0, 0.0, 0.0);
    for (glm::uint j = 0; j < vertices[i].adjacentFaces.size(); ++j) {
      newNormal += computeAreaNormal(faces[vertices[i].adjacentFaces[j]]);
    }
    vertices[i].weightedNormal = glm::normalize(newNormal);
  }
}

SparseMatrix<double> Mesh::computeLMatrix(glm::uint n) {
  SparseMatrix<double> M(n, n);
  SparseMatrix<double> D(n, n);
  for (unsigned int i = 0; i < n; i++) {
    M.insert(i, i) = -1.0;
    for (unsigned int j = 0; j < vertices[i].adjacentVertices.size(); j++) {
      M.insert(i, vertices[i].adjacentVertices[j]) =
          1.0 / float(vertices[i].adjacentVertices.size());
    }
    D.insert(i, i) = 1.0 / (2.0 * getSurroundingArea(vertices[i]));
  }
  return D * M;
}

void Mesh::computeUniformLaplacian(unsigned int numEigenVectors) {
  const unsigned int n = vertices.size();
  auto L = computeLMatrix(n);
  // std::cout << L << std::endl;

  SparseSymMatProd<double> op(L);
  // choose smallest eigenvalues
  SymEigsSolver<double, SMALLEST_ALGE, SparseSymMatProd<double>> eigs(
      &op, numEigenVectors, 2 * numEigenVectors);
  eigs.init();
  eigs.compute();
  VectorXd eigenValues;
  MatrixXd eigenVectors;
  if (eigs.info() == SUCCESSFUL) {
    eigenValues = eigs.eigenvalues();
    eigenVectors = eigs.eigenvectors();
  }

  std::cout << "Eigenvalues found:\n" << eigenValues << std::endl;
  // std::cout << "Eigenvectors found:\n" << eigenVectors.col(0) << std::endl;

  // fill a matrix with original vertices
  MatrixXd originalVertices(n, 3);
  for (unsigned int i = 0; i < vertices.size(); i++) {
    originalVertices(i, 0) = vertices[i].x;
    originalVertices(i, 1) = vertices[i].y;
    originalVertices(i, 2) = vertices[i].z;
  }

  VectorXd xValues = VectorXd::Zero(n);
  VectorXd yValues = VectorXd::Zero(n);
  VectorXd zValues = VectorXd::Zero(n);
  // reconstruct mesh
  for (unsigned int i = 0; i < eigenVectors.cols(); i++) {
    xValues +=
        originalVertices.col(0).dot(eigenVectors.col(i)) * eigenVectors.col(i);
    yValues +=
        originalVertices.col(1).dot(eigenVectors.col(i)) * eigenVectors.col(i);
    zValues +=
        originalVertices.col(2).dot(eigenVectors.col(i)) * eigenVectors.col(i);
  }
  // component vectors to vertex
  verticesUniformLaplacian.clear();
  for (unsigned int i = 0; i < vertices.size(); i++) {
    verticesUniformLaplacian.push_back(
        Point(xValues(i), yValues(i), zValues(i)));
  }
}

void Mesh::computeExplicitCotan(double stepSize, glm::uint basisFunctions) {
  const glm::uint n = vertices.size();
  MatrixXd points(n, 3);
  if (verticesExplicitLaplace.empty()) {
    // Setup points matrix, but only if we haven't
    // run through a step of the integration already
    verticesExplicitLaplace = std::vector<Point>(n, Point(0.0, 0.0, 0.0));
    for (glm::uint i = 0; i < n; i++) {
      points(i, 0) = vertices[i].x;
      points(i, 1) = vertices[i].y;
      points(i, 2) = vertices[i].z;
    }
  } else {
    // Build matrix by using previous integration result
    for (glm::uint i = 0; i < n; i++) {
      points(i, 0) = verticesExplicitLaplace[i].x;
      points(i, 1) = verticesExplicitLaplace[i].y;
      points(i, 2) = verticesExplicitLaplace[i].z;
    }
  }

  // Setup L matrix
  auto laplacian = computeLMatrix(n);
  // Setup Identity matrix
  auto identity = MatrixXd::Identity(n, n);

  // Perform integratrion
  auto explicitResult = (identity + stepSize * laplacian) * points;

  // Create result mesh
  for (glm::uint i = 0; i < n; i++) {
    auto pointRow = explicitResult.row(i);
    auto p = verticesExplicitLaplace[i];
    p.x = pointRow[0];
    p.y = pointRow[1];
    p.z = pointRow[2];
    verticesExplicitLaplace[i] = p;
  }
}

void Mesh::computeImplicitCotan(double stepSize, glm::uint basisFunctions) {
  const glm::uint n = vertices.size();
  SparseMatrix<double> points(n, 3);
  if (verticesImplicitLaplace.empty()) {
    verticesImplicitLaplace = std::vector<Point>(n, Point(0.0, 0.0, 0.0));
    for (glm::uint i = 0; i < n; i++) {
      points.insert(i, 0) = vertices[i].x;
      points.insert(i, 1) = vertices[i].y;
      points.insert(i, 2) = vertices[i].z;
    }
  } else {
    // Build matrix by using previous integration result
    for (glm::uint i = 0; i < n; i++) {
      points.insert(i, 0) = verticesImplicitLaplace[i].x;
      points.insert(i, 1) = verticesImplicitLaplace[i].y;
      points.insert(i, 2) = verticesImplicitLaplace[i].z;
    }
  }

  // Setup of our matrices
  auto laplacian = computeLMatrix(n);
  auto identity = MatrixXd::Identity(n, n);

  // Calculate LHS of linear system
  auto inner = identity - stepSize * laplacian;

  // Apply sparse Cholesky solver
  Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> chol(inner);
  // Use decomp to solve with our point matrix
  MatrixXd choleskyResult = chol.solve(points);

  // Create result mesh
  for (glm::uint i = 0; i < n; i++) {
    auto pointRow = choleskyResult.row(i);
    auto p = verticesImplicitLaplace[i];
    p.x = pointRow[0];
    p.y = pointRow[1];
    p.z = pointRow[2];
    verticesImplicitLaplace[i] = p;
  }
}


void Mesh::resetCotanLaplace() {
  verticesExplicitLaplace.clear();
  verticesImplicitLaplace.clear();
}

float Mesh::getSurroundingArea(Point &p) {
  float area = 0.0;
  for (unsigned int i = 0; i < p.adjacentFaces.size(); i++)
    area += faces[p.adjacentFaces[i]].area;
  // TODO mean?
  return area;  // /float(p.adjacentFaces.size());
}

glm::vec3 Mesh::getCenter() { return center; }

float Mesh::getBoundingRadius() { return boundingRadius; }

std::vector<Point> Mesh::getVertices() { return vertices; }

std::vector<Face> Mesh::getFaces() { return faces; }
