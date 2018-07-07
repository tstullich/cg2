#include "mesh.hpp"

Mesh::Mesh(std::vector<Point> _vertices, std::vector<Face> _faces) :
    vertices(std::move(_vertices)), faces(std::move(_faces)) {
  assert(vertices.size() > 0);
  // calculate center of vertex cloud
  for (glm::uint i = 0; i < vertices.size(); ++i) {
    center += vertices[i].toVec3();
  }
  // determine lagest distance from center to vertex
  center /= vertices.size();
  for (glm::uint i = 0; i < vertices.size(); ++i) {
    float tmp = glm::length(vertices[i].toVec3() - center);
    boundingRadius = (tmp > boundingRadius)? tmp : boundingRadius;
  }
  // add adjacent vertex and face ids to vertices and compute face normals and face area
  for (glm::uint i = 0; i < faces.size(); ++i) {
    for (glm::uint j = 0; j < faces[i].numVertices(); ++j) {
      // add adjacent faces
      vertices[faces[i][j]].adjacentFaces.push_back(i);

      // add adjacent vertices
      if(j == 0)
        vertices[faces[i][j]].addVertex(faces[i][faces[i].numVertices()-1]);
      else
        vertices[faces[i][j]].addVertex(faces[i][j-1]);
      if(j == faces[i].numVertices()-1)
        vertices[faces[i][j]].addVertex(faces[i][0]);
      else
        vertices[faces[i][j]].addVertex(faces[i][j+1]);
    }

    if(faces[i].numVertices() == 3) {
      // compute face normals and face area
      faces[i].normal = glm::cross(vertices[faces[i][1]].toVec3() - vertices[faces[i][0]].toVec3(),
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
    v /= vertices[i].adjacentFaces.size();
    vertices[i].normal = v;
  }
}

void Mesh::computeUniformLaplacian(unsigned int numEigenVectors) {
  //TODO
  const unsigned int n = vertices.size();
  SparseMatrix<double> M(n, n);
  SparseMatrix<double> D(n, n);
  for(unsigned int i = 0; i < n; i++) {
    M.insert(i,i) = -1.0;
    for(unsigned int j = 0; j < vertices[i].adjacentVertices.size(); j++) {
      M.insert(i, vertices[i].adjacentVertices[j]) = 1.0/float(vertices[i].adjacentVertices.size());
    }
    D.insert(i,i) = 1.0 / (2.0 * getSurroundingArea(vertices[i]));
  }
  SparseMatrix<double> L(n, n);
  L = D * M;
  //std::cout << L << std::endl;

  SparseSymMatProd<double> op(L);
  // choose smallest eigenvalues
  SymEigsSolver<double, SMALLEST_ALGE, SparseSymMatProd<double>> eigs(&op, numEigenVectors,
      2*numEigenVectors);
  eigs.init();
  eigs.compute();
  VectorXd eigenValues;
  MatrixXd eigenVectors;
  if(eigs.info() == SUCCESSFUL) {
    eigenValues = eigs.eigenvalues();
    eigenVectors = eigs.eigenvectors();
  }

  std::cout << "Eigenvalues found:\n" << eigenValues << std::endl;
  //std::cout << "Eigenvectors found:\n" << eigenVectors.col(0) << std::endl;

  // fill a matrix with original vertices
  MatrixXd originalVertices(n, 3);
  for(unsigned int i = 0; i < vertices.size(); i++) {
    originalVertices(i, 0) = vertices[i].x;
    originalVertices(i, 1) = vertices[i].y;
    originalVertices(i, 2) = vertices[i].z;
  }

  VectorXd xValues = VectorXd::Zero(n);
  VectorXd yValues = VectorXd::Zero(n);
  VectorXd zValues = VectorXd::Zero(n);
  // reconstruct mesh
  for(unsigned int i = 0; i < eigenVectors.cols(); i++) {
    xValues += originalVertices.col(0).dot(eigenVectors.col(i)) * eigenVectors.col(i);
    yValues += originalVertices.col(1).dot(eigenVectors.col(i)) * eigenVectors.col(i);
    zValues += originalVertices.col(2).dot(eigenVectors.col(i)) * eigenVectors.col(i);
  }
  // component vectors to vertex
  verticesUniformLaplacian.clear();
  for(unsigned int i = 0; i < vertices.size(); i++) {
    verticesUniformLaplacian.push_back(Point(xValues(i), yValues(i), zValues(i)));
  }
}

float Mesh::getSurroundingArea(Point &p) {
  float area = 0.0;
  for(unsigned int i = 0; i < p.adjacentFaces.size(); i++)
    area += faces[p.adjacentFaces[i]].area;
  // TODO mean?
  return area; // /float(p.adjacentFaces.size());
}

glm::vec3 Mesh::getCenter() {
  return center;
}

float Mesh::getBoundingRadius() {
  return boundingRadius;
}

std::vector<Point> Mesh::getVertices() {
  return vertices;
}

std::vector<Face> Mesh::getFaces() {
  return faces;
}
