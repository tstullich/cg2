#include "parser.hpp"

bool Parser::open(const std::string &fileName) {
  inputStream.open(fileName, std::fstream::in);
  if (inputStream.is_open()) {
    std::string off;
    inputStream >> off;
    if (off != "OFF") {
      std::cout << "OFF file does not start with 'OFF' declaration. Invalid "
                   "file format"
                << std::endl;
      return false;
    }

    inputStream >> numVertices;
    inputStream >> numFaces;
    inputStream >> numEdges;
  }

  return inputStream.is_open();
}

std::vector<Vertex> Parser::getVertices() {
  if (!inputStream.is_open()) {
    std::cout << "Unable to open file!" << std::endl;
    return std::vector<Vertex>();
  }

  std::vector<Vertex> vertices(numVertices, Vertex(0, 0, 0));
  float x, y, z;
  for (int i = 0; i < numVertices; i++) {
    inputStream >> x >> y >> z;

    vertices[i].x = x;
    vertices[i].y = y;
    vertices[i].z = z;
  }

  return vertices;
}

std::vector<Face> Parser::getFaces() {
  if (!inputStream.is_open()) {
    std::cout << "Unable to open file!" << std::endl;
    return std::vector<Face>();
  }

  std::vector<Face> faces(numFaces);
  int vertexCount, faceIndex;
  for (int i = 0; i < numFaces; i++) {
    inputStream >> vertexCount;
    Face face;
    while (vertexCount-- > 0) {
      inputStream >> faceIndex;
      face.add(faceIndex);
    }
    faces[i] = face;
  }

  return faces;
}

void Parser::close() { inputStream.close(); }
