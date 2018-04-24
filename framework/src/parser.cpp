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

    inputStream >> numPoints;
    inputStream >> numFaces;
    inputStream >> numEdges;
  }

  return inputStream.is_open();
}

std::vector<Point> Parser::getPoints() {
  if (!inputStream.is_open()) {
    std::cout << "Unable to open file!" << std::endl;
    return std::vector<Point>();
  }

  std::vector<Point> points(numPoints, Point(0, 0, 0));
  float x, y, z;
  for (int i = 0; i < numPoints; i++) {
    inputStream >> x >> y >> z;

    points[i].x = x;
    points[i].y = y;
    points[i].z = z;
  }

  return points;
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
