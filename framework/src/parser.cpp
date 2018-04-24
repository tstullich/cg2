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

std::unique_ptr<std::vector<Point>> Parser::getPoints() {
  if (!inputStream.is_open()) {
    std::cout << "Unable to open file!" << std::endl;
    return std::make_unique<std::vector<Point>>();
  }

  auto points = std::make_unique<std::vector<Point>>(numPoints, Point(0, 0, 0));
  float x, y, z;
  for (int i = 0; i < numPoints; i++) {
    inputStream >> x >> y >> z;

    points->at(i).x = x;
    points->at(i).y = y;
    points->at(i).z = z;
  }

  return points;
}

std::unique_ptr<std::vector<Face>> Parser::getFaces() {
  if (!inputStream.is_open()) {
    std::cout << "Unable to open file!" << std::endl;
    return std::make_unique<std::vector<Face>>();
  }

  auto faces = std::make_unique<std::vector<Face>>(numFaces);
  int vertexCount, faceIndex;
  for (int i = 0; i < numFaces; i++) {
    inputStream >> vertexCount;
    Face face;
    while (vertexCount-- > 0) {
      inputStream >> faceIndex;
      face.add(faceIndex);
    }
    faces->at(i) = face;
  }

  return faces;
}

void Parser::close() { inputStream.close(); }
