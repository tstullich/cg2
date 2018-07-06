#include "parser.hpp"

struct Borders;

bool Parser::open(const std::string &fileName) {
  inputStream.open(fileName, std::fstream::in);
  auto found = fileName.find("off");
  if (inputStream.is_open() && found != std::string::npos) {
    std::string off;
    inputStream >> off;
    if (off == "OFF") {
      mode = OFF;
    } else if (off == "NOFF") {
      mode = NOFF;
    } else if (off == "NNOFF") {
      mode = NNOFF;
    } else {
      std::cout << "OFF file does not start with 'OFF' declaration. Invalid "
                   "file format"
                << std::endl;
      return false;
    }

    inputStream >> numPoints;
    inputStream >> numFaces;
    inputStream >> numEdges;
  } else if (inputStream.is_open() && fileName.find("obj") != std::string::npos) {
    std::cout << "Opening an obj data file" << std::endl;
  }

  return inputStream.is_open();
}

std::unique_ptr<std::vector<Point>> Parser::getPoints() {
  if (!inputStream.is_open()) {
    std::cout << "Unable to open file!" << std::endl;
    return std::make_unique<std::vector<Point>>();
  }

  // clears limits of outer box
  clearOuterBox();

  auto points = std::make_unique<std::vector<Point>>(numPoints, Point(0, 0, 0));
  float x, y, z, n0, n1, n2;
  for (int i = 0; i < numPoints; i++) {
    switch(mode) {
      case OFF:
        inputStream >> x >> y >> z;
        points->at(i).x = x;
        points->at(i).y = y;
        points->at(i).z = z;
        break;
      case NOFF:
        inputStream >> x >> y >> z >> n0 >> n1 >> n2;
        points->at(i).x = x;
        points->at(i).y = y;
        points->at(i).z = z;
        points->at(i).normal = glm::vec3(n0, n1, n2);
        break;
      case NNOFF:
        inputStream >> x >> y >> z >> n0 >> n1 >> n2;
        points->at(i).x = x;
        points->at(i).y = y;
        points->at(i).z = z;
        points->at(i).normal = -glm::vec3(n0, n1, n2);
        break;
      default:
        std::cout << "parser error" << std::endl;
        return nullptr;
    }

    outerBox.xMin = std::min(outerBox.xMin, x);
    outerBox.xMax = std::max(outerBox.xMax, x);
    outerBox.yMin = std::min(outerBox.yMin, y);
    outerBox.yMax = std::max(outerBox.yMax, y);
    outerBox.zMin = std::min(outerBox.zMin, z);
    outerBox.zMax = std::max(outerBox.zMax, z);
  }

  // make outer box slightly bigger
  float scalar = 0.05;
  float xShift = scalar * std::abs(outerBox.xMax - outerBox.xMin);
  outerBox.xMin -= xShift;
  outerBox.xMax += xShift;
  float yShift = scalar * std::abs(outerBox.yMax - outerBox.yMin);
  outerBox.yMin -= yShift;
  outerBox.yMax += yShift;
  float zShift = scalar * std::abs(outerBox.zMax - outerBox.zMin);
  outerBox.zMin -= zShift;
  outerBox.zMax += zShift;

  return points;
}

void Parser::clearOuterBox() {
  outerBox.xMin = std::numeric_limits<double>::max();
  outerBox.xMax = std::numeric_limits<double>::min();
  outerBox.yMin = std::numeric_limits<double>::max();
  outerBox.yMax = std::numeric_limits<double>::min();
  outerBox.zMin = std::numeric_limits<double>::max();
  outerBox.zMax = std::numeric_limits<double>::min();
};

void Parser::close() { inputStream.close(); }
