#include "parser.hpp"

struct Borders;

bool Parser::open(const std::string &fileName) {
  inputStream.open(fileName, std::fstream::in);
  if (!inputStream.is_open() || fileName.find("obj") == std::string::npos) {
      std::cout << "Invalid file selected or unable to open file" << std::endl;
  }

  return inputStream.is_open();
}

void Parser::parse() {
  if (!inputStream.is_open()) {
    std::cout << "Unable to open file. Did you call close() before calling parse?" << std::endl;
    return;
  }

  // Clear out old data in case we open a new file
  if (!faces.empty() || !vertices.empty()) {
    faces.clear();
    vertices.clear();
  }

  std::string line;
  while(std::getline(inputStream, line)) {
    std::istringstream lineStream(line);
    std::string type;
    lineStream >> type;
    if (type == "v") {
      float x, y, z;
      lineStream >> x >> y >> z;
      vertices.emplace_back(Point(x, y, z));
    } else if (type == "f") {
      std::string v0, v1, v2;
      lineStream >> v0 >> v1 >> v2;
      if (v0.find_first_of('/') != std::string::npos) {
        // If there is a slash '/' contained in our token we just want to value
        // before the slash, which should be the regular vertex index
        auto vertIdx0 = std::stoul(v0.substr(0, v0.find_first_of('/'))) - 1;
        auto vertIdx1 = std::stoul(v1.substr(0, v1.find_first_of('/'))) - 1;
        auto vertIdx2 = std::stoul(v2.substr(0, v2.find_first_of('/'))) - 1;
        faces.emplace_back(Face(vertIdx0, vertIdx1, vertIdx2));
      } else {
        // No slash so we can stuff the values in the vector without issues... hopefully
        faces.emplace_back(Face(std::stoul(v0) - 1, std::stoul(v1) - 1, std::stoul(v2) - 1));
      }
    }
  }
}

// Wraps our parsed faces into a unique pointer
std::vector<Face> Parser::getFaces() {
  return faces;
}

// Wraps our parsed vertices into a unique pointer
std::vector<Point> Parser::getVertices() {
  return vertices;
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
