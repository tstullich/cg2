#ifndef PARSER_HPP
#define PARSER_HPP

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <cmath>

#include "face.hpp"
#include "point.hpp"

class Point;

// limits of a particular cell are saved here
struct Borders {
  float xMin = std::numeric_limits<float>::max();
  float xMax = std::numeric_limits<float>::min();
  float yMin = std::numeric_limits<float>::max();
  float yMax = std::numeric_limits<float>::min();
  float zMin = std::numeric_limits<float>::max();
  float zMax = std::numeric_limits<float>::min();
};

class Parser {
public:
  /**
   * Attempts to open the file passed in through the
   * fileName parameter. The parser will assume that
   * we the given file has the OFF file format and
   * will perform some basic checks to make sure
   * that the file is valid.
   *
   * Returns a boolean to indicate if the file was opened
   * successfully or not.
   *
   */
  bool open(const std::string &fileName);

  /*
   * Parses the given file and stores the resulting face
   * and vertex vectors internally, which can then be
   * retrieved through getter methods
   */
  void parse();

  // Retrieves the parsed vertices
  std::unique_ptr<std::vector<Point>> getVertices();

  // Retrieves the faces from parser.
  std::unique_ptr<std::vector<Face>> getFaces();

  /**
   * Closes the filestream that was opened
   */
  void close();

  /**
   * Holds the characteristics of a box containing all parsed data points.
   */
  Borders outerBox;

private:
  void clearOuterBox();
  std::vector<Face> faces;
  std::vector<Point> vertices;

  std::ifstream inputStream;
};
#endif // PARSER_HPP
