#ifndef PARSER_HPP
#define PARSER_HPP

#include <fstream>
#include <iostream>
#include <memory>
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

  /**
   * Parses the number of vertices contained in the OFF file
   * Currently keeping this in a vector until we can figure
   * out what else we need to do with the data
   */
  std::unique_ptr<std::vector<Point>> getPoints();

  /**
   * Parses the number of faces contained in the OFF file
   * Currently keeping this in a vector until we can figure
   * out what else we need to do with the data
   *
   * !!! This funtion needs to be called only after getVertices
   *     is called. Otherwise we get unexpected behavior !!!
   */
  std::unique_ptr<std::vector<Face>> getFaces();

  // Not sure if we need this but I am keeping this stubeed for now
  std::vector<int> getEdges();

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

  std::ifstream inputStream;
  int numPoints;
  int numFaces;
  int numEdges;
};
#endif // PARSER_HPP
