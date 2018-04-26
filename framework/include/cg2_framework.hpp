#ifndef CG2_FRAMEWORK_HPP
#define CG2_FRAMEWORK_HPP

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "parser.hpp"
#include "point.hpp"

class Node;
typedef std::vector<Point> PointList;
typedef std::vector<std::shared_ptr<Point>> PointPointerList;
typedef std::vector<std::shared_ptr<Node>> NodeList;

// example of a node in spatial data structure
class Node {

public:
  PointPointerList plist; // list of pointers to points
  // contained in this node
  NodeList nlist; // list of children
};

// abstract distance function interface
class DistFunc {

public:
  virtual float dist(const Point &a, const Point &b) = 0;
  virtual float dist(const Point &a, const Node &n) = 0;
};

// euclidian distance
class EuclDist : public DistFunc {

public:
  float dist(const Point &a, const Point &b);
  float dist(const Point &a, const Node &n);
};

class KDTree {

public:
  KDTree(std::unique_ptr<PointList> plist, std::unique_ptr<DistFunc> dfunc);
  PointPointerList collectKNearest(const Point &p, int knearest);
  PointPointerList collectInRadius(const Point &p, float radius);
  void draw();
  void draw(const PointPointerList &plist);
  int size();
  std::shared_ptr<PointList> getPoints();

  friend std::ostream &operator<<(std::ostream &os, const KDTree &sds);

private:
  /**
   * Starting code to build the KDTree. It should at the end
   * return the root node of the tree so that we can use it
   * in the constructor of the tree
   */
  std::shared_ptr<Node> buildTree(int depth);

  int partitionList(const std::vector<std::shared_ptr<Point>> &pointList,
                    int leftIndex, int rightIndex, int pivot, int axis);

  /**
   * This is the final step of the Median of Medians algorithm
   * where we partition a list of 5 elements or less.
   * To accomplish this we will use Insertion sort to quickly
   * sort the small array since the algorithm is easy to implement
   * and efficient on small data sets, such as our list
   */
  int partitionListOfFive(const std::vector<std::shared_ptr<Point>> &pointList,
                          int startIndex, int endIndex, int axis);
  /**
   * This routine retrieves the Median in a given list by
   * adopting the Median of Medians algorithm used in
   * Quickselect
   */
  int selectMedian(const std::vector<std::shared_ptr<Point>> &pointList,
                   int leftIndex, int rightIndex, int n, int axis);

  /**
   * This is a helper function for the selectMedian functiion.
   */
  int selectMedianOfMedians(
      const std::vector<std::shared_ptr<Point>> &pointList, int leftIndex,
      int rightIndex, int axis);

  /**
   * Function that will sort our list based on a given dimension.
   * The values should be as follows:
   * 0 - Sort for the X axis
   * 1 - Sort for the Y axis
   * 2 - Sort for the Z axis
   *
   * The algorithm we will use to quickly sort over a median
   * is Quickselect, which will use the Median of Medians algorithm
   * to select a suitable median
   */
  void sortPoints(const std::vector<std::shared_ptr<Point>> &pointList,
                  int axis);

  // Swaps two elements in a list based on two indices
  // No bounds-checking is done to see if the if indices are valid
  void swapPoints(const std::vector<std::shared_ptr<Point>> &pointList,
                  int leftIndex, int rightIndex);

  // stores the (unorganized) pointset
  std::shared_ptr<PointList> plist;

  // root (head) of spatial data structure
  std::shared_ptr<Node> rootnode;

  // generic distance function
  std::unique_ptr<DistFunc> dfunc;

  // How deep we want to make our recursion for the tree
  const int finalDepth = 8;

  // The number of dimensions for the KD Tree.
  const int kDimension = 3;

  // Sets the maximum size of a partition.
  // 5 was the suggested size on wikipedia but we may be able to change this
  const int partitionSize = 5;
};

std::ostream &operator<<(std::ostream &os, // for debugging
                         const PointPointerList &l);
#endif // CG2_FRAMEWORK_HPP
