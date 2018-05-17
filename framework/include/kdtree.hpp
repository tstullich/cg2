#ifndef CG2_FRAMEWORK_HPP
#define CG2_FRAMEWORK_HPP

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include <queue>

#include "parser.hpp"
#include "point.hpp"

typedef std::vector<std::shared_ptr<Point>> PointPointerList;
typedef std::vector<Point> PointList;
typedef std::vector<std::shared_ptr<Node>> NodeList;

// sort operator of priority queue
struct LessThanByDistance {
  bool operator()(const std::shared_ptr<Point> lhs, const std::shared_ptr<Point> rhs) {
    return lhs->dist < rhs->dist;
  }
};
typedef std::priority_queue<std::shared_ptr<Point>, PointPointerList, LessThanByDistance> PriorityQueue;

// characteristics of the split plane
struct Split {
  unsigned int axis;
  float value;
};

// Our node class that holds information for the K-d tree
class Node {
public:
  std::shared_ptr<Node> parent = nullptr; // parent node
  PointPointerList plist;                 // list of pointers to points
  // contained in this node
  NodeList nlist; // list of children
  Borders borders;
  Split split;
};

class KDTree {

public:
  KDTree(std::unique_ptr<PointList> plist, Borders outerBox);

  PointPointerList collectInRadiusSimple(const Point &p, float radius);
  PointPointerList collectInRadius(const Point &p, float radius);
  void recursiveLeafSearch(const Point &p, float radius, const Node &n, PointPointerList &plist);

  PointPointerList collectKNearestSimple(const Point &p, int knearest);
  PointPointerList collectKNearest(const Point &p, int knearest);
  void recursiveKNearestSearch(const Point &p, int k, std::shared_ptr<Node> &n,
      NodeList &nl, PriorityQueue &pq);
  void add(int k, const Point &p, PriorityQueue &pq, PointPointerList &pl);

  int size();
  std::shared_ptr<PointList> getPoints();
  std::shared_ptr<Node> getRootnode();

  friend std::ostream &operator<<(std::ostream &os, const KDTree &sds);

private:
  /**
   * Builds a KDTree from plist and saves the root to rootnode.
   */
  bool buildTree(Borders &outerBox);

  /**
   * Recursively creates new children to KDTree nodes.
   */
  void recursiveTreeExtend(unsigned int depth, std::shared_ptr<Node> node);

  /**
   * Splits the given list at given index into two lists.
   */
  std::vector<PointPointerList> splitList(PointPointerList &list,
                                          unsigned int index);

  int partitionList(const std::vector<std::shared_ptr<Point>> &pointList,
                    int leftIndex, int rightIndex, int pivot, int axis);

  /**
   * This is the final step of the Median of Medians algorithm
   * where we partition a list of 5 elements or less.
   * To accomplish this we will use Insertion sort to quickly
   * sort the small array since the algorithm is easy to implement
   * and efficient on small data sets, such as our list
   */
  int sortListOfFive(const std::vector<std::shared_ptr<Point>> &pointList,
                          int startIndex, int endIndex, int axis);
  /**
   * This routine retrieves the Median in a given list by
   * adopting the Median of Medians algorithm used in
   * Quickselect
   */
  int selectMedian(const std::vector<std::shared_ptr<Point>> &pointList,
                   int leftIndex, int rightIndex, int positionOfMedian,
                   int axis);

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
  int sortPoints(const std::vector<std::shared_ptr<Point>> &pointList,
                 int axis);

  // Swaps two elements in a list based on two indices
  // No bounds-checking is done to see if the if indices are valid
  void swapPoints(const std::vector<std::shared_ptr<Point>> &pointList,
                  int leftIndex, int rightIndex);

  // stores the (unorganized) pointset
  std::shared_ptr<PointList> plist;

  // root (head) of spatial data structure
  std::shared_ptr<Node> rootnode;

  // How deep we want to make our recursion for the tree
  const unsigned int maxDepth = 8;

  // How many points are enough to not split further?
  const unsigned int maxPoints = 50;

  // The number of dimensions for the KD Tree.
  const int kDimension = 3;

  // Sets the maximum size of a partition.
  // 5 was the suggested size on wikipedia but we may be able to change this
  const int partitionSize = 5;
};

std::ostream &operator<<(std::ostream &os, // for debugging
                         const PointPointerList &l);
#endif // CG2_FRAMEWORK_HPP
