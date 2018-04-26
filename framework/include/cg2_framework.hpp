#ifndef CG2_FRAMEWORK_HPP
#define CG2_FRAMEWORK_HPP

#include <algorithm>
#include <iostream>
#include <memory>
#include <queue>
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

  /**
   * Function that will sort our list based on a given dimension.
   * The values should be as follows:
   * 0 - Sort for the X axis
   * 1 - Sort for the Y axis
   * 2 - Sort for the Z axis
   *
   */
  void sortPoints(int axis);

  std::shared_ptr<PointList> plist; // stores the (unorganized) pointset
  std::shared_ptr<Node> rootnode;   // root (head) of spatial data structure
  std::unique_ptr<DistFunc> dfunc;  // generic distance function

  // How deep we want to make our recursion for the tree
  const int finalDepth = 8;

  // The number of dimensions for the KD Tree.
  const int kDimension = 3;
};

std::ostream &operator<<(std::ostream &os, // for debugging
                         const PointPointerList &l);
#endif // CG2_FRAMEWORK_HPP
