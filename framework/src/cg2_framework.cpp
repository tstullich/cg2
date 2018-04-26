#include "cg2_framework.hpp"
#include <cmath>

float EuclDist::dist(const Point& a, const Point& b) {
  float xd = a.x - b.x;
  float yd = a.y - b.y;
  float zd = a.z - b.z;
  return sqrt(xd * xd + yd * yd + zd * zd);
}

float EuclDist::dist(const Point& a, const Node& n) {
  // return 0.0 if point is within node n
  // and the euclidian distance otherwise
  return 0.0f;
}

// TODO build true spatial datastructure etc...
KDTree::KDTree(std::unique_ptr<PointList> plist,
               std::unique_ptr<DistFunc> dfunc)
    : plist(std::move(plist)), dfunc(std::move(dfunc)) {
  // dummy datastructure (one node with references to all points)
  rootnode = buildTree(0);
}

// TODO use spatial data structure for sub-linear search etc...
PointPointerList KDTree::collectKNearest(const Point& p, int knearest) {
  auto closestPoints = std::priority_queue<Point>();
  PointPointerList pl;
  return pl;
}

// TODO use spatial data structure for sub-linear search etc...
PointPointerList KDTree::collectInRadius(const Point& p, float radius) {
  PointPointerList plist;
  return plist;
}

int KDTree::size() { return static_cast<int>(plist->size()); }

// TODO use OpenGL to draw the point data
void KDTree::draw() {}

// TODO use OpenGL to draw the point data
void KDTree::draw(const PointPointerList& plist) {}

std::shared_ptr<PointList> KDTree::getPoints() { return plist; }

std::ostream& operator<<(std::ostream& os, const KDTree& sds) {
  int numelem = static_cast<int>(sds.plist->size());
  os << "num points in set: " << numelem << std::endl;
  for (int i = 0; i < numelem; i++) {
    os << "point " << i << ": " << (*sds.plist)[i].x << " " << (*sds.plist)[i].y
       << " " << (*sds.plist)[i].z << std::endl;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const PointPointerList& l) {
  int numelem = static_cast<int>(l.size());
  os << "num points in set: " << numelem << std::endl;
  for (int i = 0; i < numelem; i++) {
    os << "point " << i << ": " << (*l[i]).x << " " << (*l[i]).y << " "
       << (*l[i]).z << std::endl;
  }
  return os;
}

std::shared_ptr<Node> KDTree::buildTree(int depth) {
  if (depth == finalDepth) {
    // Need to return here since we have reached our final depth
  }

  int axis = depth % kDimension;

  sortPoints(axis);

  return std::make_shared<Node>();
}

void KDTree::sortPoints(int dimension) {
  if (dimension < 0 || dimension > 2) {
    // Invalid index given. Won't sort
    return;
  }
}
