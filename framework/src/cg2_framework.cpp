#include "cg2_framework.hpp"
#include <cmath>

// ----------------------------------------------------------------
float EuclDist::dist(const Point& a, const Point& b) {
  // ----------------------------------------------------------------
  float xd = a.x - b.x;
  float yd = a.y - b.y;
  float zd = a.z - b.z;
  return sqrt(xd * xd + yd * yd + zd * zd);
}

// ----------------------------------------------------------------
float EuclDist::dist(const Point& a, const Node& n) {
  // ----------------------------------------------------------------
  // return 0.0 if point is within node n
  // and the euclidian distance otherwise
  return 0.0f;

  // TODO etc...
}

// ----------------------------------------------------------------
KDTree::KDTree(std::unique_ptr<PointList> plist,
               std::unique_ptr<DistFunc> dfunc)
    : _plist(std::move(plist)), _dfunc(std::move(dfunc)) {
  // dummy datastructure (one node with references to all points)
  _rootnode = std::make_shared<Node>();

  // TODO build true spatial datastructure etc...
}

// ----------------------------------------------------------------
PointPointerList KDTree::collectKNearest(const Point& p, int knearest) {
  // ----------------------------------------------------------------

  // !NOTE: bogus implementation of k-nearest neighbor search
  PointPointerList pl;
  return pl;

  // TODO use spatial data structure for sub-linear search etc...
}

// ----------------------------------------------------------------
PointPointerList KDTree::collectInRadius(const Point& p, float radius) {
  // ----------------------------------------------------------------

  // dummy brute force implementation
  PointPointerList plist;
  return plist;

  // TODO use spatial data structure for sub-linear search etc...
}

// ----------------------------------------------------------------
int KDTree::size() {
  // ----------------------------------------------------------------
  return static_cast<int>(_plist->size());
}

// ----------------------------------------------------------------
void KDTree::draw() {
  // ----------------------------------------------------------------

  // TODO use OpenGL to draw the point data
}

// ----------------------------------------------------------------
void KDTree::draw(const PointPointerList& plist) {
  // ----------------------------------------------------------------

  // TODO use OpenGL to draw the point data
}

// ----------------------------------------------------------------
std::shared_ptr<PointList> KDTree::getPoints() {
  // ----------------------------------------------------------------
  return _plist;
}

// ----------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const KDTree& sds) {
  // ----------------------------------------------------------------
  int numelem = static_cast<int>(sds._plist->size());
  os << "num points in set: " << numelem << std::endl;
  for (int i = 0; i < numelem; i++) {
    os << "point " << i << ": " << (*sds._plist)[i].x << " "
       << (*sds._plist)[i].y << " " << (*sds._plist)[i].z << std::endl;
  }
  return os;
}

// ----------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const PointPointerList& l) {
  // ----------------------------------------------------------------
  int numelem = static_cast<int>(l.size());
  os << "num points in set: " << numelem << std::endl;
  for (int i = 0; i < numelem; i++) {
    os << "point " << i << ": " << (*l[i]).x << " " << (*l[i]).y << " "
       << (*l[i]).z << std::endl;
  }
  return os;
}
