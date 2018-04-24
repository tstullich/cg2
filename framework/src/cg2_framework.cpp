#include "cg2_framework.h"

#include <cmath>

// ----------------------------------------------------------------
float EuclDist::dist(Point& a, Point& b) {
  // ----------------------------------------------------------------
  float xd = a.x - b.x;
  float yd = a.y - b.y;
  float zd = a.z - b.z;
  return sqrt(xd * xd + yd * yd + zd * zd);
}

// ----------------------------------------------------------------
float EuclDist::dist(Point& a, Node& n) {
  // ----------------------------------------------------------------
  // return 0.0 if point is within node n
  // and the euclidian distance otherwise
  return 0.0f;

  // TODO etc...
}

// ----------------------------------------------------------------
KDTree::KDTree(std::shared_ptr<PointList> plist,
               std::shared_ptr<DistFunc> dfunc)
    : _plist(plist), _dfunc(dfunc) {
  // dummy datastructure (one node with references to all points)
  _rootnode = std::make_shared<Node>();
  for (unsigned int i = 0; i < _plist->size(); i++) {
    _rootnode->plist.push_back(&(*_plist)[i]);
  }

  // TODO build true spatial datastructure etc...
}

// ----------------------------------------------------------------
PointPointerList KDTree::collectKNearest(Point& p, int knearest) {
  // ----------------------------------------------------------------

  // !NOTE: bogus implementation of k-nearest neighbor search
  PointPointerList pl;
  int i = 0, numelem = static_cast<int>(_rootnode->plist.size());
  while (i < knearest && i < numelem) {
    pl.push_back(_rootnode->plist[i]);
    i++;
  }
  return pl;

  // TODO use spatial data structure for sub-linear search etc...
}

// ----------------------------------------------------------------
PointPointerList KDTree::collectInRadius(Point& p, float radius) {
  // ----------------------------------------------------------------

  // dummy brute force implementation
  PointPointerList plist;
  for (int i = 0; i < static_cast<int>(_plist->size()); i++) {
    if (_dfunc->dist(p, (*_plist)[i]) < radius) {
      plist.push_back(&(*_plist)[i]);
    }
  }
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
