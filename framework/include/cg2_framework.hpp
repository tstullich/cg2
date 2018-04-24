#ifndef CG2_FRAMEWORK_H
#define CG2_FRAMEWORK_H

#include <memory>
#include <vector>
#include <iostream>

#include "parser.hpp"
#include "point.hpp"

class Node;
typedef std::vector<Point>  PointList;
typedef std::vector<std::shared_ptr<Point>> PointPointerList;
typedef std::vector<std::shared_ptr<Node>>  NodeList;

// ----------------------------------------------------------------
class Node { // example of a node in spatial data structure
    // ----------------------------------------------------------------
public:
    PointPointerList plist; // list of pointers to points
    // contained in this node
    NodeList         nlist; // list of children
};

// ----------------------------------------------------------------
class DistFunc { // abstract distance function interface
    // ----------------------------------------------------------------
public:
    virtual ~DistFunc() {}
    virtual float dist(Point& a, Point& b) = 0;
    virtual float dist(Point& a, Node& n)  = 0;
};

// ----------------------------------------------------------------
class EuclDist : public DistFunc { // euclidian distance
    // ----------------------------------------------------------------
public:
    float dist(Point& a, Point& b);
    float dist(Point& a, Node& n);
};

// ----------------------------------------------------------------
class KDTree {
    // ----------------------------------------------------------------
public:
    KDTree(std::shared_ptr<PointList> plist, std::shared_ptr<DistFunc> dfunc);
    PointPointerList  collectKNearest(Point& p, int knearest);
    PointPointerList  collectInRadius(Point& p, float radius);
    void              draw();
    void              draw(const PointPointerList& plist);
    int               size();
    std::shared_ptr<PointList>        getPoints();

    friend std::ostream& operator<<(std::ostream& os,
                                    const KDTree& sds);

    // etc ...

private:
    std::shared_ptr<PointList> _plist;    // stores the (unorganized) pointset
    std::shared_ptr<Node>      _rootnode; // root (head) of spatial data structure
    std::shared_ptr<DistFunc>  _dfunc;    // generic distance function
};

// ----------------------------------------------------------------
std::ostream& operator << (std::ostream& os, // for debugging
                           const PointPointerList& l);
// ----------------------------------------------------------------

#endif
