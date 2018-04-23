#ifndef CG2_FRAMEWORK_H
#define CG2_FRAMEWORK_H

#include <vector>
#include <iostream>

class Point;
class Node;
typedef std::vector<Point>  PointList;
typedef std::vector<Point*> PointPointerList;
typedef std::vector<Node*>  NodeList;

// ----------------------------------------------------------------
class Point { // example of a generic point in 3-space
    // ----------------------------------------------------------------
public:
    Point(float _x, float _y, float _z):
            x(_x), y(_y), z(_z) {}
    float x,y,z;
};

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
class SDS { // for 's'patial 'd'ata 's'tructure
    // ----------------------------------------------------------------
public:
    SDS(PointList* plist, DistFunc* dfunc);
    PointPointerList  collectKNearest(Point& p, int knearest);
    PointPointerList  collectInRadius(Point& p, float radius);
    void              draw();
    void              draw(const PointPointerList& plist);
    int               size();
    PointList*        getPoints();

    friend std::ostream& operator<<(std::ostream& os,
                                    const SDS& sds);

    // etc ...

private:
    PointList* _plist;    // stores the (unorganized) pointset
    Node*      _rootnode; // root (head) of spatial data structure
    DistFunc*  _dfunc;    // generic distance function

    // etc ...
};

// ----------------------------------------------------------------
std::ostream& operator << (std::ostream& os, // for debugging
                           const PointPointerList& l);
// ----------------------------------------------------------------

#endif
