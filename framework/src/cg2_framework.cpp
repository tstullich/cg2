#include "cg2_framework.hpp"
#include <cmath>

float EuclDist::dist(const Point &a, const Point &b) {
  float xd = a.x - b.x;
  float yd = a.y - b.y;
  float zd = a.z - b.z;
  return sqrt(xd * xd + yd * yd + zd * zd);
}

float EuclDist::dist(const Point &a, const Node &n) {
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
PointPointerList KDTree::collectKNearest(const Point &p, int knearest) {
  PointPointerList pl;
  return pl;
}

// TODO use spatial data structure for sub-linear search etc...
PointPointerList KDTree::collectInRadius(const Point &p, float radius) {
  PointPointerList plist;
  return plist;
}

int KDTree::size() { return static_cast<int>(plist->size()); }

// TODO use OpenGL to draw the point data
void KDTree::draw() {}

// TODO use OpenGL to draw the point data
void KDTree::draw(const PointPointerList &plist) {}

std::shared_ptr<PointList> KDTree::getPoints() { return plist; }

std::ostream &operator<<(std::ostream &os, const KDTree &sds) {
  int numelem = static_cast<int>(sds.plist->size());
  os << "num points in set: " << numelem << std::endl;
  for (int i = 0; i < numelem; i++) {
    os << "point " << i << ": " << (*sds.plist)[i].x << " " << (*sds.plist)[i].y
       << " " << (*sds.plist)[i].z << std::endl;
  }
  return os;
}

std::ostream &operator<<(std::ostream &os, const PointPointerList &l) {
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

  // TODO Do a sort as proof
  // int axis = depth % kDimension;
  // sortPoints(axis);

  return std::make_shared<Node>();
}

int KDTree::partitionList(const std::vector<std::shared_ptr<Point>> &pointList,
                          int leftIndex, int rightIndex, int pivotIndex,
                          int axis) {
  auto pivotValue = pointList[pivotIndex]->fetchPointValue(axis);
  swapPoints(pointList, pivotIndex, rightIndex);

  auto storeIndex = leftIndex;
  for (int i = leftIndex; i < rightIndex - 1; i++) {
    if (pointList[i]->fetchPointValue(axis) < pivotValue) {
      swapPoints(pointList, storeIndex, i);
      storeIndex++;
    }
  }

  swapPoints(pointList, rightIndex, storeIndex);

  return storeIndex;
}

int KDTree::partitionListOfFive(
    const std::vector<std::shared_ptr<Point>> &pointList, int startIndex,
    int endIndex, int axis) {
  for (int i = startIndex; i < endIndex; i++) {
    int j = i;
    auto currentValue = pointList[j]->fetchPointValue(axis);
    auto nextValue = pointList[j - 1]->fetchPointValue(axis);
    while (j > 0 && nextValue > currentValue) {
      swapPoints(pointList, j, j - 1);
      j--;
    }
  }

  // Our median should now be in the middle of our array
  // TODO Verify this though
  return std::floor((endIndex + startIndex) / 2);
}

int KDTree::selectMedian(const std::vector<std::shared_ptr<Point>> &pointList,
                         int leftIndex, int rightIndex, int n, int axis) {
  // According to wikipedia you are supposed to loop this.
  // Need to find a better way to get this to work
  while (true) {
    if (leftIndex == rightIndex) {
      return leftIndex;
    }

    auto pivotIndex =
        selectMedianOfMedians(pointList, leftIndex, rightIndex, axis);
    pivotIndex =
        partitionList(pointList, leftIndex, rightIndex, pivotIndex, axis);
    if (n == pivotIndex) {
      return n;
    } else if (n < pivotIndex) {
      rightIndex = pivotIndex - 1;
    } else {
      leftIndex = pivotIndex + 1;
    }
  }
}

int KDTree::selectMedianOfMedians(
    const std::vector<std::shared_ptr<Point>> &pointList, int leftIndex,
    int rightIndex, int axis) {
  if (rightIndex - leftIndex < partitionSize) {
    // List size is smaller than our partition size so we just retrieve the
    // median
    partitionListOfFive(pointList, leftIndex, rightIndex, axis);
    return (leftIndex + rightIndex) / 2;
  }

  for (int i = leftIndex; i < rightIndex; i += partitionSize) {
    int subRight = i + partitionSize - 1;
    if (subRight > rightIndex) {
      subRight = rightIndex;
    }

    auto median = partitionListOfFive(pointList, i, subRight, axis);
    swapPoints(pointList, median,
               leftIndex + std::floor((i - leftIndex) / partitionSize));
  }

  // Figure out the reasons for the parameter selection
  return selectMedian(
      pointList, leftIndex,
      leftIndex + std::floor((rightIndex - leftIndex) / partitionSize),
      leftIndex + (rightIndex - leftIndex) / (partitionSize * 2), axis);
}

void sortPoints(const std::vector<std::shared_ptr<Point>> &pointList,
                int axis) {
  if (axis < 0 || axis > 2) {
    // Invalid index given. Won't sort
    return;
  }

  // Select the pivot through Median of Medians algorithm
  // TODO Don't pass in the plist but rather a copy of pointers
  // int pivotIndex = selectMedian(&plist, 0, plist->size(), axis);
  int pivotIndex = 1;
  std::cout << "Found pivot index: " << pivotIndex << std::endl;
}

void KDTree::swapPoints(const std::vector<std::shared_ptr<Point>> &pointList,
                        int leftIndex, int rightIndex) {
  auto tempPoint = *pointList[leftIndex];
  *pointList[leftIndex] = *pointList[rightIndex];
  *pointList[rightIndex] = tempPoint;
}
