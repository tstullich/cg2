#include "cg2_framework.hpp"
#include <cmath>

float EuclDist::dist(const Point &a, const Point &b) {
  float xd = a.x - b.x;
  float yd = a.y - b.y;
  float zd = a.z - b.z;
  return sqrt(xd * xd + yd * yd + zd * zd);
}

float EuclDist::dist(const Point &a, const Node &n) {
  float d = 0;
  if (!(a.x >= n.borders.xMin && a.x < n.borders.xMax))
    d = std::max(d, std::min(std::abs(n.borders.xMin - a.x),
                             std::abs(n.borders.xMax - a.x)));
  if (!(a.y >= n.borders.yMin && a.y < n.borders.yMax))
    d = std::max(d, std::min(std::abs(n.borders.yMin - a.y),
                             std::abs(n.borders.yMax - a.y)));
  if (!(a.z >= n.borders.zMin && a.z < n.borders.zMax))
    d = std::max(d, std::min(std::abs(n.borders.zMin - a.z),
                             std::abs(n.borders.zMax - a.z)));
  return d;
}

KDTree::KDTree(std::unique_ptr<PointList> plist,
               std::unique_ptr<DistFunc> dfunc, Borders outerBox)
    : plist(std::move(plist)), dfunc(std::move(dfunc)) {
  buildTree(outerBox);
}

PointPointerList KDTree::collectInRadiusSimple(const Point &p, float radius) {
  PointPointerList plist;
  for (int i = 0; i < static_cast<int> (rootnode->plist.size()); i++) {
    if (dfunc->dist(p, *rootnode->plist[i]) < radius) {
      plist.push_back(rootnode->plist[i]);
    }
  }
  return plist;
}

PointPointerList KDTree::collectInRadius(const Point &p, float radius) {
  PointPointerList pl;
  if (dfunc->dist(p, *rootnode) > 0)
    std::cout
        << "The given point is not contained in the rootnode of the KDTree box."
        << std::endl;
  else
    recursiveLeafSearch(p, radius, *rootnode, pl);
  return pl;
}

void KDTree::recursiveLeafSearch(const Point &p, float radius, const Node &n,
                                 PointPointerList &pl) {
  if (n.nlist.size() > 0) {
    for (unsigned int i = 0; i < n.nlist.size(); i++) {
      if (dfunc->dist(p, n) <= radius)
        recursiveLeafSearch(p, radius, *n.nlist[i], pl);
    }
  } else {
    for (unsigned int i = 0; i < n.plist.size(); i++) {
      if (dfunc->dist(p, *n.plist[i]) <= radius) pl.push_back(n.plist[i]);
    }
  }
}

PointPointerList KDTree::collectKNearestSimple(const Point &p, int knearest) {
  PointPointerList pl;
  PriorityQueue pq;
  add(knearest, p, pq, rootnode->plist);
  // TODO optimize
  unsigned int pq_size = pq.size();
  for (unsigned int i = 0; i < pq_size; i++) {
    pl.push_back(pq.top());
    pq.pop();
  }
  // TODO delete p from pl
  return pl;
}

PointPointerList KDTree::collectKNearest(const Point &p, int knearest) {
  PointPointerList pl;
  if (dfunc->dist(p, *rootnode) > 0)
    std::cout
        << "The given point is not contained in the rootnode of the KDTree box."
        << std::endl;
  else if (rootnode->plist.size() <= knearest + 1) {
    PointPointerList ppl(rootnode->plist);
    pl = ppl;
  } else {
    PriorityQueue pq;
    NodeList nl;
    recursiveKNearestSearch(p, knearest + 1, rootnode, nl, pq);
    // TODO optimize
    unsigned int pq_size = pq.size();
    for (unsigned int i = 0; i < pq_size; i++) {
      pl.push_back(pq.top());
      pq.pop();
    }
  }
  // TODO delete p from pl
  return pl;
}

void KDTree::recursiveKNearestSearch(const Point &p, int k,
                                     std::shared_ptr<Node> &n, NodeList &nl,
                                     PriorityQueue &pq) {
  if (pq.empty()) {
    if (n->nlist.empty() || n->nlist[1]->plist.size() < k) {
      nl.push_back(n);
      add(k, p, pq, n->plist);
      recursiveKNearestSearch(p, k, n, nl, pq);
    } else {
      if ((n->split.axis == 0 && p.x < n->split.value) ||
          (n->split.axis == 1 && p.y < n->split.value) ||
          (n->split.axis == 2 && p.z < n->split.value))
        recursiveKNearestSearch(p, k, n->nlist[0], nl, pq);
      else
        recursiveKNearestSearch(p, k, n->nlist[1], nl, pq);
    }
  } else {
    if (std::find(nl.begin(), nl.end(), n) != nl.end()) {
      if (n->parent != nullptr)
        recursiveKNearestSearch(p, k, n->parent, nl, pq);
      else
        return;
    } else {
      if (dfunc->dist(p, *n) < pq.top()->dist) {
        if (n->nlist.empty()) {
          add(k, p, pq, n->plist);
          nl.push_back(n);
          recursiveKNearestSearch(p, k, n, nl, pq);
        } else {
          bool child1_exists =
              std::find(nl.begin(), nl.end(), n->nlist[0]) != nl.end();
          bool child2_exists =
              std::find(nl.begin(), nl.end(), n->nlist[1]) != nl.end();
          if (child1_exists && child2_exists) {
            nl.push_back(n);
            recursiveKNearestSearch(p, k, n, nl, pq);
          } else if (!child1_exists)
            recursiveKNearestSearch(p, k, n->nlist[0], nl, pq);
          else if (!child2_exists)
            recursiveKNearestSearch(p, k, n->nlist[1], nl, pq);
        }
      } else {
        nl.push_back(n);
        recursiveKNearestSearch(p, k, n, nl, pq);
      }
    }
  }
}

void KDTree::add(int k, const Point &p, PriorityQueue &pq,
                 PointPointerList &pl) {
  for (unsigned int i = 0; i < pl.size(); i++) {
    pl[i]->dist = dfunc->dist(p, *pl[i]);
    pq.push(pl[i]);
    if (pq.size() > k) pq.pop();
  }
}

int KDTree::size() { return static_cast<int>(plist->size()); }

std::shared_ptr<PointList> KDTree::getPoints() { return plist; }

std::shared_ptr<Node> KDTree::getRootnode() { return rootnode; }

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

bool KDTree::buildTree(Borders &outerBox) {
  rootnode = std::make_shared<Node>();
  rootnode->borders = outerBox;

  for (unsigned int i = 0; i < plist->size(); i++) {
    rootnode->plist.push_back(std::make_shared<Point>(plist->at(i)));
  }

  std::cout << "Start to build KDTree from " << rootnode->plist.size()
            << " data points." << std::endl;
  std::cout << "Outer box characteristics : x = [" << outerBox.xMin << ", "
            << outerBox.xMax << "]"
            << " y = [" << outerBox.yMin << ", " << outerBox.yMax << "]"
            << " z = [" << outerBox.zMin << ", " << outerBox.zMax << "]"
            << std::endl;
  recursiveTreeExtend(0, rootnode);
  std::cout << "KDTree built successful!" << std::endl;

  return true;
}

void KDTree::recursiveTreeExtend(unsigned int depth,
                                 std::shared_ptr<Node> node) {
  if (node->plist.size() <= maxPoints || depth >= maxDepth) {
    // Need to return here since we have reached our break condition
    // std::cout << "LeafNode: data points = " << node->plist.size() << " depth
    // = " << depth << std::endl;
    return;
  }

  node->split.axis = depth % kDimension;
  sortPoints(node->plist, node->split.axis);
  unsigned int medianIndex = node->plist.size() / 2;
  node->split.value = node->split.axis < 2
                          ? (node->split.axis < 1 ? node->plist[medianIndex]->x
                                                  : node->plist[medianIndex]->y)
                          : node->plist[medianIndex]->z;
  std::vector<PointPointerList> parts = splitList(node->plist, medianIndex);

  for (unsigned int i = 0; i < parts.size(); i++) {
    Node child;
    child.parent = node;
    child.plist = parts[i];
    if (node->split.axis != 0) {
      child.borders.xMin = node->borders.xMin;
      child.borders.xMax = node->borders.xMax;
    } else {
      child.borders.xMin = i % 2 == 0 ? node->borders.xMin : node->split.value;
      child.borders.xMax = i % 2 == 0 ? node->split.value : node->borders.xMax;
    }
    if (node->split.axis != 1) {
      child.borders.yMin = node->borders.yMin;
      child.borders.yMax = node->borders.yMax;
    } else {
      child.borders.yMin = i % 2 == 0 ? node->borders.yMin : node->split.value;
      child.borders.yMax = i % 2 == 0 ? node->split.value : node->borders.yMax;
    }
    if (node->split.axis != 2) {
      child.borders.zMin = node->borders.zMin;
      child.borders.zMax = node->borders.zMax;
    } else {
      child.borders.zMin = i % 2 == 0 ? node->borders.zMin : node->split.value;
      child.borders.zMax = i % 2 == 0 ? node->split.value : node->borders.zMax;
    }

    std::shared_ptr<Node> pChild = std::make_shared<Node>(child);
    node->nlist.push_back(pChild);
    recursiveTreeExtend(depth + 1, pChild);
  }
}

std::vector<PointPointerList> KDTree::splitList(PointPointerList &list,
                                                unsigned int index) {

  PointPointerList firstList(list.begin(), list.begin() + index);
  PointPointerList secondList(list.begin() + index, list.end());

  std::vector<PointPointerList> lists;
  lists.push_back(firstList);
  lists.push_back(secondList);

  return lists;
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
  int i = startIndex;
  while (i < endIndex) {
    int j = i;
    while (j > 0 && pointList[j - 1]->fetchPointValue(axis) >
                        pointList[j]->fetchPointValue(axis)) {
      swapPoints(pointList, j, j - 1);
      j--;
    }
    i++;
  }

  return (endIndex + startIndex) / 2;
}

int KDTree::selectMedian(const std::vector<std::shared_ptr<Point>> &pointList,
                         int leftIndex, int rightIndex, int positionOfMedian,
                         int axis) {
  while (true) {
    if (leftIndex == rightIndex) {
      return leftIndex;
    }

    auto pivotIndex =
        selectMedianOfMedians(pointList, leftIndex, rightIndex, axis);
    pivotIndex =
        partitionList(pointList, leftIndex, rightIndex, pivotIndex, axis);
    if (positionOfMedian == pivotIndex) {
      return positionOfMedian;
    } else if (positionOfMedian < pivotIndex) {
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

  // std::cout << "Performing splits: " << std::endl;
  for (int i = leftIndex; i < rightIndex; i += partitionSize) {
    int subRight = i + partitionSize - 1;
    if (subRight > rightIndex) {
      subRight = rightIndex;
    }

    // std::cout << "Splitting list: " << i << ", " << subRight << std::endl;
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

int KDTree::sortPoints(const std::vector<std::shared_ptr<Point>> &pointList,
                       int axis) {
  if (axis < 0 || axis > 2) {
    // Invalid index given. Won't sort
    std::cout << "Invalid axis parameter was given. Will not sort the list."
              << std::endl;
    return -1;
  }

  return selectMedian(pointList, 0, pointList.size() - 1, pointList.size() / 2,
                      axis);
}

void KDTree::swapPoints(const std::vector<std::shared_ptr<Point>> &pointList,
                        int leftIndex, int rightIndex) {
  auto tempPoint = *pointList[leftIndex];
  *pointList[leftIndex] = *pointList[rightIndex];
  *pointList[rightIndex] = tempPoint;
}
