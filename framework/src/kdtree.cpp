#include "kdtree.hpp"

KDTree::KDTree(std::unique_ptr<PointList> plist, Borders outerBox)
    : plist(std::move(plist)) {
  buildTree(outerBox);
}

void KDTree::addToTree(std::shared_ptr<Point> point) {
  plist->push_back(*point);

  if(point->distNode(*this->getRootnode()) == 0)
    recursiveAddToTree(this->getRootnode(), point);
  else
    std::cerr << "Error: New point is outside the outer box!" << std::endl;
}

void KDTree::recursiveAddToTree(std::shared_ptr<Node> n, std::shared_ptr<Point> p) {

  n->plist.push_back(p);
  if (!n->nlist.empty()) {
    if ((n->split.axis == 0 && p->x < n->split.value) ||
        (n->split.axis == 1 && p->y < n->split.value) ||
        (n->split.axis == 2 && p->z < n->split.value)) {
      recursiveAddToTree(n->nlist[0], p);
    } else {
      recursiveAddToTree(n->nlist[1], p);
    }
  }
}

PointPointerList KDTree::collectInRadiusSimple(const Point &p, float radius) {
  PointPointerList plist;
  for (int i = 0; i < static_cast<int>(rootnode->plist.size()); i++) {
    if (p.distPoint(*rootnode->plist[i]) < radius) {
      plist.push_back(rootnode->plist[i]);
    }
  }
  return plist;
}

PointPointerList KDTree::collectInRadius(const Point &p, float radius) {
  PointPointerList pl;
  /*
  if (p.distNode(*rootnode) > 0)
    std::cout
        << "The given point is not contained in the rootnode of the KDTree box."
        << std::endl;
  else
  */
  recursiveLeafSearch(p, radius, *rootnode, pl);
  return pl;
}

void KDTree::recursiveLeafSearch(const Point &p, float radius, const Node &n,
                                 PointPointerList &pl) {
  if (n.nlist.size() > 0) {
    for (unsigned int i = 0; i < n.nlist.size(); i++) {
      if (p.distNode(n) <= radius)
        recursiveLeafSearch(p, radius, *n.nlist[i], pl);
    }
  } else {
    for (unsigned int i = 0; i < n.plist.size(); i++) {
      if (p.distPoint(*n.plist[i]) <= radius) {
        pl.push_back(n.plist[i]);
      }
    }
  }
}

bool KDTree::isClosestPoint(const Point &p, const Point &referencePoint) {
  std::shared_ptr<Point> nearestPoint = this->collectKNearest(p, 1)[1];
  if(referencePoint.x == nearestPoint->x && referencePoint.y == nearestPoint->y &&
      referencePoint.z == nearestPoint->z) {
    return true;
  }
  return false;
}

PointPointerList KDTree::collectKNearestSimple(const Point &p, int knearest) {
  PointPointerList pl;
  PriorityQueue pq;
  add(knearest, p, pq, rootnode->plist);
  unsigned int pq_size = pq.size();
  for (unsigned int i = 0; i < pq_size; i++) {
    pl.emplace_back(std::move(pq.top()));
    pq.pop();
  }
  return pl;
}

PointPointerList KDTree::collectKNearest(const Point &p, int knearest) {
  PointPointerList pl;
  /*
  if (p.distNode(*rootnode) > 0)
    std::cout
        << "The given point is not contained in the rootnode of the KDTree box."
        << std::endl;
  else*/
  if (rootnode->plist.size() <= knearest + 1) {
    PointPointerList ppl(rootnode->plist);
    pl = ppl;
  } else {
    PriorityQueue pq;
    NodeList nl;
    recursiveKNearestSearch(p, knearest + 1, rootnode, nl, pq);
    unsigned int pq_size = pq.size();
    for (unsigned int i = 0; i < pq_size; i++) {
      pl.emplace_back(std::move(pq.top()));
      pq.pop();
    }
  }
  return pl;
}

float KDTree::distToClosestPoint(const Point &p) {
  PointPointerList pl;
  if (rootnode->plist.size() <= 1) {
    PointPointerList ppl(rootnode->plist);
    pl = ppl;
  } else {
    PriorityQueue pq;
    NodeList nl;
    recursiveKNearestSearch(p, 1, rootnode, nl, pq);
    unsigned int pq_size = pq.size();
    for (unsigned int i = 0; i < pq_size; i++) {
      pl.emplace_back(std::move(pq.top()));
      pq.pop();
    }
  }
  assert(pl.size() == 1);
  return glm::length(p.toVec3() - pl.front()->toVec3());

}

glm::vec3 computeNormal(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3) {
  return glm::normalize(glm::cross((v2 - v1), (v3 - v1)));
}

bool pointInsideTriangle(glm::vec3 p, glm::vec3 v1, glm::vec3 v2, glm::vec3 v3) {
  glm::vec3 N1 = computeNormal(p, v1, v2);
  glm::vec3 N2 = computeNormal(p, v2, v3);
  glm::vec3 N3 = computeNormal(p, v3, v1);

  // check if normals are orientated the same way
  if ((glm::length(N1 + N2) > 1) && (glm::length(N1 + N3) > 1)) {
    return true;
  } else {
    return false;
  }
}

float KDTree::distToSurface(const Point &p, glm::vec3 rayPos, glm::vec3 rayDir) {
  // if close enough, compute exact distance
  PointPointerList pl;
  if (rootnode->plist.size() <= 3) {
    PointPointerList ppl(rootnode->plist);
    pl = ppl;
  } else {
    PriorityQueue pq;
    iterativKNearestSearch(p, 3, pq);
    unsigned int pq_size = pq.size();
    for (unsigned int i = 0; i < pq_size; i++) {
      pl.emplace_back(std::move(pq.top()));
      pq.pop();
    }
  }
  assert(pl.size() == 3);

  // project p onto plain
  glm::vec3 N = computeNormal(pl[0]->toVec3(), pl[1]->toVec3(), pl[2]->toVec3());
  glm::vec3 V = p.toVec3() - pl[0]->toVec3();
  float dist = glm::dot(V, N);
  glm::vec3 plainPoint = p.toVec3() - dist * N;

  // check if projected point is inside triangle
  bool inside = pointInsideTriangle(plainPoint,
                                    pl[0]->toVec3(),
                                    pl[1]->toVec3(),
                                    pl[2]->toVec3());

  if (inside) { // projected point inside triangle, so return dist to it
    return glm::length(p.toVec3() - plainPoint);
  } else {  // project point not inside triangle, so return min dist to points
    float minDist = std::numeric_limits<float>::max();
    for (unsigned i = 0; i < 3; ++i) {
      float tmp = glm::length(pl[i]->toVec3() - p.toVec3());
      minDist = (tmp < minDist)? tmp : minDist;
    }
    return minDist;
  }
}

void KDTree::iterativKNearestSearch(const Point &p, int k, PriorityQueue &pq) {
  std::shared_ptr<Node> n = this->rootnode;

  // to blacklist nodes we use a bool-vector
  unsigned maxNumOfNodes = pow(2, maxDepth+1) - 1;
  std::vector<bool> nodeBlackList(maxNumOfNodes);
  for (unsigned i = 0; i < maxNumOfNodes; ++i) {
    nodeBlackList[i] = false;
  }

  // this is used to keep track of the current node index
  unsigned currIndex = 0;

  if (pq.empty()) {
    while (!(n->nlist.empty()) && !(n->nlist[1]->plist.size() < k)) {
      // descending
      if ((n->split.axis == 0 && p.x < n->split.value) ||
          (n->split.axis == 1 && p.y < n->split.value) ||
          (n->split.axis == 2 && p.z < n->split.value)) {
        n = n->nlist[0];
        currIndex = (currIndex * 2) + 1;
      } else {
        n = n->nlist[1];
        currIndex = (currIndex * 2) + 2;
      }
    }
    // hit rock bottom
    add(k, p, pq, n->plist);
    nodeBlackList[currIndex] = true;
  }

  // ascent and check rest of tree
  while (true) {
    if (nodeBlackList[currIndex] == true) {
      if (n->parent != nullptr) {
        // ascending
        n = n->parent;
        currIndex = ceil((float)currIndex / 2.0) - 1;
        continue;
      } else {
        // hit ceiling
        break;
      }
    } else {
      if (p.distNode(*n) < pq.top()->dist) {
        if (n->nlist.empty()) {
          add(k, p, pq, n->plist);
          nodeBlackList[currIndex] = true;
          continue;
        } else {
          bool child1_exists = nodeBlackList[(currIndex * 2) + 1];
          bool child2_exists = nodeBlackList[(currIndex * 2) + 2];
          if (child1_exists && child2_exists) {
            nodeBlackList[currIndex] = true;
            continue;
          } else if (!child1_exists) {
            n = n->nlist[0];
            currIndex = (currIndex * 2) + 1;
            continue;
          } else if (!child2_exists) {
            n = n->nlist[1];
            currIndex = (currIndex * 2) + 2;
            continue;
          }
        }
      } else {
        nodeBlackList[currIndex] = true;
        continue;
      }
    }
  }
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
          (n->split.axis == 2 && p.z < n->split.value)) {
        recursiveKNearestSearch(p, k, n->nlist[0], nl, pq);
      } else {
        recursiveKNearestSearch(p, k, n->nlist[1], nl, pq);
      }
    }
  } else {
    if (std::find(nl.begin(), nl.end(), n) != nl.end()) {
      if (n->parent != nullptr) {
        recursiveKNearestSearch(p, k, n->parent, nl, pq);
      } else {
        return;
      }
    } else {
      if (p.distNode(*n) < pq.top()->dist) {
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
          } else if (!child1_exists) {
            recursiveKNearestSearch(p, k, n->nlist[0], nl, pq);
          } else if (!child2_exists) {
            recursiveKNearestSearch(p, k, n->nlist[1], nl, pq);
          }
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
    pl[i]->dist = p.distPoint(*pl[i]);
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

  float x = outerBox.xMin + (outerBox.xMax - outerBox.xMin) / 2.0;
  float y = outerBox.yMin + (outerBox.yMax - outerBox.yMin) / 2.0;
  float z = outerBox.zMin + (outerBox.zMax - outerBox.zMin) / 2.0;
  center = glm::vec3(x, y, z);
  diagonal = glm::length(glm::vec3(outerBox.xMax, outerBox.yMax, outerBox.zMax) - glm::vec3(outerBox.xMin, outerBox.yMin, outerBox.zMin));

  centerOfGravity = glm::vec3(0.0f, 0.0f, 0.0f);
  for (unsigned int i = 0; i < plist->size(); i++) {
    rootnode->plist.push_back(std::make_shared<Point>(plist->at(i)));
    centerOfGravity += rootnode->plist.back()->toVec3();
  }
  centerOfGravity /= rootnode->plist.size();

  std::cout << "Start to build KDTree from " << rootnode->plist.size()
            << " data points." << std::endl;
  std::cout << "Outer box characteristics : x = [" << outerBox.xMin << ", "
            << outerBox.xMax << "]"
            << " y = [" << outerBox.yMin << ", " << outerBox.yMax << "]"
            << " z = [" << outerBox.zMin << ", " << outerBox.zMax << "]"
            << " center = " << center[0] << ", " << center[1] << ", " << center[2]
            << " diagonal = " << diagonal
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
    // " << depth << std::endl;
    return;
  }

  node->split.axis = depth % kDimension;
  uint64_t medianIndex = sortPoints(node->plist, node->split.axis);
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
  for (int i = leftIndex; i < rightIndex; i++) {
    if (pointList[i]->fetchPointValue(axis) < pivotValue) {
      swapPoints(pointList, storeIndex, i);
      storeIndex++;
    }
  }

  swapPoints(pointList, rightIndex, storeIndex);

  return storeIndex;
}

int KDTree::sortListOfFive(const std::vector<std::shared_ptr<Point>> &pointList,
                           int startIndex, int endIndex, int axis) {
  int i = startIndex + 1;
  while (i <= endIndex) {
    int j = i;
    while (j > startIndex && pointList[j - 1]->fetchPointValue(axis) >
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
    sortListOfFive(pointList, leftIndex, rightIndex, axis);
    return (leftIndex + rightIndex) / 2;
  }

  for (int i = leftIndex; i < rightIndex; i += partitionSize) {
    int subRight = i + partitionSize - 1;
    if (subRight > rightIndex) {
      subRight = rightIndex;
    }

    auto median = sortListOfFive(pointList, i, subRight, axis);
    swapPoints(pointList, median,
               leftIndex + std::floor((i - leftIndex) / partitionSize));
  }

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

  auto result = selectMedian(pointList, 0, pointList.size() - 1,
                             pointList.size() / 2, axis);
  return result;
}

void KDTree::swapPoints(const std::vector<std::shared_ptr<Point>> &pointList,
                        int leftIndex, int rightIndex) {
  auto tempPoint = *pointList[leftIndex];
  *pointList[leftIndex] = *pointList[rightIndex];
  *pointList[rightIndex] = tempPoint;
}

float KDTree::getDiagonal() {
  return this->diagonal;
}

glm::vec3 KDTree::getCenter() {
  return center;
}

glm::vec3 KDTree::getCenterOfGravity() {
  return centerOfGravity;
}
