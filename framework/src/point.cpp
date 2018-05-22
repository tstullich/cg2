#include "point.hpp"
#include "kdtree.hpp"

float Point::fetchPointValue(int axis) {
  if (axis == 0) {
    return x;
  }
  if (axis == 1) {
    return y;
  }
  if (axis == 2) {
    return z;
  }
  // Returning the smallest possible number here
  // to make sure that someone notices a mistake
  return std::numeric_limits<float>::min();
}

float Point::distPoint(const Point &b) const {
  float xd = this->x - b.x;
  float yd = this->y - b.y;
  float zd = this->z - b.z;
  if (K_DIMENSION == 2) {
      return sqrt(xd * xd + yd * yd);
  } else {
      return sqrt(xd * xd + yd * yd + zd * zd);
  }
}

float Point::distNode(const Node &n) const {
  float d = 0;
  if (!(this->x > n.borders.xMin && this->x < n.borders.xMax))
    d = std::max(d, std::min(std::abs(n.borders.xMin - this->x),
                             std::abs(n.borders.xMax - this->x)));
  if (!(this->y > n.borders.yMin && this->y < n.borders.yMax))
    d = std::max(d, std::min(std::abs(n.borders.yMin - this->y),
                             std::abs(n.borders.yMax - this->y)));
  if (!(this->z > n.borders.zMin && this->z < n.borders.zMax))
    d = std::max(d, std::min(std::abs(n.borders.zMin - this->z),
                             std::abs(n.borders.zMax - this->z)));
  return d;
}
