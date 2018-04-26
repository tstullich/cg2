#ifndef POINT_HPP
#define POINT_HPP

/**
 * Quick mockup of a Point according to the OFF file
 * format. All of the members are public for now for
 * easy data access, but we can refine this class later on.
 */
class Point {
public:
  Point(float x, float y, float z) : x(x), y(y), z(z) {}
  float x;
  float y;
  float z;

  float fetchPointValue(int axis) {
    if (axis == 0) {
      return x;
    }
    if (axis == 1) {
      return y;
    }
    return z;
  }
};
#endif // POINT_HPP
