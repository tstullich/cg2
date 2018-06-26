#ifndef MARCHING_CUBES_HPP
#define MARCHING_CUBES_HPP
#include <vector>

#include "point.hpp"
#include "triangle.hpp"

typedef struct {
  Point p[8];
  double val[8];
} GridCell;

class MarchingCubes {
public:
static std::vector<Triangle> polygonise(const GridCell &grid, double isolevel);

private:
static Point vertexInterp(double isolevel, const Point &p1, const Point &p2, double valp1, double valp2);

};
#endif // MARCHING_CUBES_HPP
