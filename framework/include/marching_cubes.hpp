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
/*
   Linearly interpolate the position where an isosurface cuts
   an edge between two vertices, each with their own scalar value
*/
static Point vertexInterp(double isolevel, const Point &p1, const Point &p2, double valp1, double valp2);

/*
   Given a grid cell and an isolevel, calculate the triangular
   facets required to represent the isosurface through the cell.
   Return the number of triangular facets, the array "triangles"
   will be loaded up with the vertices at most 5 triangular facets.
        0 will be returned if the grid cell is either totally above
   of totally below the isolevel.
*/
static std::vector<Triangle> polygonise(GridCell grid, double isolevel);
};
#endif // MARCHING_CUBES_HPP
