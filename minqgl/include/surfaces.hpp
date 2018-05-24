#ifndef SURFACES_HPP
#define SURFACES_HPP

#include <algorithm>
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

#include "kdtree.hpp"

class Surfaces {
    public:
        Surfaces(std::shared_ptr<KDTree> kdtree, int m, int n, float r);

        PointPointerList getSurfaceMLS();
        PointPointerList getSurfaceBTPS();

        /**
         * Update m and n values, no computations.
         */
        void setGrid(int m, int n);

        /**
         * Update radius value, no computations.
         */
        void setRadius(float r);

        /**
         * Compute surface based on MLS method
         */
        void updateSurfacesMLS();

        /**
         * Compute surface based on Bézier Tensor Product Surface method
         */
        void updateSurfacesBTPS();

    private:
        PointPointerList surfaceMLS;
        PointPointerList surfaceBTPS;
        std::shared_ptr<KDTree> kdtree;
        int M;
        int N;
        float radius;
};

#endif // SURFACES_HPP
