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
        Surfaces(std::shared_ptr<KDTree> kdtree, int m, int n);

        std::shared_ptr<PointPointerList> getSurfaceMLS();
        std::shared_ptr<PointPointerList> getSurfaceBTPS();

        /**
         * Update m and n values, no computations.
         */
        void setGrid(int m, int n);

        /**
         * Compute surface based on MLS method
         */
        void updateSurfacesMLS();

        /**
         * Compute surface based on Bézier Tensor Product Surface method
         */
        void updateSurfacesBTPS();

    private:
        std::shared_ptr<PointPointerList> surfaceMLS = nullptr;
        std::shared_ptr<PointPointerList> surfaceBTPS = nullptr;
        std::shared_ptr<KDTree> kdtree;
        int m;
        int n;
};

#endif // SURFACES_HPP
