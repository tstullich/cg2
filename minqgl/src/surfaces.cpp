#include "surfaces.hpp"

Surfaces::Surfaces(std::shared_ptr<KDTree> kdtree, int m, int n)
    : kdtree(std::move(kdtree)), m(m), n(n) {
        std::cout << "kdtree:" << this-kdtree << std::endl;
        std::cout << "m:" << this->m << std::endl;
        std::cout << "n:" << this->n << std::endl;
}

std::shared_ptr<PointPointerList> Surfaces::getSurfaceMLS() { return surfaceMLS }

std::shared_ptr<PointPointerList> Surfaces::getSurfaceBTPS() { return surfaceBTPS }

void Surfaces::setGrid(int m, int n) {
    this->m = m;
    this->n = n;
}

void Surfaces::updateSurfacesMLS() {
    std::cout << "updateSurfacesMLS()" << std::endl;
}

void Surfaces::updateSurfacesBTPS() {
    std::cout << "updateSurfacesBTPS()" << std::endl;
}
