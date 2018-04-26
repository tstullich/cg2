#include <iostream>

#include "cg2_framework.hpp"
#include "parser.hpp"

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cout << "Unable to open a file. Please specify a file with the"
              << " executable." << std::endl;
    return 1;
  }

  Parser p;
  if (!p.open(argv[1])) {
    std::cout << "Unable to open file " << argv[1] << std::endl;
    return 1;
  }

  // Start the construction of a tree
  KDTree tree(p.getPoints(), std::make_unique<EuclDist>());
  for (auto p : *tree.getPoints()) {
    std::cout << "Point: (" << p.x << ", " << p.y << ", " << p.z << ")"
              << std::endl;
  }

  p.close();
  return 0;
}
