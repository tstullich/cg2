

#include "parser.hpp"

int main(int argc, char *argv[])
{
  Parser p;
  if (!p.open("src/off_files/dragon.off")) {
    std::cout << "Unable to open file" << std::endl;
  }
  return 0;
}
