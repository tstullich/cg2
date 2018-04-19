#ifndef FACE_HPP
#define FACE_HPP

#include <iostream>
#include <vector>

/**
 * Basic class to describe a Face according
 * to the OFF file format. The underlying
 * structure to this class is a simple vector
 * and the individual elements can be accessed
 * via the [] operator.
 */
class Face {
public:
  /**
   * This lets you add a vertex index to the
   * Face object. The element will be added to
   * the back of the vector, so if order matters
   * we might need to find another solution to
   * adding new vertices
   */
  void add(int vertIndex) { faces.push_back(vertIndex); }

  /**
   * Returns the number of vertices in the current
   * vector
   */
  int numVertices() { return faces.size(); }

  /**
   * An overloaded operator to allow direct indexng
   * of face indices. Does no bounds-checking so that
   * might be useful later
   */
  int &operator[](int idx) { return faces[idx]; }

  /**
   * An overloaded operator to allow direct indexng
   * of face indices. Does no bounds-checking so that
   * might be useful later
   */
  const int &operator[](int idx) const { return faces[idx]; }

private:
  std::vector<int> faces;
};
#endif // FACE_HPP
