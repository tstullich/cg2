#ifndef VERTEX_HPP
#define VERTEX_HPP

/**
 * Quick mockup of a Vertex according to the OFF file
 * format. All of the members are public for now for
 * easy data access, but we can refine this class later on.
 */
class Vertex {
  public:
    Vertex(float x, float y, float z) : x(x), y(y), z(z) {}
    float x;
    float y;
    float z;
};
#endif // VERTEX_HPP
