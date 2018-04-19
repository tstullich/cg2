#ifndef PARSER_HPP
#define PARSER_HPP

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "face.hpp"
#include "vertex.hpp"

class Parser {
  public:
    /**
     * Parses the vertices that are contained in the file.
     * This function relies the numVertices variable or being
     * set correctly to determine how many vertices need to
     * be created for the vector.
     */
    bool open(const std::string &fileName);

    /**
     * Parses the number of vertices contained in the OFF file
     * Currently keeping this in a vector until we can figure
     * out what else we need to do with the data
     */
    std::vector<Vertex> getVertices();

    /**
     * Parses the number of faces contained in the OFF file
     * Currently keeping this in a vector until we can figure
     * out what else we need to do with the data
     *
     * !!! This funtion needs to be called only after getVertices
     *     is called. Otherwise we get unexpected behavior !!!
     */
    std::vector<Face> getFaces();

    // Not sure if we need this but I am keeping this stubeed for now
    std::vector<int> getEdges();

    /**
     * Closes the filestream that was opened
     */
    void close();

  private:
    std::ifstream inputStream;
    int numVertices;
    int numFaces;
    int numEdges;
};
#endif // PARSER_HPP
