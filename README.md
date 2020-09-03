# Computer Graphics 2 (TU Berlin)
This is code base for the the Computer Graphics 2 course at the Technical University of Berlin. The course taught a number of different techniques focused
around geometric reconstruction and implicit surfaces. All of the assignments focused on a particular topic within those domains and were completed in
collaboration with two group members. For the coding portions of the assignments we used OpenGL to perform our graphics computations, with Qt5 acting as
the framework for the demos’ user interface. Qt5 allowed us to dynamically adjust the parameters for the various demonstrations.

## Installation
In order to build the project make sure you have Qt5, CMake, GLEW, GLUT and GLM installed.
Then to build follow these steps while in the root directory:

### Building
```
mkdir build
cd build
cmake ..
make
```

The resulting binary should then be found in the build directory `minqgl/src` with the name `cg2`

## Examples
### K-d Trees With Nearest Neighbor Point Search
![k-d tree](https://tstullich.github.io/img/kd-tree.png)

### Surface Reconstruction Using Marching Cubes
![The isosurface used for the marching cubes algorithm](https://tstullich.github.io/img/mc-points.png)

![The reconstructed mesh after marching cubes was run](https://tstullich.github.io/img/mc-dog.png)

## Features
The individual assignments were the features for this course. They are tagged in the repository as `v1.0`, `v2.0` and `v3.0`
- v1.0 K-d trees and nearest neighbor search
- v2.0 Laplace mesh smoothing operations and Bezier curves
- v3.0 Surface reconstruction using Marching Cubes

## Future Work
- [ ] Cleaning up the code base
