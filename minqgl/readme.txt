This provides the source for a minimal application using Qt for Window/gui and OpenGL for 3D visualization.

Compilation Instuctions:

Linux:
[Install dependencies]
mkdir build
cd build
cmake ..
make

Windows:
Using CMake (http://www.cmake.org/) and VisualStudio 2010
-Install CMake.
-Run CMake-GUI
-Select as source directory the folder where the CMakeLists.txt file is
-Create a build folder and select it for the target using CMAKE-GUI
-Press Configure and select the visual studio 2010 compiler
-If everything went right, press configure and the visual studio project should be in the build folder ready to compile

MacOS:
- Install Qt5 (e.g. using brew).
- Execute cmake.
- Build project using make.
