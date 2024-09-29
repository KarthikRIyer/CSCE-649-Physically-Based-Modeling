# Assignment 3 (Flocking)

Name: Karthik Ramesh Iyer
UIN: 234009250

## Prequisites

- OpenGL 4.x
- GLM
- GLEW
- Eigen

Set the following environment variables:
```console
GLM_INCLUDE_DIR=/Users/karthik/lib/glm-0.9.9.8;GLFW_DIR=/Users/karthik/lib/glfw-3.3.8;GLEW_DIR=/Users/karthik/lib/glew-2.1.0
```

## How to build and run

Clone the project and run the following commands in the root directory of the project:

```console
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
```

To run the simulator:

```console
./A3 ../resources ../data
```

## Note

- The simulation and rendering loop are set up in main.cpp.
- All geometry is loaded in Scene.cpp, into the Shape class defined in Shape.h/Shape.cpp


## Credits

This project uses:
- Rendering boilerplate code from Dr. Sueda's [Computer Animation course (CSCE-689)](https://people.engr.tamu.edu/sueda/courses/CSCE450/2023F/index.html), which has been modified to support OpenGL 4, and to load multiple scenes whilepausing the simulation
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) for vector math
- [https://github.com/g-truc/glm](glm) for vector math and for opengl support
- [Dear ImGui](https://github.com/ocornut/imgui) for immediate mode UI
- [One TBB](https://github.com/oneapi-src/oneTBB) for parallel processing