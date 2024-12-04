# Course Project (PBD Grains)

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
./pbd-grains ../resources ../data
```

## Note

- The simulation and rendering loop are set up in main.cpp. The approach used is to run both threads in parallel and the simulation thread updates the state which is read by the rendering thread. The rendering thread does not wait for the simulation thread. If the timestep used is `h` and processing one timestep takes time `x`, the simulation loop waits for time = `max(h - x, 0)` before processing the next timestep. This makes sure that the simulation is displayed at the expected rate.
- Grain physics is defined in Grains.h and Grains.cpp
- Spatial Hashing is defined in SpatialHash.h and SpatialHash.cpp


## Credits

This project uses:
- Rendering boilerplate code from Dr. Sueda's [Computer Animation course (CSCE-689)](https://people.engr.tamu.edu/sueda/courses/CSCE450/2023F/index.html), which has been modified to support OpenGL 4, and to load multiple scenes whilepausing the simulation
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) for vector math
- [https://github.com/g-truc/glm](glm) for vector math and for opengl support
- [Dear ImGui](https://github.com/ocornut/imgui) for immediate mode UI
- [One TBB](https://github.com/uxlfoundation/oneTBB) for parallel processing