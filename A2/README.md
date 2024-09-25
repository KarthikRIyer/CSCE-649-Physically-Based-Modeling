# Assignment 2 (Particle Systems)

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
./A2 ../resources ../data
```

## Note

- The simulation and rendering loop are set up in main.cpp. The approach used is to run both threads in parallel and the simulation thread updates the state which is read by the rendering thread. The rendering thread does not wait for the simulation thread. If the timestep used is `h` and processing one timestep takes time `x`, the simulation loop waits for time = `max(h - x, 0)` before processing the next timestep. This makes sure that the simulation is displayed at the expected rate.
- All geometry is loaded in Scene.cpp, into the Shape class defined in Shape.h/Shape.cpp
- Particle-object collision is handled in the `detectCollision` and `step` functions in the Particle class.
- Pressing the `f` key and running the sim will write particle positions out to a file. Note that the sim will become significantly slower because of file IO.
- Pressing the `z` key will render in wireframe mode, showing the mesh triangles


## Credits

This project uses:
- Rendering boilerplate code from Dr. Sueda's [Computer Animation course (CSCE-689)](https://people.engr.tamu.edu/sueda/courses/CSCE450/2023F/index.html), which has been modified to support OpenGL 4, and to load multiple scenes whilepausing the simulation
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) for vector math
- [https://github.com/g-truc/glm](glm) for vector math and for opengl support
- [Dear ImGui](https://github.com/ocornut/imgui) for immediate mode UI
- [One TBB](https://github.com/oneapi-src/oneTBB) for parallel processing
- [Circular Saw model](https://sketchfab.com/3d-models/circular-saw-blade-219c51b8284345568e4a5a80156125ef) by YouniqueIdeaStudio on Sketchfab
- Chladni Plate simulation inspired by this [Blender tutorial](https://youtu.be/Nqus6inp9Tk) by Kris Bettini
- Circular saw sparks simulation inspired by this [Blender tutorial](https://youtu.be/QCMyvcc3ZaU) by Blenderguru
