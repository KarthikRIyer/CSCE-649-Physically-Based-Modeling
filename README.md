# CSCE-649-Physically-Based-Modeling

![CSCE-649](screenshots/pbd-grains-render-csce.png)

This is a collection of assignments done as part of the Physically Based Modeling course at Texas A&M University.
Each assignment is in a self contained directory.

## How to build and run

From the directory of any assignment run:

```console
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
./<application-name> ../resources ../data
```

## Assignments

### A1 - Bouncing Ball
![A1 - Bouncing Ball](screenshots/A1.png)

### A2 - Particle Systems

Sparks             |  Sparks Rendered
:-------------------------:|:-------------------------:
![A2 - Sparks](screenshots/A2-sparks.png)  |  ![A2 - Sparks Rendered](screenshots/A2-sparks-render.png)

Chladni Plate             | 
:-------------------------:|
![](screenshots/A2-chladni.png)  |

### A3 - Flocking

![A3 - Flocking](screenshots/A3.png)


### A4 - Springy Structures

![A4 - Jello Cube](screenshots/A4.png)


### A5 - Rigid Bodies

![A4 - Rigid Cube](screenshots/A5.png)


### Final Project - PBD Granular Material Simulation

Grains             |  Grains Rendered
:-------------------------:|:-------------------------:
![Project - Grains](screenshots/pbd-grains.png)  |  ![Project - Grains Rendered](screenshots/pbd-grains-render.png)
