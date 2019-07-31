# MROB: Mobile Robotics library
The Skoltech Mobile Robotics library (mrob) is our common framework for implementing our robotics research and projects. It includes a core set of functionalities including perception, path planning and optimization. The present library is meant to be a self-contained library.
* [common](https://cdise-bitbucket.skoltech.ru/projects/MR/repos/mrob/browse/src/common): common matrix definitions and typedefs.
* [SE3](https://cdise-bitbucket.skoltech.ru/projects/MR/repos/mrob/browse/src/SE3): Rigid Body Transformations library.
* [PCReg](https://cdise-bitbucket.skoltech.ru/projects/MR/repos/mrob/browse/src/PCRegistration): Point Cloud Registration (WIP)

## Dependencies
* C++'14
* CMake
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) (requires installation)

`sudo apt install build-essential cmake libeigen3-dev`

## Installation
```
cd mrob
mkdir build
cd build
cmake ..
make -j
```

