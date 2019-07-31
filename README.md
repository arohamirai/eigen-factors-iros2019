# Eigen-Factors

## Dependencies

`sudo apt install build-essential cmake libeigen3-dev`

Install source open3d, from this [link](http://www.open3d.org/docs/release/compilation.html). For this project it is not required the python version.

## Compiling
```
cd eige_factors_iros2019
mkdir build
cd build
cmake ..
make -j
```

## Running code

Run Eigen-factors visualizer. It generates a number of random planes (specified at planeVisualizer.cpp) and a number of poses on the trajectory.
This executable shows first the non-aligned scene, then optimizes over EFs on single iterration, and then show the comparison with ICP

`./bin/planeVisualizer`


Second executable sweepes over the hyperparamtyers alpha and beta for the NAG optimization method.
`./bin/syntheticOptimizationEval_params`


Third executable is a comparison of the optimziation, gradient vs NAG
`./bin/syntheticOptimizationEval_nag_vs_gd`


Last executalbe, compares EFs with two variants of ICP, point to point and point to plane:
`./bin/syntheticOptimizationEval_eigen_vs_icps`
