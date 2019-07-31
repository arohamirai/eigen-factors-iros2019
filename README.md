# Eigen-Factors

The present project is the implementation of the paper "Eigen-Factors: Plane Estimation for Multi-Frame and Time-Continuous Point Cloud Alignment" :
```
@inproceedings{ferrer2019,
  title={Eigen-Factors: Plane Estimation for Multi-Frame and Time-Continuous Point Cloud Alignment},
  author={Ferrer, Gonzalo},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2019}
}
```

You can find [here](http://sites.skoltech.ru/app/data/uploads/sites/50/2019/07/ferrer2019planes.pdf) the author's version of the document.

Here is a video showing how the Eigen-Factors work.

<iframe width="560" height="315" src="https://www.youtube.com/embed/_1u_c43DFUE" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## Dependencies

`sudo apt install build-essential cmake libeigen3-dev`

Install source open3d, from this [link](http://www.open3d.org/docs/release/compilation.html). For this project we did not use the python version.

## Compiling
```
cd eigen-factors-iros2019
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



