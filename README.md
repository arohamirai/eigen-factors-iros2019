# Eigen-Factors

The present project is the implementation of the paper "Eigen-Factors: Plane Estimation for Multi-Frame and Time-Continuous Point Cloud Alignment" :
```
@inproceedings{ferrer2019,
  title={Eigen-Factors: Plane Estimation for Multi-Frame and Time-Continuous Point Cloud Alignment},
  author={Ferrer, Gonzalo},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={1278--1284},
  year={2019}
}
```

You can find [here](http://sites.skoltech.ru/app/data/uploads/sites/50/2019/07/ferrer2019planes.pdf) the author's version of the document.

Here is a [video](https://www.youtube.com/_1u_c43DFUE) showing how the Eigen-Factors work.


## Dependencies

**NOTE**: the original code was built for Open3d v 0.5.0. We have updated code to be compliant with the new Open3d 0.8.0 cpp interface. If you want to use the old version, simply checkot the previous version. If you have installed the 0.8.0 version, just compile this form.

`sudo apt install build-essential cmake libeigen3-dev`

Install source Open3d 0.8.0, from this [link](http://www.open3d.org/docs/release/compilation.html). For this project we did not use the python version.

A more detailed explanation.
 - 1 Download open3d (tested on version e6568ea3ee57a53828c6353a1da04638c9a234e4, from 1-10-2019)
`git clone --recursive https://github.com/intel-isl/Open3D`

 - 2 Download Open3D dependencies: 
```
cd Open3D
util/scripts/install-deps-ubuntu.sh
```

 - 3 Compile and install
```
mkdir build
cmake ..
make -j
sudo make install
```

Now, you should have in your /usr/local/include the Open3D project installed. It should also be deteted by our project when doing cmake (output Found Open3D 0.8.0.0).

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



