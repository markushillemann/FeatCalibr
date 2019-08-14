# FeatCalibr
FeatCalibr is a Matlab-Toolbox for automatic extrinsic self-calibration based on geometric 3D-features. FeatCalibr estimates the rigid transformation between a 6-DOF pose estimation sensor (e.g. GPS/INS, Camera + visual odometry/SLAM, Motion Capture System, etc.) and a mapping sensor (e.g. a 2D/3D laser scanner, depth camera, stereo camera, etc.) without using special calibration objects such as a checkerboard.

![](calibrationExample.png )

## References
Details to the calibration approach can be found in the following paper:

Markus Hillemann, Martin Weinmann, Markus S. Mueller, Boris Jutzi (2019). Automatic Extrinsic Self-Calibration of Mobile Mapping Systems Based on Geometric 3D Features. ......

Please cite the paper when using FeatCalibr or parts of it in an academic publication.

## Dependencies
The current version of FeatCalibr requires the following MATLAB toolboxes:

* Optimization Toolbox
* Statistics and Machine Learning Toolbox
* Computer Vision Toolbox (for downsampling and displaying point clouds)
* Robotics System Toolbox (just for converting between rotation parametrizations. This dependency will be removed in the next version of FeatCalibr.)
* Parallel Computing Toolbox (to accelerate the computations. This dependency will be optional in the next version of FeatCalibr.)

## Usage
Run the script 'runFeatCalibr.m' to see an example.
