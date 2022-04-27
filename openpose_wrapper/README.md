# OpenPTrack OpenPose Wrapper
ROS Wrapper for OpenPose to be used with [OpenPTrack v2](https://github.com/unipd-rvlab/open_ptrack_v2)]

## Dependencies
* [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) (tested with OpenPose v1.7.0)


## Launch files
**opw\_default.launch**
Contains the default values for each parameter

**custom\_configuration\_example.launch**
Contains an example on how to set some parameters of choice for either a Kinect v2 or a Realsense camera


## Usage
```
roslaunch openpose_wrapper custom_configuration_example.launch
```
