# Fusion of LiDAR data from two LiDAR sensors using ROS

This repository contains a ROS package with a C++ node to fuse LiDAR data from two LiDAR sensors along with a LeGO-LOAM submodule modified for using the package. The node takes in point clouds from two Velodyne VLP-16 LiDAR sensors and merges them to form a single point cloud. Merging point clouds from multiple lidars eliminates the blindspots caused when one LiDAR sensor blocks the laser pulses from another LiDAR sensor.

THe images of the blindspots are shown below.



## Dependency

- [ROS](http://wiki.ros.org/ROS/Installation) (Noetic)
- [gtsam](https://github.com/borglab/gtsam.git) (Georgia Tech Smoothing and Mapping library, 4.1.1)


## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone --recurse-submodules https://github.com/Santhosh-Sankar/LiDAR-fusion.git
cd ..
catkin_make -j1
```
When you compile the code for the first time, you need to add "-j1" behind "catkin_make" for generating some message types. "-j1" is not needed for future compiling.


## Run the package

1. Run the launch file:
```
roslaunch lego_loam run.launch
```
Notes: The parameter "/use_sim_time" is set to "true" for simulation, "false" to real robot usage.

2. Play existing bag files:
```
rosbag play *.bag --clock --topic /ns1/velodyne_points /ns2/velodyne_points