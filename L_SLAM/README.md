# L - SLAM Module

This is a implementation of the simultaneous localization and mapping (SLAM) solution for Lidar systems. The implementation is based on the method described [here](https://www.ri.cmu.edu/publications/loam-lidar-odometry-and-mapping-in-real-time/) and aims to be a stand-alone module to support any mobile Lidar systems and mostly Velodyne LiDAR. This work is under the [Cooper Mapper](../README.md) Porject

## LOAM in a nutshell
[Lidar Odometry and Mapping ](https://www.ri.cmu.edu/publications/loam-lidar-odometry-and-mapping-in-real-time/) is the the-state-of-arts Lidar SLAM algorithm that is able to estimate odometry and construct a map simultaneously. It has keypoints granularity and needs data input from a 3D Lidar setup and optional IMU data.
	The algorithm is able to run in real-time in modest hardware. It can be summarized in the following steps:

1. Feature extraction: incoming point clouds are unwarped with inertial measurements. Plane and edge features are extracted.
2. Laser odometry (∼ 10 Hz): scan-to-scan odometry is estimated using strong features.
3. Laser mapping (∼ 1 Hz): scan-to-map odometry is estimated us- ing strong features. All features extracted are registered with the latest odometry estimate, and the map is updated.
4. Transform integration: the motion estimates from the odometry and mapping modules are integrated.

The odometry and mapping modules estimate incremental trans- formations by employing a variant of point-to-point and point-to-plane ICP. For every pair of planar features belonging to different scans, a point-to-plane constraint is created. Similarly, point-to-point con- straints are generated between edge features. Both sets of constraints are stacked into a matrix, and Singular Value Decomposition (SVD) is employed to estimate the optimal transformation.

## Block Diagram

![alt text](../assets/pics/block-diagram.png)

Block diagram of the lidar odometry and mapping software system.

## Refactorization

* Add encapsulation to the repetitive code block in the original work
* Refactor the work into modules
* Optimized data strcuture usage
* Avoid hard coding and support configuration support

## Extension
* Add Ros nodelet support to avoid extra copying cost
* Add map management code
* Add relocalization module
* Refer to Google Cartographer for some modules

## Prerequisites

### Hardware
* A Lidar
* A movable power source.
* A mobile computing platform if you want to run the algorithm in real time

### Software

* ROS Kinetic or Later.
* PCL, g2o, Eigen


## Build (this Module Only)
```
git clone https://github.com/ZhekaiJin/the-Cooper-Mapper.git
cp the-Cooper-Mapper/smartbot/L_SLAM/ your_ros_working_space/src
cd your_ros_working_space
catkin_make -j4
```

## Versioning

This work use [SemVer](http://semver.org/) for versioning. This repo now contains version 1.0.

## Acknowledgments

* **Ji Zhang** and **Sanjiv Singh** - *LOAM* - [LOAM](https://www.ri.cmu.edu/publications/loam-lidar-odometry-and-mapping-in-real-time/)

## Authors
**Zhekai Jin**

## License

This project is licensed under the MIT License - see the [LICENSE](../../LICENSE) file for details.
