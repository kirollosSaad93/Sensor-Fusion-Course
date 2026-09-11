# Sensor Fusion Self-Driving Car Course

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/ObstacleDetectionFPS.gif" width="700" height="400" />

### Welcome to the Sensor Fusion course for self-driving cars.

In this course we will be talking about sensor fusion, whch is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. we will mostly be focusing on two sensors, lidar, and radar. By the end we will be fusing the data from these two sensors to track multiple cars on the road, estimating their positions and speed.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing lidar's high resoultion imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.


## Installation

### Linux Ubuntu 16

Install PCL, C++

The link here is very helpful, 
https://larrylisky.com/2014/03/03/installing-pcl-on-ubuntu/

A few updates to the instructions above were needed.

* libvtk needed to be updated to libvtk6-dev instead of (libvtk5-dev). The linker was having trouble locating libvtk5-dev while building, but this might not be a problem for everyone.

* BUILD_visualization needed to be manually turned on, this link shows you how to do that,
http://www.pointclouds.org/documentation/tutorials/building_pcl.php


## Obstacle detection project

The recorded PCD stream runs voxel downsampling and region/roof filtering,
custom three-point 3D RANSAC road segmentation, and custom Euclidean clustering
using the balanced 3D KD-Tree in `src/kdtree.h`. Each accepted cluster receives
one axis-aligned bounding box. PCL is used for point-cloud IO, filtering, and
visualization; segmentation and clustering do not call PCL algorithms.

Build and test with a C++14 compiler, CMake, Make, PCL, and Boost installed:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
make -C build -j2
ctest --test-dir build --output-on-failure
./build/environment
```

The default viewer loops through `data_1`. Supply another PCD directory as an
argument to use a different stream. For a single pass without a display:

```sh
./build/environment --headless src/sensors/data/pcd/data_1
```

The tests check KD-Tree radius searches against brute force, plane segmentation,
3D cluster membership and size limits, bounding-box bounds, and processing of
both included streams. Visual review is still needed to judge object identity
and bounding-box continuity; cluster IDs are local to each frame.
