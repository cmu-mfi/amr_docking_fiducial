# AMR Docking Fiducial

## Installation

### Dependent packages
`docking_with_fiducial` depends on following packages:
* [realsense-ros](https://github.com/IntelRealSense/realsense-ros/tree/ros2-development)
* [aruco_ros](https://github.com/pal-robotics/aruco_ros)

```
$ cd ros2_ws/src
$ git clone -b humble-devel https://github.com/pal-robotics/aruco_ros.git
$ git clone -b ros2-development https://github.com/IntelRealSense/realsense-ros.git
$ git clone https://github.com/cmu-mfi/amr_docking_fiducial.git
$ cp docking_with_fiducial/extras/docking_markers.launch.py aruco_ros/aruco_ros/launch/
$ cd ../
$ colcon build --symlink-install
```

### Compile main package
```
colcon build --symlink-install --packages-select docking_with_fiducial
```

## Launch

```
ros2 launch docking_with_fiducial docking_with_markers.launch.py namespace:=robot1
```
