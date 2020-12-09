# Duro Inertial GPS Driver
## Overview

This is a C++ ROS driver for Swiftnaw Duro Inertial (Piksi Multi Inertial) GPS / GNSS Receivers. The code is based on offical Swiftnav libswiftnav C example and <del>Alex Hajnal's</del> Apollo UTM converter code. **Note** that orientation data is produced by Duro Inertial but not produced by Piksi Multi or Duro.

## Scope
The current version supports *only* (not tested elsewhere):
- Ubuntu 18.04
- ROS Melodic
- Catkin tools https://catkin-tools.readthedocs.io/
- Only ethernet version (no serial or USB support)

## Install
### 1. step
Install libsbp (Swift binary protocol library) C client library from GitHub: https://github.com/swift-nav/libsbp
It is detailed in github, but the main steps are:
```
sudo apt-get install build-essential pkg-config cmake doxygen check
cd ~; mkdir git; cd git     # eg create a git folder, the folder name can be different
git clone https://github.com/swift-nav/libsbp.git
cd libsbp
git checkout fe7b78992fb87eef5bd6d12b2daf70f4ac90bc39
cd c
git submodule update --init --recursive
mkdir build; cd build
cmake ../
make
sudo make install
```
This will create some files in `/usr/local/include/libsbp/` and in `/usr/local/lib`.

### 2. step
Clone the duro_gps_driver package into your Catkin workspace. 
Navigate to the root of your Catkin workspace. Source your setup.bash file. Build the `duro_ros` package using Catkin:
```
catkin build duro_ros
source devel/setup.bash
```

## Settings 
Enable MSG ID 544 and 545 in swift console. Once again, orientation data not produced by Piksi Multi or Duro. These orientation messages are not enabled in default configuration.
The MSG ID is defined in the headers, e.g. `#define SBP_MSG_ORIENT_QUAT 0x0220` which is decimal `544`.

## Run
Make sure that `roscore` is running. 
The `duro-gps` driver can be run using the `rosrun` command. It is necessary to provide your device's IP address and port number. 
E.g:
```
rosrun duro_ros duronode _ip_address:=192.168.1.10 _port:=55555
```
Alternatively you can use a [launch](launch/duro_example.launch) file. E.g:
```
roslaunch duro_ros duro_example.launch
```

## Topics
`duro_ros duronode` publishes the following topics and [types]:
```
/gps/duro/current_pose    [geometry_msgs/PoseStamped]
/gps/duro/fix             [sensor_msgs/NavSatFix]
/gps/duro/imu             [sensor_msgs/Imu]
/gps/duro/mag             [sensor_msgs/MagneticField]
/gps/duro/odom            [nav_msgs/Odometry]
/gps/duro/rollpitchyaw    [geometry_msgs/Vector3]
/gps/duro/status_flag     [std_msgs/UInt8]
/gps/duro/status_string   [std_msgs/String]
```
An important topic is `/gps/duro/current_pose` which is `geometry_msgs/PoseStamped` type in UTM (https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system) eg:

``` c
header: 
  seq: 554S
  stamp: 
    secs: 1580388036
    nsecs: 641742448
  frame_id: 'map'
pose: 
  position: 
    x: 697214.762607
    y: 5285721.97968
    z: 0.0
  orientation: 
    x: 0.00819693645462
    y: 0.00343747669831
    z: 0.692575566471
    w: 0.721290563233

```

## Further reading:
- Libswiftav: https://github.com/swift-nav/libswiftnav
- ETH python Piksi ROS drivers: https://github.com/ethz-asl/ethz_piksi_ros
