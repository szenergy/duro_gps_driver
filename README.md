# Duro Inertial GPS Driver
## Overview

This is a C++ ROS driver for Swiftnaw Duro Inertial (Piksi Multi Inertial) GPS / GNSS Receivers. The code is based on offical Swiftnav libswiftnav C example and Alex Hajnal's UTM converter code.

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
cd ~; mkdir git; cd git     # eg create a git folder, the forder name can be different
git clone https://github.com/swift-nav/libsbp.git
cd libsbp/c/
git submodule update --init --recursive
mkdir build; cd build
cmake ../
make
sudo make install
```
This will create some files in `/usr/local/include/libsbp/` and in `/usr/local/lib`.

### 2. step
Catkin build `duro-ros`.
Todo: @rudolfkrecht

## Settings 
Enable MSG ID 544 and 545 in swift console. These orientation messages are not enabled in default configuration.
The MSG ID is defined in the headers, e.g. `#define SBP_MSG_ORIENT_QUAT 0x0220` which is decimal `544`.
Todo: @rudolfkrecht

## Run
Todo: @rudolfkrecht
E.g:
```
rosrun duro-gps duronode _address:=192.168.0.222 _port:=55555
rosrun duro-gps duronode _address:=192.168.1.222 _port:=55555
rosrun duro-gps duronode
```

## Topics
Todo: @rudolfkrecht
An important topic is `/gps/duro/current_pose` which is `geometry_msgs/PoseStamped` type in UTM (https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system) eg:

``` c
header: 
  seq: 554S
  stamp: 
    secs: 1580388036
    nsecs: 641742448
  frame_id: ''
pose: 
  position: 
    x: 697214.762607
    y: 5285721.97968
    z: 157.104031829
  orientation: 
    x: 0.00819693645462
    y: 0.00343747669831
    z: 0.692575566471
    w: 0.721290563233

```

## Further reading:
- Libswiftav: https://github.com/swift-nav/libswiftnav
- ETH python Piksi ROS drivers: https://github.com/ethz-asl/ethz_piksi_ros