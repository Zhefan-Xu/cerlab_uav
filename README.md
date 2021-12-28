# cerlab_uav
This package is for cerlab UAV team's autonomous drone. 


## Install Package
IMPORTANT! If you have not install [Octomap](http://wiki.ros.org/octomap_server), [Realsese_Camera ROS](https://github.com/IntelRealSense/realsense-ros), and [MAVROS with PX4](https://docs.px4.io/master/en/ros/mavros_installation.html), please follow the link for installation.

After installing prerequsite packages, install this package:
```
cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/cerlab_uav

cd ~/catkin_ws
catkin_make
```

## Usage
NOTE: Please set EKF2_AID_MASK to the approriate value (e.g. 280 for VIO).

#### I.Launch dual camera (l515 and t265) with mavros and octomap
```
roslaunch cerlab_uav mavros_l515_and_t265_octomap.launch
```


By far, you can enter [position mode](https://docs.px4.io/v1.11/en/flight_modes/position_mc.html) and control by transmitter to fly stably.



#### V. Simulation
Please follow the [PX4 instruction](https://docs.px4.io/master/en/simulation/ros_interface.html) to install simulation.
To launch simulation (you can change parameters and arguments in the file),
```
roslaunch cerlab_uav uav_simulation
```

