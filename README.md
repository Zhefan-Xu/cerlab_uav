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

#### I.Launch dual camera (D435 and T265)
```
roslaunch cerlab_uav uav_d435_and_t265.launch
```

#### II. Setup the communication between computer and FCU (PLEASE fully understand how to setup px4.launch file from their [instructions](https://docs.px4.io/master/en/ros/mavros_installation.html))
```
roslaunch mavros px4.launch
```

By far, you can enter [position mode](https://docs.px4.io/v1.11/en/flight_modes/position_mc.html) and control by transmitter to fly stably.





The followings are for the autonomous flight usage:
#### III. Autonomous Takeoff (Take off and keep 0.5m height)
```
roslaunch cerlab_uav offboard_node
```

#### IV. Autonomous Exploration
```
rosrun cerlab_uav deplanner_node
rosrun cerlab_uav publish_goal
```

#### V. Simulation
Please follow the [PX4 instruction](https://docs.px4.io/master/en/simulation/ros_interface.html) to install simulation.
To launch simulation (you can change parameters and arguments in the file),
```
roslaunch cerlab_uav uav_simulation
```

