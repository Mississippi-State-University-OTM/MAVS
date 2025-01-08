# MAVS - ROS

There are two separate packages for interfaceing MAVS with ROS, The [mavs_ros](https://github.com/CGoodin/mavs_ros) package for ROS-1 and the [mavs-ros2](https://github.com/CGoodin/mavs-ros2) package for ROS2. These are described in more detail below.

## *mavs_ros* Package for ROS 1
The MAVS-ROS interface wraps certain MAVS simulation elements like lidar and camera sensors in ROS-nodes and has them publish data to a ros network. Mavs-ROS is tested on Ubuntu 16.04 with ROS-Kinetic Kame. 

The following discussion assumes the user has some prior knowledge about ROS and the catkin build system.

### Building the mavs_ros package
The following instructions assume you have already [built and installed MAVS](wiki-page:Building MAVS) on your system. 

In the catkin_ws/src directory, clone the mavs_ros package using the command
```
git clone https://github.com/CGoodin/mavs_ros
```

When this is complete, you can cd to your catkin_ws directory and build the package with the *catkin_make* command.

### Running the example
The mavs_ros package build several nodes including *mavs_sensor_node* and *mavs_vehicle_node*. The file *mavs_ros/launch/mavs_sim.launch* demonstrates how these nodes can be launched to run a simulation in which the user inputs driving commands with the keyboard while a camera and lidar sensor attached to the vehicle publish data to the ROS network.

To run the example, first edit the file *mavs_ros/launch/mavs_sim.launch* to include the path to the simulated scene and vehicle you wish to use. 

When this is complete, from the terminal, type 
```
$ roslaunch mavs_sim.launch
```

## *mavs-ros2* package for ROS2
The mavs-ros2 package is documented on github, see the [README](https://github.com/CGoodin/mavs-ros2#readme) and [wiki](https://github.com/CGoodin/mavs-ros2/wiki) for mor information.
