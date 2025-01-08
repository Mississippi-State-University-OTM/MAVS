# MAVS Autonomy

MAVS comes with some built-in autonomy algorithms for testing your autonomous vehicle code. These algorithms are provided for convenience to enable users to test autonomous behaviors without the need for third-party code such as ROS. 

 The algorithms are split into three basic parts - perception, planning, and control.

## Perception
The MAVS perception algorithm uses 3D lidar data to populate an occupancy grid. The grid uses a slope map to identify obstacles in the terrain [\[Manduchi 2003\]](https://idp.springer.com/authorize/casa?redirect_uri=https://link.springer.com/content/pdf/10.1023/B:AURO.0000047286.62481.1d.pdf&casa_token=oeQzbS-LY1AAAAAA:eMaMozt0eR5_Sa7jjVpi7cjEnhbbJ_JCOFhYnxSxJgjTLCwe0PhNbqd7GB29q1IWWn26CFBO9YmBdRV7TDE). The parameters of the model are the resolution and size of the grid as well as the threshold for flagging a slope as an obstacle. 

The MAVS perception algorithm is implemented in python. The source can be found in *mavs/src/mavs_python/examples/autonomy/perception.py*.

## Path Planning
There are four path planning algorithms available in the MAVS Autonomy. All the algorithms are modified from the [PythonRobotics library](https://github.com/AtsushiSakai/PythonRobotics) by Atsushi Sakai. The algorithms are modified have consistent inputs and outputs so that they can be easily interchanged. The available algorithms are.

* RRT
* RRT*
* A*
* Potential field 

## Vehicle Control
The vehicle control module uses the pure-pursuit algorithm [Coulter 1992](https://apps.dtic.mil/sti/pdfs/ADA255524.pdf), a steering algorithm for Ackermann vehicles that takes the vehicle wheelbase into account. The throttle is set by a [PID controller](https://en.wikipedia.org/wiki/PID_controller#:~:text=A%20proportional%E2%80%93integral%E2%80%93derivative%20controller,applications%20requiring%20continuously%20modulated%20control). which is trying to reach a controlled speed.