# MAVS Examples
The MAVS provides several example codes

- [Examples](#Examples)
  * [Actor Example](##actor_example)
  * [Camera Example](##camera_example)
  * [Forester Example](##forester_example)
  * [Free Camera Example](##free_camera_example)
  * [HMF Read/Write](##hmf_read_write_example)
  * [Lidar Example](##lidar_example)
  * [Multiple Vehicles](##multi_vehicle_example)
  * [Radar Example](##radar_example)
  * [Simulation Example](##simulation_example)
  * [Batch Halo Sim](##batch_halo_sim)
  * [Halo Lidar Trainer](##halo_lidar_trainer)
  * [Halo Car Simulation](./MavsHaloCarExample.md)
  * [Lidar Roughness Analyzer](./MavsLidarRoughnessExample.md)
- [Unit Tests](#UnitTests)
  * [Solar Position](##utest_mavs_solar_position)
  * [Chrono Vehicle](##utest_mavs_chrono_vehicle)
  * [Camera Distortion](##utest_mavs_camera_distortion)
  * [Camera](##utest_mavs_camera)
  * [GPS](##utest_mavs_gps)
  * [LIDAR](##utest_mavs_lidar)
  * [RADAR](##utest_mavs_radar)
  * [Spherical Camera](##utest_mavs_spherical_camera)

# Examples

## actor_example
An actor is any dynamic (movable) object in MAVS. This code provides an example of an actor in a MAVS simulation

Usage: 
``` shell
> ./actor_example scene.json actor.json
```

Where "scene.json" examples can be found in *mavs/data/scenes* and "actor.json" examples found in *mavs/data/actors*. The example will create two windows. One window is a camera simulation that automatically follows the hmmwv actor. The other is a free camera window that can be used to move through the scene with the WASD, PageUp, PageDown, and Arrow keys.

## batch_halo_sim
This is an example of a MAVS batch simulation for generating labeled training data. 

Usage:
``` shell
>./batch_halo_simulation (hotstart_num)
```

"hotstart_num" is an optional parameter that will restart the batch sim at a given frame. There are 10,725 frames of output, with each frame saving multiple image, lidar, and annotation files.

## camera_example
An example MAVS RGB camera sensor

Usage: 
``` shell
>./camera_example scene.json env.json camera.json
```
Where "scene.json" examples can be found in *mavs/data/scenes* and "camera.json" examples found in *mavs/data/sensors/cameras*

The output will be a camera window that moves through the scene in the X (east) direction.

## forester_example
An example sensor placement analysis with MAVS. Finds the coverage statistics of two LIDAR rotated through various mounting angles on the front of a vehicle

Usage: 
``` shell
>./forester_example sensor_inputs.json
```

The file "sensor_inputs.json" is specific to this example, and a sample can be found in *mavs/data/sims/misc_sims/forester_sim.json*.

## forester_example_simple
Similar to *forester_example*, but with simplified analysis. 

Usage: 
``` shell
>./forester_example_simple sensor_inputs.json
```

The file "sensor_inputs.json" is specific to this example, and a sample can be found in *mavs/data/sims/misc_sims/forester_sim.json*.

## hmf_read_write_example
Example that demonstrates how MAVS can be used to load and write out applanix heightmap files (.hmf).

Usage: 
``` shell
>./hmf_read_write_example heightmap.hmf
```

The file "heightmap.hmf" is a binary input file in the Applanix .hmf format. An example can be found in *mavs/data/hmf_files*.

The code will load in the hmf file, then write it back out. A window displaying the hmf file will pop up, close the window to finish the program.

## free_camera_example
Example of a user-controllable camera view of a MAVS scene.

Usage: 
``` shell
>./free_camera_example scene.json
```
Where "scene.json" examples can be found in *mavs/data/scenes* 

Fly the camera around the scene with the W-A-S-D keys. Page Up & Page Down move the camera up and down. Arrow keys rotate the view (left,right,up,down). Home and End keys rotate the roll of the camera. Close the view window to finish the program.

## halo_lidar_trainer
Generates labeled lidar data in a given scene along a given path.

Usage: 
``` shell
>./halo_lidar_trainer scene.json anvel_replay.vprp
```
Where "scene.json" examples can be found in *mavs/data/scenes* and "anvel_replay.vprp" is an ANVEL replay file in text format, examples in *mavs/data/waypoints*.

The output of the example is annotated lidar data.

## lidar_example
Demonstrates a MAVS lidar simulation.

Usage: 
``` shell
>./lidar_example scene.json (lidar_num) (rain_rate)
```
Where "scene.json" examples can be found in *mavs/data/scenes*.

*lidar_num* is optional and specifies the type of lidar sensor.

* 291 = SICK LMS-291
* 8 = Quanergy M8
* 16 = Velodyne VLP-16
* 32 = Velodyne HDL-32E
* 64 = Velocyne HDL-64E

The default LIDAR is the SICK LMS-291.

*rain_rate* is an optional parameter and specifies the rain rate in mm/h. Typical values are 2.5-25.0.

The simulation will save several files

* *scene_stats.txt* - specifies the number of triangles in the scene
* *lidar_output.bmp* - A top-down rendering of the point cloud
* *lidar_sensor00001_annotated.txt* - A space-delimited column file of the point cloud (x,y,z,i,label_num)
* *lidar_sensor00001_annotated.csv* - A comma-delimited file listing the objects that were detected in the point cloud and their extent

## radar_example
Demonstrates a MAVS radar simulation.

Usage: 
``` shell
>./radar_example mavs_scene_file.json
```
Where "scene.json" examples can be found in *mavs/data/scenes*.

The simulation will save two files, "camera.bmp" and "radar.bmp". "camera.bmp" shows the view from the position of the radar. "radar.bmp" shows the radar output with detected targets in yellow.

## simulation_example
Demonstrates a MAVS closed-loop autonomy simulation with vehicle, driver, and sensors in the loop.

Usage: 
``` shell
>./simulation_example scene.json (lidar_num)
```
or, in the case where the code was built with MPI enabled
``` shell
>mpirun -np 6 ./simulation_example scene.json (lidar_num)
```
Where "scene.json" examples can be found in *mavs/data/scenes*. When using MPI, the simulation requires at least 6 processors to run.

*lidar_num* is optional and specifies the type of lidar sensor.
* 291 = SICK LMS-291
* 8 = Quanergy M8
* 16 = Velodyne VLP-16
* 32 = Velodyne HDL-32E
* 64 = Velocyne HDL-64E

## multi_vehicle_example
Demonstrates how to use MAVS MPI framework to simulate multiple vehicles.

Usage: 
``` shell
>./multi_vehicle_example scene.json
```
or, in the case where the code was built with MPI enabled
``` shell
>mpiexec -np 8 ./multi_vehicle_example scene.json
```
Where "scene.json" examples can be found in *mavs/data/scenes*. When using MPI, the simulation requires at least 6 processors to run.

When using the MPI version of the code, this example requires 8 processors to run. 

The code will display to camera windows of separate vehicles moving through the simulated scene.

# UnitTests

## utest_mavs_solar_position
Usage: 
``` shell
>./utest_mavs_camera
```

The correct output is a camera frame, rotating around a simple scene with a red sphere, a yellow box, and a green surface.

Press Ctrl+C to stop the simulation. 

## utest_mavs_chrono_vehicle
Unit test to evaluate the MAVS-Chrono vehicle interface.

Usage: 
```
>./utest_mavs_chrono_vehicle chrono_vehicle_input_file.json
```
where "chrono_vehicle_input_file.json" gives the path to the chrono data directory and the different vehicle configurations to use. An example can be found in "mavs/data/vehicles/chrono_inputs/hmmwv_windows.json".

The simulation will run for 15 seconds of simulated time, printing a state update every second. The vehicle drives in a straight line and should reach a maximum speed of around 38.9 m/s.

## utest_mavs_camera_distortion
Usage: 
``` shell
>./utest_mavs_camera_distortion
```
Creates a distortion model, then prints the undistorted and distorted positions of those pixels.

The correct output is:
```
Undistorted.u Undistorted.v Distorted.u Distorted.c
0 0 23.3065 18.3965
0 483 23.6183 466.037
603 0 581.996 18.3743
603 483 582.06 466.059
302 242 303.22 242.83
```

## utest_mavs_camera
Usage: 
``` shell
>./utest_mavs_camera
```
The correct out put is a camera frame, rotating around a simple scene with a red sphere, a yellow box, and a green surface.

Press Ctrl+C to stop the simulation.

## utest_mavs_gps
Usage: 
``` shell
>./utest_mavs_gps
```

Creates a differential and dual-band GPS and runs two iterations of each sensor. In the first simulation the GPS sensors are run in completely "open-field" conditions with no satellite occlusion or multipath errors. In the second, a simple scene with buildings surrounding the GPS units is created, causing multipath and dilution of precision errors. Correct errors should be on the order of 10-100 centimeters for the dual band GPS and a few centimeters for the differential gps.

## utest_mavs_lidar
Usage: 
``` shell
>./utest_mavs_lidar
```
If working correctly, program should show a simple scene with a red sphere, a green floor, and a yellow box. Smoke will rise up from the floor in front of the sphere. A top-down rendering of lidar scan will appear in a second window.

Press Ctrl+C to kill the simulation.

## utest_mavs_radar
Usage: 
``` shell
>./utest_mavs_radar
```

If working correctly, should show a simple scene with several columns. A camera and a radar will rotate through the scene, and the radar returns will show up in yellow.

## utest_mavs_spherical_camera
Usage: 
``` shell
>./utest_mavs_spherical_camera scene.json
```
Where "scene.json" examples can be found in *mavs/data/scenes*.

The result is a rendering of a spherical projection of the input scene. Two files will be saved, "simple_render.bmp" and "spherical_projection.bmp"