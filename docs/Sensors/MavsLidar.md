# MAVS Lidar Model
The lidar sensor can also be accessed through the [python interface](../Interfaces/MavsPython.md).

## Creating a MAVS Lidar
A lidar can be created in one of three ways. First, one of the built-in lidar models can be used.
``` c++
mavs::sensor::lidar::Lms291_S05 s05;
mavs::sensor::lidar::Vlp16 vlp;
mavs::sensor::lidar::Hdl32E hdl32;
mavs::sensor::lidar::Hdl64E hdl64;
mavs::sensor::lidar::MEight m8;
mavs::sensor::lidar::OusterOS1 os1;
mavs::sensor::lidar::OusterOS2 os2;
mavs::sensor::lidar::Rs32 rs32;
```

Secondly, a lidar can be created by explicitly setting the scan properties. See the [lidar API documentation](https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/classmavs_1_1sensor_1_1lidar_1_1_lidar.html#a547d68786e402bdc76f01e1f7133a285) for more information.
``` c++
mavs::sensor::lidar::Lidar lidar;
lidar.SetScanPattern(-180.0f,179.0,1.0f,-15.8f,15.8f,1.0f);
```

A lidar can also be created by loading a lidar input file.
``` c++
mavs::sensor::lidar::Lidar lidar;
lidar.Load("path/to/mavs/data/sensors/LMS291.json");
```

An example input file is
``` json
{
  "Scan Pattern": {
    "Horizontal Range": [-180,179.75],
    "Horizontal Step": 0.25,
    "Vertical Range": [-15.8,15.8],
    "Vertical Step": 1.0
  },

  "Mode": 1,

  "Beam Properties":{
    "Shape": "circle",
    "Divergence": [0.012,0.012]
  },

  "Max Range": 80.0,
  "Min Range": 0.0
}
```
The parameters are defined below. Not that these parameters can also be set through the API.
* Max Range: The maximum range of the lidar in meters
* Min Range: The minimum range of the lidar in meters
* Beam Properties:
  * Shape: Can be "circle", "rectangle", or "ellipse"
  * Divergence: The beam divergence in the horizontal and vertical planes, in radians/m
* Scan Pattern:
  * Horizontal Range: 
  * Horizontal Step:
  * Vertical Range: 
  * Vertical Step:
* Mode: The return mode of the lidar, as an integer value from 0-3. The default mode is 1
  * 0: Average return distance
  * 1: strongest return
  * 2: last return
  * 3: strongest and last return

## Example Lidar Simulation
The following example ilustrates many of the features of the MAVS lidar simulation. More explanation of the function calls can be found in the [lidar API documentation](https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/classmavs_1_1sensor_1_1lidar_1_1_lidar.html).
``` c++
// Include a ray tracer and lidar model
#include "raytracers/simple_tracer/simple_tracer.h"
#include "sensors/lidar/vlp16.h"

int main (int argc, char *argv[]){
  // Create a temple test scene
  mavs::raytracer::SimpleTracer scene;
  scene.CreateTestScene();
  // Create an environment and add the scene to it
  mavs::environment::Environment env;
  env.SetRaytracer(&scene);
  // Specify the pose / state of the sensor
  glm::vec3 sens_pos(-25.0f, 0.0f, 10.0f);
  glm::vec3 sens_vel(1.0f, 0.0f, 0.0f;
  glm::quat sens_orientation(1.0f, 0.0f, 0.0f, 0.0f);
  // Create the lidar sensor to simulate
  mavs::sensor::lidar::Vlp16 vlp;
  // Specify the time step. 
  //Should be the revolution time of the sensor
  float dt = 0.1f;
  // Set the position & orientation of the sensor
  lidar->SetPose(sens_pos, sens_orientation);
  // Set the velocity of the sensor
  // This will automatically calculate movement during the scan
  lidar->SetVelocity(sens_vel.x, sens_vel.y, sens_vel.z);
  // Update the sensor and point cloud
  lidar->Update(&env, dt);

  // display a top-down view of the point cloud
  // points will be rgb colorized
  // options are "color", "range", "height", "intensity", and "white"
  lidar->SetDisplayColorType("color");
  lidar->Display();
  // Now display perspective view by height
  lidar->SetDisplayColorType("height");
  lidar->DisplayPerspective();
  // Save the point cloud to a labeled PCD file
  lidar->AnnotateFrame(&env,true);
  lidar->WritePcdWithLabels("labeled_points.pcd);
  return 0;
}
```


## Updating the MAVS Lidar
