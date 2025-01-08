# MAVS Vehicles
There are two options for vehicle simulation in MAVS. The default option is the MAVS built-in vehicle simulation based on the [ReactPhysics3D dynamics engine](https://www.reactphysics3d.com/). The other option is to use the vehicle simulation capability in the [Project Chrono Vehicles library](https://projectchrono.org/). Only wheeled vehicles are currently available in MAVS.

## Default MAVS Vehicles
Examples for MAVS vehicle input files can be found in the MAVS data directory under *vehicles/rp3d_vehicles*. The vehicle suspension model is a lumped parameter, indpenedent spring-damper approximation, while the tires are also modeled as linear spring-dampers. The parameters are described in the following subsections, along with example input file sections.

When using the default MAVS vehicle model, the [MAVS-VTI model](./mavs_vti.md) and [MAVS radial tire model](./mavs_radial_tire.md) will be used to calculate tire and terrain forces.

### Chassis
The chassis is treated as a single rigid, to which each suspension element is attached. The chassis is defined by the following paramters:

* Sprung Mass - in kilograms
* CG Offset - The distance in meters from the top of the suspension elements to the center of gravity
* Dimensions - The dimensions of the cuboid defining the chassis inertial properties. This is equivelent to defining the moments of inertia about the x-y-z axes for a [solid cuboid](https://en.wikipedia.org/wiki/List_of_moments_of_inertia).
``` json
"Chassis": {
  "Sprung Mass": 1587.6,
  "CG Offset": 0.3,
  "Dimensions": [ 1.795, 1.45, 1.245 ]
},
```

### Powertrain
The powertrain is modeled using simple rpm vs torque model. The parameters are 

* Final Drive Ratio - The gear ratio between the transmission and wheels
* Max Engine Torque - The max output of the engine in N/m
* Max Engine Rpm - The max engine speed in revolutions per minute
* Max Braking Torque - The maximum applied braking torque
* Idle Rpm - The engine speed at idle. 
``` json
"Powertrain": {
  "Final Drive Ratio": 1.0,
  "Max Engine Torque": 500.0,
  "Max Engine Rpm": 6000.0,
  "Max Braking Torque": 600.0,
  "Idle Rpm": 1000.0
}
```

### Axles
Axles are listed in an input json array, with an example of a two-axle vehicle shown below. Any number of axles can be listed, and each axle can be either steered or unsteered, powered or unpowered. Each axle must have the following parameters.

* Longitudinal Offset - Distance in meters from the CG to the center of the axle. If the axle is in front of the CG, the value is positive. If it is behind the CG, the value is negative.
* Track Width - Distance in meters from the center of the left tire to the center of the right tire.
* Spring Constant - The linear spring coefficient of the suspension, in N/m
* Damping Constant - The linear damping coefficient of the suspension, in N*s/m
* Spring Length - The rest length of the suspension spring, in meters
* Steered - true if the axle is steered, false if not
* Powered - true if the axle is powered, false if not
* Max Steer Angle - Maximum steer angle in degrees. If the axis is not steered, this entry can be set to zero or removed.
* Unsprung Mass - The mass of the suspension element and tire, in kg
* Tire properties:
  * Spring Constant - The linear spring coefficient of the tire, in N/m
  * Damping Constant - The linear damping coefficient of the tire, in N*s/m
  * Radius - The undeflected radius of the tire, in meters
  * Width - Tire section width in meters
  * Section Height - Tire section height in meters
  * High slip crossover angle - Angle that defines where the tractive force versus side slip function "turns over", ie the angle where steering physics transisitions from low slip to high slip conditions.
``` json
"Axles": [
  {
    "Longitudinal Offset": 1.19,
    "Track Width": 1.54,
    "Spring Constant": 37044.0,
    "Damping Constant": 1058.4,
    "Spring Length": 0.3,
    "Steered": true,
    "Powered": true,
    "Max Steer Angle": 35.0,
    "Unsprung Mass": 62.45,
    "Tire": {
      "Spring Constant": 291060.0,
      "Damping Constant": 169344.0,
      "Radius": 0.341,
      "Width": 0.15,
      "Section Height": 0.07,
      "High Slip Crossover Angle": 5.0
    }
  },
  {
    "Longitudinal Offset": -1.408,
    "Track Width": 1.54,
    "Spring Constant": 40219.2,
    "Damping Constant": 1164.24,
    "Spring Length": 0.3,
    "Steered": false,
    "Powered": true,
    "Unsprung Mass": 62.45,
    "Tire": {
      "Spring Constant": 291060.0,
      "Damping Constant": 169344.0,
      "Radius": 0.341,
      "Width": 0.15,
      "Section Height": 0.07,
      "High Slip Crossover Angle": 5.0
    }
  }
]
```

### Mesh
The mesh entry defines the visualization that will be associated with the vehicle. The mesh should be an obj file located in the *scenes/meshes* subdirectory of the MAVS data folder. The following parameters describe the vehicle.

* File - The name of the .obj file
* Rotate Y to Z, X to Y, and Y to X - these three parameters specify if the mesh is in the default MAVS frame of X=forward, Z-up. If not, set these to true as necessary
* Offset - If the mesh is not centered at the Chassis CG, it will need to be translated so the vehicle visualization aligns with the physics simulation
* Scale - Scale the mesh if it is not in the appropriate units
``` json
"Mesh": {
  "File": "forester_vehicle_cg.obj",
  "Rotate Y to Z": false,
  "Rotate X to Y": false,
  "Rotate Y to X": false,
  "Offset": [ -0.004, 0.0, -0.62 ],
  "Scale": [ 1.0, 1.0, 1.0 ]
},
```

### Initial Pose
You can optionally define and initial position and orientation in world coordinates for the vehicle. The Position is in East-North-Up meters, while the orientation is a normalized W-X-Y-Z quaternion.
``` json
"Initial Pose": {
  "Position": [ 0.0, 0.0, 0.0 ],
  "Orientation": [ 1.0, 0.0, 0.0, 0.0 ]
}
```

## Chrono Vehicles
Example chrono vehicle inputs can be found in the MAVS data folder under *vehicles/chrono_inputs*. The input file simply specifies which Chrono input files to use. The Chrono library must be cloned and [built separately](../MavsBuildInstructions).

All file paths listed in the json input are relative to the *chrono/data/vehicle* installation directory.

* Vehicle File - Path to the Chrono vehicle input file
* Terrain File - Path to the Chrono terrain input file
* Powertrain File - Path to the Chrono powertrain input file
* Tire File - Path to the Chrono tire input file
``` json
{
  "Vehicle File": "generic/vehicle/Forester_Vehicle.json",
  "Terrain File": "terrain/RigidPlane.json",
  "Powertrain File": "generic/powertrain/SimplePowertrain.json",
  "Tire File": "generic/tire/RigidTire.json"
}
```