# MAVS Simulation Input Files
MAVS primarily uses [json input files.](https://www.json.org/) These files describe everything from the scene geometry to the simulation time step. This page documents the simulation input files and their contents.

There are two primary input file types: Input to autonomy simulations and input to sensor simulations. Some of the input fields in these files are paths to other json files, which will be discussed below.

## Autonomy Simulation Inputs
An autonomy simulation contains a single vehicle with one or more attached sensors operating in a digital terrain. 

### Scene 
``` json
"Scene":{
		 "Input File":"/media/sf_vm_shared/mavs/data/scenes/grass_maze.json",
		 "Origin": [32.3526,90.8779,73.152],
		  "Time Zone":6
	}
```
* "Input File": Full path to the json input file defining the scene geometry. See the "Scene Input File" section below for details. 
* "Origin": The latitude, longitude, and altitude of the origin of the scene ([0,0,0] in East-North-Up coordinates) in decimal degrees and meters. Postive = Northing / Westing. For more information, see the [MAVS Coordinate System page](./MavsCoordinateSystem).
* "Time Zone": The time zone of the scene, as an integer offset from GMT. Positive = west, such that CST offset is 6.

### Driver
``` json
"Driver":{ 
		 "Type":"A* Planner",
		 "Input File": "/media/sf_vm_shared/mavs/data/waypoints/example_waypoints.json",
		 "Number Processors":1,
		 "Update Rate":10.0
	}
```

### Vehicle
``` json
"Vehicle":{
		"Type":"Rp3d",
		 "Input File":"full_path_to_vehicle.json",
		 "Number Processors":1,
		 "Initial Position":[0.0,0.0,0.0],
		 "Initial Orientation":[1.0,0.0,0.0,0.0],
		 "CG Height":2.0,
		 "Update Rate":20000.0
	}
```
* "Type": The type of simulated vehicle. The allowed types are "Rp3d", and "Chrono"
    - "Rp3d": A lumped parameter vehicle dynamics implementation
    - "Chrono": Vehicle simulated using the [Chrono Vehicle simulator](https://projectchrono.org/)
* "Input File": The input file associated with the vehicle model. Not that this is currently only used with the Chrono vehicle. 
* "Number Processors": This should typically be 1.
* "Initial Position": The initial position of the vehicle in world ENU coordinates. 
* "Initial Orientation": Quaternion representing the initial orientation of the vehicle in world ENU coordinates.
* "CG Height": The height of the vehicle CG above the ground in the static state. Used by the Car, Bicycle, and Turtle models.
* "Update Rate": Frequency with which to step the vehicle simulation, in Hz. 

### Sensors
``` json
"Sensors": [
		{"Type": "camera", 
		 "Model":"XCD-V60", 
		 "Offset": [0.0, 0.0, 0.0],
		 "Orientation":[1.0, 0.0, 0.0, 0.0],
		 "Repitition Rate (Hz)":10.0, 
		 "Name":"camera", 
		 "Number Processors":1
		},

		{"Type": "gps", 
		 "Input File":"gps", 
		 "Offset": [0.0, 0.0, 0.0],
		 "Orientation":[1.0, 0.0, 0.0, 0.0],
		 "Repitition Rate (Hz)":10.0, 
		 "Name":"gps", 
		 "Number Processors":1
		},

		{"Type": "compass", 
		 "Input File":"compass", 
		 "Offset": [0.0, 0.0, 0.0],
		 "Orientation":[1.0, 0.0, 0.0, 0.0],
		 "Repitition Rate (Hz)":10.0, 
		 "Name":"compass", 
		 "Number Processors":1
		},

		{"Type": "lidar", 
		 "Model":"HDL-32E", 
		 "Offset": [0.0, 0.0, 0.0],
		 "Orientation":[1.0, 0.0, 0.0, 0.0],
		 "Repitition Rate (Hz)":10.0, 
		 "Name":"lidar", 
		 "Number Processors":1
		}
	]
```

### Environment
See the [MAVS Environment Parameters](./Environment/MavsEnvironmentParams.md) page for a description of these inputs.

### Other Simulation inputs
``` json
 "Time Step":5e-05,
 "Max Sim Time":10.0,
 "Display Sensors":true,
 "Save Data":false,
```

The final result of these fields is shown below in *example_input.json*.
``` json
{ 
 "Scene":{
		 "Input File":"/media/sf_vm_shared/mavs/data/scenes/grass_maze.json",
		 "Origin": [32.3526,90.8779,73.152],
		  "Time Zone":6
	},

 "Driver":{ 
		 "Type":"A* Planner",
		 "Input File": "/media/sf_vm_shared/mavs/data/waypoints/example_waypoints.json",
		 "Number Processors":1,
		 "Update Rate":10.0
	},
 "Vehicle":{
		"Type":"Car",
		 "Input File":"",
		 "Number Processors":1,
		 "Initial Position":[0.0,0.0,0.0],
		 "Initial Orientation":[1.0,0.0,0.0,0.0],
		 "CG Height":2.0,
		 "Update Rate":20000.0
	},

 "Time Step":5e-05,
 "Max Sim Time":10.0,
 "Display Sensors":true,
 "Save Data":false,
 "Sensors": [
		{"Type": "camera", 
		 "Model":"XCD-V60", 
		 "Offset": [0.0, 0.0, 0.0],
		 "Orientation":[1.0, 0.0, 0.0, 0.0],
		 "Repitition Rate (Hz)":10.0, 
		 "Name":"camera", 
		 "Number Processors":1
		},

		{"Type": "gps", 
		 "Input File":"gps", 
		 "Offset": [0.0, 0.0, 0.0],
		 "Orientation":[1.0, 0.0, 0.0, 0.0],
		 "Repitition Rate (Hz)":10.0, 
		 "Name":"gps", 
		 "Number Processors":1
		},

		{"Type": "compass", 
		 "Input File":"compass", 
		 "Offset": [0.0, 0.0, 0.0],
		 "Orientation":[1.0, 0.0, 0.0, 0.0],
		 "Repitition Rate (Hz)":10.0, 
		 "Name":"compass", 
		 "Number Processors":1
		},

		{"Type": "lidar", 
		 "Model":"HDL-32E", 
		 "Offset": [0.0, 0.0, 0.0],
		 "Orientation":[1.0, 0.0, 0.0, 0.0],
		 "Repitition Rate (Hz)":10.0, 
		 "Name":"lidar", 
		 "Number Processors":1
		}
	], 
 "Environment": { 
 		 "Month": 3,
 		 "Day": 22,
 		 "Year": 2004,
 		 "Hour": 14,
 		 "Minute": 0,
 		 "Second": 0,
 		 "Turbidity": 3.0,
 		 "Local Albedo":[0.25,0.25,0.25]
	 }
 }
```


