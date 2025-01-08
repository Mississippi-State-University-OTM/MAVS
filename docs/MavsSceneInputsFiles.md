# MAVS Scene Input Files
MAVS primarily uses [json input files.](https://www.json.org/) These files describe everything from the scene geometry to the simulation time step. This page documents the scene input files and their contents.

Scene files for outdoor scenes can be automatically generated with the [MAVS Ecosystem Simulator](./Environment/MavsEcosystemSimulation.md).

## Scene Input File
A MAVS scene input file specifies the meshes that comprise the scene, as well as their position, scale and orientation. The following section discuss the input file fields and their values. Note that the [MAVS GUI](./Gui/RunningMavsGui) can be used to create and edit scenes without directly modifying the JSON files by hand.

MAVS meshes are input as [Wavefront .obj files](https://en.wikipedia.org/wiki/Wavefront_.obj_file). These files have an associate material file (.mtl) which may also contain associated texture files. Depending on the libraries that are available when MAVS is built, supported texture types may include .bmp, .jpg, .png, .tiff, and other formats. However, .bmp and .jpg are the recommended types as they are typically supported by all compilers. 

It may be helpful to review the [MAVS Coordinate System](./Environment/MavsCoordinateSystem.md) in addition to this section.

### Path to mesh files
This optional parameter specifies the full path to the folder containing the mesh files (.obj), material files (.mtl), and associated texture files (.bmp or .jpg). All these files must be in the same directory. If this parameter is not specified, then the path is assumed to be relative to the *DEFAULT_DATA_DIR* specified during the [MAVS build](./MavsBuildInstructions.md). 
``` json
"PathToMeshes": "C:/Users/cgoodin/Desktop/goodin_docs/repos/mavs_data/scenes/meshes/"
```

### Object Labels
This optional parameter tells MAVS to load a file that contains semantic labels for all objects in the scene. By default, this file should be located in the same directory as the mesh (.obj) files. The format of this file is discussed on the [MAVS Annotations page](./Sensors/MavsSensorAnnotations.md).
``` json
"Object Labels": "labels.json"
```

### Layered Surface
A layered surface is an optional entry that uses pre-defined texture layers, a surface mesh, and a path through that math to automatically create a textured surface for rendering by MAVS at runtime. Enabling this feature conserves memory (as opposed to creating a highly detailed texture file covering the entire surface), but also slows down the sensor simulation by 5-10%. 
``` json
  "Layered Surface": {
    "Mesh": "full/path/to/surface/surface.obj",
    "Trail": {
      "Trail Width": 2.0,
      "Track Width": 0.6000000238418579,
      "Wheelbase": 1.25,
      "Path": "full/path/to/replay/path.vprp"
    },
    "Layers": "full/path/to/layers.json"
  }
```
- Mesh: The full path to the base mesh file to be used for the surface
- Trail: Properties of the trail to be created
  - Trail Width: The total width of the trail in meters
  - Track Width: The width of the trail tire tracks, in meters
  - Wheelbase: The distance between the centerline of the tire tracks, in meters
  - Path: Full file path to the ANVEL replay file defining the trail
- Layers: The layer input file used to create the texture layers

### Surface Mesh
The surface mesh is an optional parameter that is listed separately from the rest of the mesh definitions. The surface mesh is not used in the sensor simulation but rather in other functions that need to quickly find the surface height - for example vehicle-terrain interaction. The surface mesh is also used by MAVS when randomly placing vegetation and other objects. Multiple surface meshes may be listed in the array. If surface meshes overlap, the surface is taken as the highest of the overlapping meshes.
Note that the full path to the surface mesh must be listed as it is not included in the search path specified by "PathToMeshes" entry.
``` json
"Surface Mesh":[
  { "Mesh": "/full/path/to/surface/surface.obj",
    "Rotate Y to Z" : false,
    "YawPitchRoll":[0.0,0.0,0.0],
    "Position":[0.0,0.0,0.0],
    "Scale":[10000.0,10000.0,0.01],
	"Material": "sand",
	"Cone Index": 150.0
  }
]
```
- *Mesh*: The surface mesh file
- *Rotate Y to Z*: Many software applications such as Maya create meshes in which the y-axis faces up. If this is the case, this value must be set to *true*.
- *YawPitchRoll*: Rotations about the Z, Y, and X axes of the mesh, respectively, in degrees. 
- *Position*: The (x,y,z) position of the mesh in local East-North-Up coordinates. 
- *Scale*: The scale in the (x,y,z) directions of the mesh, respectively.
- *Material*: Optional parameter that defines the material of the surface for the vehicle simulation. Options are "dry", "wet", "ice", "snow", "clay", and "sand". If left blank, the default is "dry", which is dry pavement.
- *Cone Index*: Optional parameter that defines the cone index of the surface, in PSI. Only relevant if the surface type has been set to "clay" or "sand". If left blank, the default is 250 PSI.

### Creating a Surface with a Heightmap File
A grayscale heightmap can also be used to create surface mesh in MAVS. In this case, the "Surface Mesh" entry is modified:
```json
  "Layered Surface": {
    "Heightmap": {
      "Map": "surfaces/mountain_heights.bmp",
      "Resolution": 10.0,
      "Scale": 10.0
    },
    "Trail": {
      "Trail Width": 4.5,
      "Track Width": 0.45,
      "Wheelbase": 2.0,
      "Path": [
        "waypoints/brownfield_main_trail.vprp"
      ]
    },
    "Layers": "surface_textures/meadow_surfaces.json"
  }
```
The *Mesh* field is now replaced by the *Heightmap* field. The .bmp heightmap file must be specified. Additionally, the horizontal (pixel) resolution of the map in meters must be specified, as well as the z-scale of the step size in the vertical dimension.

Note that when using a grayscale map, a mesh is automatically generated in the working folder. That mesh is automatically loaded in place of the "Surface Mesh" entry for the physics simulation. 

### Animations
An animation entry is a description of a mesh that will be animated by MAVS. Because animated meshes are handled differently by the Embree ray-tracing kernel, animations must be specified separately from other objects in the scene. The following shows an animation entry.
``` json
"Animations": [
  {"Animation Folder": "animations/GenericWalk",
   "Frame List": "animations/GenericWalk/walk_frames.txt",
   "Behavior": "wander",
   "Mesh Scale": 0.01,
   "Rotate Y to X": true,
   "Rotate Y to Z": false,
   "Initial Position": [ -5.0, 0.0, 0.0 ],
   "Initial Heading": 0.0,
   "Frame Rate": 30.0,
   "Speed": 1.5
  }
]
```
- *Animation Folder*: Location of the animation meshes, relative to the scenes/meshes folder
- *Frame List*: A text file containing a list of the keyframes to be used in the animation. Each keyframe is a mesh name that must be located in the folder specified in the *Animation Folder* entry.
- *Behavior*: Specifies how the mesh will behave. Options are "wander", "circle", or "straight"
- *Path*: Alternately to behavior, a path may be specifed. Path should be a waypoints json file in ENU coordinates. 
- *Mesh Scale*: Scale the input keyframe meshes if they are too big or small
- *Rotate Y to X*, *Rotate Y to Z*: If the mesh is not in the [correct MAVS default orientation](./Environment/MavsCoordinateSystem.md), set these to true.
- *Initial Position*: The initial position of the animation in ENU coordinates
- *Initial Heading*: The initial heading of the mesh, in radians relative to the positive x-axis
- *Frame Rate*: The rate of the keyframes, in Hz
- *Speed*: The speed at which the animation will move through the scene, in m/s

### Objects
This required entry is a list of all the static objects in the scene. Objects can be placed in the scene by defining a mesh and either listing one or more instances of that mesh at specified locations, or by defining a polygon inside which instances of that object will be randomply placed. Note that if a  *Surface Mesh* entry is not specifed, the first entry in the *Objects* list is assumed to be the surface when randomly placing objects. The example below shows a complete list of objects for a simple courtyard scene with trees and grass. 
``` json
"Objects":
  [
    { "Mesh": "cube.obj",
      "Instances":[
	    { "YawPitchRoll":[0.0,0.0,0.0],
	      "Position":[0.0,0.0,0.0],
	      "Scale":[625.0,625.0,0.1]}]},
    { "Mesh": "wall.obj",
      "Instances":[
	  { "YawPitchRoll":[0,0,0],
	    "Position": [310,0,0],
	    "Scale": [1.0,620.0,50.0]},
	  { "YawPitchRoll":[0,0,0],
	    "Position": [0,310,0],
	    "Scale": [620.0,1.0,50.0]},
	  {  "YawPitchRoll":[0,0,0],
	     "Position": [-310,0,0],
	     "Scale": [1.0,620.0,50.0]},
	  { "YawPitchRoll":[0,0,0],
	    "Position": [0,-310,0],
	    "Scale": [620.0,1.0,50.0]}]},
      { "Mesh": "Grass_02.obj",
	    "Rotate Y to Z": true,
	    "Random": {
		  "Offset": [0.0,0.0,1.0],
		  "Number": 5000,
		  "Polygon": [ [-300,-300],[-300,300],[300,300],[300,-300]],
		  "Scale": [0.25,0.5]
	    }
      },
      { "Mesh": "Tree_V9_Final.obj",
	    "Rotate Y to Z": true,
	    "Random":{
	   	  "Offset": [0.0,0.0,9.3],
		  "Number": 40,
		  "Polygon": [ [-300,-300],[-300,300],[300,300],[300,-300]],
		  "Scale": [0.5,3.5]
        }
      }
    ]    
```
#### Instanced objects
When specifying features such as buildings, walls, or other obstacles, it is often necessary to specify the location, scale, and orientation directly. This is done in a MAVS scene file by listing object *Instances*.
``` json
"Mesh": "wall.obj",
"Instances":[
  { 
    "YawPitchRoll":[0,0,0],
    "Position": [310,0,0],
    "Scale": [1.0,620.0,50.0]
  },
  { 
    "YawPitchRoll":[0,0,0],
    "Position": [0,310,0],
    "Scale": [620.0,1.0,50.0]
  }
]
```
- *Mesh*: The surface mesh file
- *Smooth Normals*: Use normal interpolation on this mesh (true/false). Default is false
- *Rotate Y to Z*: Many software applications such as Maya create meshes in which the y-axis faces up. If this is the case, this value must be set to *true*. If this field is not listed, it is assumed to be false.
- *YawPitchRoll*: Rotations about the Z, Y, and X axes of the mesh, respectively, in degrees. 
- *Position*: The (x,y,z) position of the mesh in local East-North-Up coordinates. 
- *Scale*: The scale in the (x,y,z) directions of the mesh, respectively.

#### Randomly placed objects
In scenes with a large number of plant models such as grass and trees, it is often easier to randomly place the vegetation than to place each model one-by-one. This is done in the MAVS scene file by the *Random* field.
``` json
"Mesh": "Tree_V9_Final.obj",
"Rotate Y to Z": true,
"Random":{
  "Offset": [0.0,0.0,9.3],
  "Number": 40,
  "Polygon": [ [-300,-300],[-300,300],[300,300],[300,-300]],
  "Scale": [0.5,3.5]
}
```
- *Mesh*: The surface mesh file
- *Rotate Y to Z*: Many software applications such as Maya create meshes in which the y-axis faces up. If this is the case, this value must be set to *true*. If this field is not listed, it is assumed to be false.
- *Offset*: The mesh needs to be centered in order to randomly place it. If this is not the case, the offset from center must be specified in local East-North-Up coordinates. 
- *Number*: The number of mesh instances to be randomly placed inside the specified polygon.
- *Polygon": A list of 2-D coordinates in the x-y plane that specify a convex polygon in which the meshes are to be placed.
- *Scale*: The scale range for the randomly placed meshes. For random meshes, the x-y-z dimensions are scaled by the same number. The scale value will be randomly chosen for each instance in the specified range.