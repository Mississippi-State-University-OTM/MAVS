# MAVS Ecosystem Simulation
The MAVS provides a utility to automatically generate outdoor scenes with different types of vegetation. This feature can be accessed [through the GUI](../Gui/GeneratingRandomDataWithGUI.md), but it requires ecosystem input files to populate the vegation.

The ecosystem input files are found in the MAVS data directory under *data/ecosystem_files* and specify the type of plants that compete with each other in a given ecosystem. The meshes and material files (.obj and .mtl) for these plants must all be located in the same directory, specified by the "PathToMeshes" entry below.

## Ecosystem input files
An example ecosystem file is shown below, with description of the parameters following.
``` json
{
"Surface Mesh": "path/to/surface.obj",
	
"Anvel Replay File": "path/to/vehicle2_pos_1.vprp",
	
"PathToMeshes": "path/to/meshes/",

"Simulation Length": 20,
	
"Trail Properties": {
	"Tire Width": 0.6,
	"Wheelbase": 2.5,
	"Ground Clearance": 0.3,
	"Trail Width": 4.5
},
	
"Species":[
	{
	"Mesh File": "Tree_V9_Final.obj",
	"Species Name": "Oak tree",
	"Num New Per Area":0.05,
	"Growth Rate": 0.05,
	"Max Height": 10.0,
	"Min Height": 3.0,
	"Mesh Height": 18.5955,
	"Height To Diameter Ratio": 0.5,
	"Max Age": 50.0, 
	"Rotate Mesh": true 
	},
	{
	"Mesh File": "GC08_1.obj",
	"Species Name": "Pine tree",
	"Num New Per Area":3.0,
	"Growth Rate": 0.25,
	"Max Height": 1.0,
	"Min Height": 0.1,
	"Mesh Height": 4.10518,
	"Height To Diameter Ratio": 0.05,
	"Max Age": 50.0, 
	"Rotate Mesh": false	
	}
]
}
```
- **"Surface Mesh"**: The is the .obj file of the surface that was used when driving in the ANVEL simulation.
- **"Anvel Replay File"**: An ANVEL vehicle replay file (.vprp) in text format. The ecosystem will be created around this path, with the path defining the trail through the ecosystem.
- **"PathToMeshes"**: Full file path to the location of the folder containg the vegetation and surface meshes
- **"Simulation Length"**: The number of years to run the simulation. 15-20 years is typically sufficient.
- **"Trail Properties"**: All units are meters.
  - **"Tire Width"**: The width of the tire tracks on the trail. May be slightly wider than the actual tire.
  - **"Wheelbase"**: Wheelbase of the vehicle defines the width between the tracks.
  - **"Ground Clearance"**: The ground clearance of the vehicle limits the height of the vegetation on the trail. This may be set higher than the actual clearance of the vehicle to give more vegetation grown up on the trail.
  - **"Trail Width"**: The total width of the trail
- **"Species"**: An array listing all the species in the ecosystem. The species will compete and grow, resulting in a realistic looking plant size and species distribution.
  - **"Mesh File"**: The plant mesh file associated with the species.
  _ **"Species Name"**: A short informal name of the species.
  - **"Num New Per Area"**: How many new plants grow per square meter per year.
  - **"Growth Rate"**: How fast does plant grow. [/year]
  - **"Max Height"**: The maximum height of the plant in meters.
  - **"Height To Diameter Ratio"**: Define the "competitive" diameter of the plant, relative to height. The plant will compete with other plants within this area, with the larger plants typically winning out.
  - **"Max Age"**: The maximum age of the plant. The plant will die after this length of time.
  - **"Rotate Mesh"**: If the species mesh file as y-axis as up, set this to true.

An example output from a MAVS ecosystem simulation is shown below.
![Mavs Ecosystem](./ecosystem.png)