# MAVS Environment Parameters
The MAVS environment parameters describe the environmental condition for use in sensor and vehicle simulations. They can either be set in the environment block of  [MAVS Simulation](../RunningASimulation.md) or can be set in a separate json file located in the MAVS data directory under the *environments* folder.
``` json
"Environment": { 
	"Month": 3,
	"Day": 22,
	"Year": 2004,
	"Hour": 14,
	"Minute": 0,
	"Second": 0,
	"Turbidity": 3.0,
	"Local Albedo":[0.25,0.25,0.25],
	"Rain Rate": 25.0,
	"Wind": [5.0,2.0]
}
```
* *Month*: Month of the simulation from 1-12
* *Day*: Day of the month, from 1 to 31
* *Year*: Four digit year of the simulation 
* *Hour*: Hour, from 0-23
* *Minute*: From 0-59, 
* *Second*: From 0-59
* *Turbidity*: The atmospheric turbidity. 2.0 is very clear, while 10.0 is very hazy.
* *Local Albedo*: The RGB reflectance of the local terrain.
* *Rain Rate*: The rainfall rate in mm/h. 2.5 is light rain, while 50.0 is very heavy rain
* *Wind*: The horizontal wind speed and direction in m/s