# MAVS Radar Model
The MAVS radar model is defined by a few simple parameters in the json file. These parameters may also be accessed through the *Radar* tab of the *Sensor Editor* in the [MAVS GUI](../Gui/RunningMavsGUI.md). Note that the MAVS radar model is tailored specifically to automoutive radar typically used in robotics, for example the [Delphi ESR](https://www.autonomoustuff.com/wp-content/uploads/2016/08/delphi-esr.pdf). It is not intended to be a general purpose radar simulation.

The RADAR model works by defining a lobe shape. The shape of the lobe is defined by the horizontal field of view of the RADAR, the divergence of the RADAR lobe, and the maximum range of the RADAR. The lobe is sampled spatially at an angular resolution specified in the input file to create a list of targets. 

Targets are clusters of returns that the RADAR identifies as belonging to a single object based on the relative angle and distance to the RADAR sensor.

``` json
"Max Range": 60.0,
"Field of View": 90.0,
"Lobe Size": 3.0,
"Lobe Sample Resolution": 0.25
```
- "Max Range": Maximum range of the radar in meters.
- "Field of View": Horizontal field-of-view of the radar in degrees.
- "Lobe Size": The vertical divergence of the RADAR lobe, in degrees.
- "Lobe Sample Resolution": Angular resolution at which to sample the lobe, in degrees. The defualt is 1/4 degree.
