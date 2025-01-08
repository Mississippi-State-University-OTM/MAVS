# MAVS Camera Model
The MAVS camera model is defined by a few simple parameters in the json file. These parameters may also be accessed through the *Camera* tab of the *Sensor Editor* in the [MAVS GUI](../Gui/RunningMavsGUI.md).
``` json
"Pixels": [604, 484],
"FocalPlaneDimensions": [0.003215, 0.002576],
"FocalLength": 0.0035,
"Gamma": 0.65,
"Gain": 1.0,
"Samples Per Pixel": 1
```
- "Pixels": The number of pixels in the image plane in the horizontal and vertical directions.
- "FocalPlaneDimensions": The horizontal and vertical size of the imaging plane, in meters.
- "FocalLength": The focal length of the idealized camera system, in meters.
- "Gamma": The gamma range compression factor for the image. Pixel intensities are raised to the power of gamma. Should typically range from 0.5 (scaled by sqrt) to 1.0 (no scaling).
- "Gain": The relative gain of the camera. Raising from 1.0 will make the image brighter, lowering will make it dimmer.

Additional optional entries discussed below include
- "Distortion" - use a distorted camera model
- "Fisheye Projection" - make the camera  fisheye lens
- Anti Aliasing" - use anti-aliasing 

 Note that the distortion and fisheye entries cannot be used in the same sensor.

## Anti Aliasing
Anti-aliasing can remove jittery features around objects and boundaries. If this optional entry is not included, no anti-aliasing will be performed. Note that all anti-aliasing options will slow down the simulation to some degree.
``` json
"Anti Aliasing":{
  "Type" : "corners",
  "Samples Per Pixel": 5
}
```
- "Type": The type of anti aliasing. The options are
  * "none" - Does not perform anti-aliasing. This is the fastest option.
  * "adaptive" - Adaptively oversamples the pixels by the specified factor in areas where edges are detected. Gives better results than the "corners" method when the aliasing is caused by geometry variations, such as in densely vegetated scenes.
  * "oversampled" - Uniformly oversamples each pixel by the selected factor. This is the slowest method but also gives the best results, especially for rendering shadows of complex geometries.
- "Samples Per Pixel": The number of rays to tracer per pixel. Typical values range from 2-10, with 3-5 usually sufficient for quality results. Increasing this value will reduce aliasing and make prettier images, but will also slow the simulation down. 

## Distortion
Distortion parameters can be included by adding a "Distortion" field to the json file, as shown below. The distortion model is based on the model by Jean Bouget used in the [MATLAB Camera Calibration Toolbox](http://www.vision.caltech.edu/bouguetj/calib_doc/). The distortion parameters for a camera can be measured with the toolbox. Users should visit the [website](http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html) for more information about the definitions and conventions of these parameters. 
``` json
"Distortion": {
  "FC": [657.30,657.74],
  "CC": [302.72,242.33],
  "KC": [-0.25349,0.11868,-0.00028,0.00005,0.0],
  "alpha":0.00042
}
```
- "FC": The focal length of the camera system, in pixels
- "CC": The center of the image plane, in pixels. 
- "KC": The radial and tangential distortion coefficients
- "alpha": The skew coefficient

If the *Distortion* entry is present, the *Fisheye Projection* parameter described below should not be included.

## Fisheye Camera
A camera can be made into a fisheye camera by including the "Fisheye Projection" parameter in the camera file.
``` json
"Fisheye Projection": "equidistant"
```
The option for fisheye projection models are *equidistant*, *equisolid*, and *stereographic*. An explanation of these projection models can be found on [Wikipedia](https://en.wikipedia.org/wiki/Fisheye_lens#Mapping_function). Note that typically the focal plane dimensions should be larger than the focal length for a fisheye camera.

If the *Fisheye Projection* parameter is used, the *Distortion* entry should be omitted.