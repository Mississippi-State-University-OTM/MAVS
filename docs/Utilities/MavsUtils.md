# MAVS  Utilities
The MAVS provides several utility applications.

* [Utilities](#Utilities)  
  * [Ecosystem](##create_ecosystem)
  * [DEM Loader](##dem_tool)
  * [Mesh Manipulator](##mesh_manipulator)
  * [Scene Viewer](##scene_viewer)
  * [Orthographic View](##top_down_ortho)
  * [.obj Viewer](./MavsObjViewerUtility)

# Utilities

## create_ecosystem
Utility for MAVS to create a random scene. The input definition includes parameters for the terrain and vegetation.

Usage: 
``` shell
>./create_ecosystem random_scene_inputs.json
```
where "random_scene_inputs.json" is random scene definition file. An example is in mavs/data/ecosystem_files/random_scene_inputs.json.

This utility will create a surface file mesh and associated scene file.

## dem_tool
Load a Digital Elevation Model (DEM) in .asc format and display it.

Usage: 
``` shell
>./dem_loader asc_file.asc
```
where "asc_file.asc" file is an ascii DEM file. An example can be found in *mavs/data/hmf_files*.

## mesh_manipulator
Code to scale, rotate, and re-center Wavefront .obj meshes.

Usage:
``` shell
>./mesh_manipulator path_to_mtl full_path_to_mesh.obj
```
"path_to_mtl" is a file path, without the file, to the directory where the associated .mtl file is located. "full_path_to_mesh.obj" is the full path to the obj file to manipulate.

The code will prompt for scale, rotation, and translation values.

## scene_viewer
View a scene and move the camera through it with the arrow keys. Identical functionality to the "free_camera_example."

Usage: 
``` shell
>./scene_viewer scene.json
```
Where "scene.json" examples can be found in *mavs/data/scenes*. 

Fly the camera around the scene with the W-A-S-D keys. Page Up & Page Down to move up and down. Arrow keys rotate the view (left,right,up,down) Home and End keys rotate the roll of the camera Close the view window to finish the program.

## top_down_ortho
Generates a top-down orthographic rendering of a scene and saves it to a .ppm image file.

Usage: 
``` shell
> top_down_ortho scene.json (display)
```
Where "scene.json" examples can be found in *mavs/data/scenes*, and "display" tells the program to display the result (display=1) or not (display=0).

The program saves two image files, "height_orthoview.ppm" and "color_orthoview.ppm".
