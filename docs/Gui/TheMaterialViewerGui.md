# The Material Viewer GUI
MAVS provides a GUI for viewing the materials associated with a particular file, including the diffuse and spectral RGB values as well as the reflectance spectrum.

To run the Material viewer GUI
``` python
$python mavs/src/mavs_python/gui/material_viewer_gui.py
```

You will be prompted to select a mesh file with .obj extension. All the materials associated with that mesh will be loaded and listed to the GUI screen. Double clicking on any material will display its properties.