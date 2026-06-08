# MAVS Scene Builder

Lightweight browser-based scene builder for placing MAVS `.obj` assets into a blank scene.

## JavaScript Architecture

The browser loads the scene builder directly from the single `app.js` script.

## Run

From the repository root:

```powershell
python utilities\scene_builder\server.py
```

Open:

```text
http://127.0.0.1:8765
```

## Controls

- Browse collapsed `.obj` asset groups by type in the left sidebar. Search opens matching groups, and each visible asset gets a small generated preview.
- Pick a saved scene from the toolbar and click `Load` to edit it.
- Click `Preview` to launch a MAVS camera preview of the current editor scene. The preview camera is placed automatically from the scene object bounds.
- Click an object to select it.
- Drag over empty viewport space to marquee-select multiple objects.
- Click `Add Random Zone`, then drag a box on the ground to define a random vegetation/object area.
- Select a random zone to edit its mesh, density or fixed count, minimum spacing, scale range, offset, and bounds.
- Use the selection panel buttons to switch the gizmo: `Move`, `Rotate`, or `Scale`.
- Press `W` for move, `E` for rotate, and `R` for scale when an object is selected.
- Hold right mouse and use `W`, `A`, `S`, and `D` to free-move the camera viewpoint. `W` moves toward what you are looking at.
- The active selection is outlined with a yellow bounding box.
- Press `Ctrl+C` to copy the selected object or group and `Ctrl+V` to paste duplicates.
- Hold `Alt` while dragging a move gizmo arrow to duplicate the selection and move the copy.
- Drag the red, green, or blue gizmo handles to transform along X, Y, or Z.
- Multi-selection rotate and scale use the selection center as the pivot.
- Left-drag a selected object body to move it freely on the ground plane.
- Use `Snap to Ground` to align the selected object's bottom to the highest mesh surface below it.
- Right-drag in the viewport to orbit the camera.
- Shift-drag or middle-drag to pan.
- Mouse wheel zooms.
- Edit position, yaw/pitch/roll, and scale in the inspector.
- Select a vehicle preset group to choose No Controller, Human Controller, or Path Following. Path Following exposes path selection, speed, generated point spacing, steering scale, maximum steer angle, and lookahead distances.
- Only vehicle presets can be assigned to waypoint paths.
- Press `Delete` or use the Delete button to remove a selected object.
- Save writes MAVS scene JSON into `data/scenes`.
- The current editor draft is stored in the browser and restored after a refresh.

New scenes always include `data/scenes/meshes/surfaces/scene_builder_plane.obj` as the blank surface plane.


## TODO
1. ~~on startup Unexpected token '<', "<!DOCTYPE "... is not valid JSON appears at top~~ (fixed: response.ok guard added before JSON parse)
3. ~~Ability to increase the size of the ground plane~~ (fixed: "Plane _ m" number input in toolbar; scale persists on save/load)
4. ~~Only click and drag selects multiple items; single click should only select one object~~ (fixed: click selection collapses to one object; marquee selection requires dragging)
6. ~~Add editable boxes for MAVS random vegetation/object zones, including grass density and object count/minimum spacing~~ (fixed: random zones export MAVS `Random` polygons; density zones calculate `Number` from area)
7. Ability to place a path (multiple, they should render as different colors), and see the path (nodes and lines) for a vehicle waypoints.
8. Spawn location for vehicle, and the ability to tell a vehicle which path its following (maybe we add some symbol like a diamond thats the same color of the path its following)
10. MAVS supports actors that move. We need the ability to place these special actors that do things like trees falling
