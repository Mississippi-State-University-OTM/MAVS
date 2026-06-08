import { state } from '../core/state.js';
import {
  deleteObjectBtn, ungroupObjectsBtn, groupObjectsBtn, newGroupNameInput, snapToGroundBtn,
  newSceneBtn, loadSceneBtn, saveSceneBtn, exportSimulationBtn, previewSceneBtn, runSimulationBtn,
  planeSizeInput, addRandomZoneButton, assetSearch, addPathBtn,
  vehicleControllerMode, followPath,
  pathAddVehicle, pathAddVehicleBtn,
  togglePathPlacementBtn, clearLastWaypointBtn, saveWaypointsBtn, deletePathBtn,
  pathNameInput,
} from '../core/dom.js';
import { saveWaypoints, exportSimulation, runSimulation, saveScene, previewScene, loadSceneFromPicker } from '../scene/scene-io.js';
import { setPlaneScale } from '../rendering/renderer.js';
import { hasSelection, selectObject, selectedObjects } from '../scene/selection.js';
import { ungroupSelection, groupSelection, setVehicleControllerMode, setObjectPath } from '../vehicles/vehicles.js';
import { objectById } from '../scene/objects.js';
import { setZonePlacementMode } from '../zones/zones.js';
import { selectedPath, clearPathSelection, setPathPlacementMode, addPath } from '../paths/paths.js';
import { syncPathInspector, renderPathList } from '../paths/path-ui.js';
import { renderAssets } from './asset-ui.js';
import { syncInspector } from './inspector.js';
import { snapSelectedToGround } from '../interaction/ground-snap.js';
import { listen } from '../core/events.js';

let _setStatus = () => {};

export function initEditorActions({ setStatus }) {
  _setStatus = setStatus;

  listen(deleteObjectBtn, "click", () => {
    if (!hasSelection()) return;
    state.objects = state.objects.filter((item) => !state.selectedIds.has(item.id));
    selectObject(null);
  });

  listen(ungroupObjectsBtn, "click", ungroupSelection);

  listen(groupObjectsBtn, "click", () => {
    const name = newGroupNameInput.value.trim() || "Group";
    newGroupNameInput.value = "";
    groupSelection(name);
  });

  listen(snapToGroundBtn, "click", snapSelectedToGround);

  listen(newSceneBtn, "click", () => {
    setZonePlacementMode(false);
    clearPathSelection();
    state.objects = [];
    state.randomZones = [];
    state.paths = [];
    state.selectedIds = new Set();
    state.selectedZoneId = null;
    state.nextId = 1;
    state.nextZoneId = 1;
    state.nextPathId = 1;
    planeSizeInput.value = "100";
    setPlaneScale(1);
    renderPathList();
    syncInspector();
    _setStatus("Started a blank scene");
  });

  listen(loadSceneBtn, "click", loadSceneFromPicker);
  listen(saveSceneBtn, "click", saveScene);
  listen(exportSimulationBtn, "click", exportSimulation);
  listen(previewSceneBtn, "click", previewScene);
  listen(runSimulationBtn, "click", runSimulation);

  listen(addRandomZoneButton, "click", () => setZonePlacementMode(!state.ui.zonePlacementMode));
  listen(assetSearch, "input", renderAssets);

  listen(planeSizeInput, "input", () => {
    const s = Math.max(0.1, (Number(planeSizeInput.value) || 100)) / 100;
    setPlaneScale(s);
  });

  listen(addPathBtn, "click", addPath);

  listen(togglePathPlacementBtn, "click", () => {
    if (state.selectedPathId === null) return;
    setPathPlacementMode(!state.ui.pathPlacementMode);
  });

  listen(clearLastWaypointBtn, "click", () => {
    const path = selectedPath();
    if (path && path.waypoints.length) {
      path.waypoints.pop();
      if (state.selectedWaypointIndex !== null && state.selectedWaypointIndex >= path.waypoints.length) {
        state.selectedWaypointIndex = null;
      }
      syncPathInspector();
    }
  });

  listen(saveWaypointsBtn, "click", saveWaypoints);

  listen(deletePathBtn, "click", () => {
    if (!selectedPath()) return;
    const deletedId = state.selectedPathId;
    state.paths = state.paths.filter((p) => p.id !== deletedId);
    for (const obj of state.objects) {
      if (obj.pathId === deletedId) obj.pathId = null;
    }
    clearPathSelection();
    renderPathList();
    syncInspector();
    _setStatus("Path deleted");
  });

  listen(vehicleControllerMode, "change", () => {
    const vehicle = selectedObjects().find((object) => object.vehicleDefName);
    if (!vehicle) return;
    setVehicleControllerMode(vehicle, vehicleControllerMode.value);
    syncInspector();
  });

  listen(followPath, "change", () => {
    const vehicle = selectedObjects().find((object) => object.vehicleDefName);
    if (!vehicle) return;
    const value = followPath.value;
    setObjectPath(vehicle, value ? Number(value) : null);
    syncInspector();
  });

  listen(pathAddVehicleBtn, "click", () => {
    const path = selectedPath();
    if (!path || !pathAddVehicle.value) return;
    const obj = objectById(Number(pathAddVehicle.value));
    if (obj) {
      setObjectPath(obj, path.id);
      syncInspector();
    }
  });

  listen(pathNameInput, "input", () => {
    const path = selectedPath();
    if (path) {
      path.name = pathNameInput.value || "path.json";
      renderPathList();
    }
  });
}
