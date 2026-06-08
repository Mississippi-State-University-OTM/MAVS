// Main inspector orchestration for objects, groups, vehicles, sensors, paths, and zones.
import { state } from '../core/state.js';
import {
  inspectorEl, inspectorTitle, selectionPanel, emptySelection, zonePanel, pathPanel,
  inspector, groupInfoRow, createGroupRow, sensorListPanel,
  vehicleControllerPanel, vehicleControllerMode, followPathRow, followPath,
  vehicleParamsPanel, vehicleSpeed, vehiclePathSpacing, vehicleSteeringScale,
  vehicleMaxSteerAngle, vehicleMinLookAhead, vehicleMaxLookAhead,
} from '../core/dom.js';
import {
  selectedObjects, selectionGroupId, selectionCenter,
} from '../scene/selection.js';
import {
  controllerModeFor, setObjectPath, setVehicleControllerMode,
  setVehicleParams, vehicleParamsFor,
} from '../vehicles/vehicles.js';
import {
  sensorsFor, syncGhostPositionsForTransforms,
} from '../vehicles/sensors.js';
import { selectedPath } from '../paths/paths.js';
import { selectedZone } from '../zones/zones.js';
import { syncPathInspector } from '../paths/path-ui.js';
import { syncZoneInspector } from '../zones/zone-ui.js';
import { renderSensorPanel, syncVizToggleBar } from '../vehicles/sensor-ui.js';

export function syncInspector() {
  const selection = selectedObjects();
  const zone = selectedZone();
  const path = selectedPath();
  const hasAnySelection = selection.length > 0 || Boolean(zone) || Boolean(path);
  document.body.classList.toggle("inspector-collapsed", !hasAnySelection);
  inspectorEl.classList.toggle("is-hidden", !hasAnySelection);
  selectionPanel.hidden = selection.length === 0 || Boolean(path);
  zonePanel.hidden = !zone || Boolean(path);
  pathPanel.hidden = !path;
  emptySelection.hidden = hasAnySelection;
  if (path) {
    inspectorTitle.textContent = "Waypoint Path";
    syncPathInspector();
    syncVizToggleBar(null);
    return;
  }
  if (zone) {
    inspectorTitle.textContent = "Random Zone";
    syncZoneInspector(zone);
    syncVizToggleBar(null);
    return;
  }
  if (!selection.length) {
    inspectorTitle.textContent = "Selection";
    syncVizToggleBar(null);
    return;
  }

  const gid = selectionGroupId();
  const isSingleGhost = selection.length === 1 && selection[0].vehicleRole === "camera_ghost";
  const isSingleSensorGhost = selection.length === 1 && selection[0].vehicleRole === "sensor_ghost";
  inspectorTitle.textContent = isSingleSensorGhost ? "Sensor" : isSingleGhost ? "Camera" : (gid != null ? "Group" : (selection.length === 1 ? "Object" : "Objects"));
  const center = selectionCenter();
  if (gid != null) {
    const rep = (isSingleGhost || isSingleSensorGhost)
      ? state.objects.find(o => o.vehicleGroupId === gid && o.vehicleRole === "chassis")
      : state.objects.find(o => o.vehicleGroupId === gid);
    if (isSingleSensorGhost) {
      const sg = selection[0];
      const sgChassis = state.objects.find(o => o.vehicleGroupId === sg.vehicleGroupId && o.vehicleRole === "chassis");
      const sgSensor = sensorsFor(sgChassis || {}).find(s => s.id === sg.sensorId);
      inspector.asset.value = `${rep?.groupName || "Vehicle"} / ${sgSensor?.name || "Sensor"}`;
      inspector.asset.disabled = true;
      inspector.asset.placeholder = "";
    } else if (isSingleGhost) {
      inspector.asset.value = `${rep?.groupName || rep?.vehicleDefName || "Vehicle"} Camera`;
      inspector.asset.disabled = true;
      inspector.asset.placeholder = "";
    } else {
      inspector.asset.value = rep?.groupName || rep?.vehicleDefName || "Group";
      inspector.asset.disabled = false;
      inspector.asset.placeholder = "Unnamed Group";
    }
  } else {
    inspector.asset.value = selection.length === 1 ? selection[0].mesh : `${selection.length} objects selected`;
    inspector.asset.disabled = true;
    inspector.asset.placeholder = "";
  }

  let vehicle = selection.find(o => o.vehicleDefName) || null;
  if (!vehicle && isSingleSensorGhost) {
    vehicle = state.objects.find(o => o.vehicleGroupId === selection[0].vehicleGroupId && o.vehicleRole === "chassis") || null;
  }
  const selectedSensorId = isSingleSensorGhost ? selection[0].sensorId : null;
  const controllerMode = vehicle ? controllerModeFor(vehicle) : "none";
  vehicleControllerPanel.hidden = !vehicle;
  sensorListPanel.hidden = !vehicle;
  if (vehicle) renderSensorPanel(vehicle, selectedSensorId);
  vehicleControllerMode.value = controllerMode;
  followPathRow.hidden = !vehicle || controllerMode !== "path";
  if (vehicle && controllerMode === "path") {
    followPath.innerHTML = '<option value="">None</option>';
    for (const candidate of state.paths) {
      const option = document.createElement("option");
      option.value = String(candidate.id);
      option.textContent = candidate.name;
      followPath.appendChild(option);
    }
    followPath.value = vehicle.pathId != null ? String(vehicle.pathId) : "";
  }
  vehicleParamsPanel.hidden = !vehicle || controllerMode !== "path" || vehicle.pathId == null;
  if (!vehicleParamsPanel.hidden) {
    const params = vehicleParamsFor(vehicle);
    vehicleSpeed.value = params.speed;
    vehiclePathSpacing.value = params.pathSpacing;
    vehicleSteeringScale.value = params.steeringScale;
    vehicleMaxSteerAngle.value = params.maxSteerAngle;
    vehicleMinLookAhead.value = params.minLookAhead;
    vehicleMaxLookAhead.value = params.maxLookAhead;
  }
  groupInfoRow.hidden = gid == null || isSingleGhost || isSingleSensorGhost;
  createGroupRow.hidden = gid != null || selection.length < 2;
  inspector.posX.value = center[0].toFixed(2);
  inspector.posY.value = center[1].toFixed(2);
  inspector.posZ.value = center[2].toFixed(2);
  if (selection.length === 1) {
    const object = selection[0];
    inspector.rotZ.value = object.rotation[2].toFixed(1);
    inspector.rotY.value = object.rotation[1].toFixed(1);
    inspector.rotX.value = object.rotation[0].toFixed(1);
    inspector.scaleX.value = object.scale[0].toFixed(2);
    inspector.scaleY.value = object.scale[1].toFixed(2);
    inspector.scaleZ.value = object.scale[2].toFixed(2);
  } else {
    inspector.rotZ.value = "";
    inspector.rotY.value = "";
    inspector.rotX.value = "";
    inspector.scaleX.value = "";
    inspector.scaleY.value = "";
    inspector.scaleZ.value = "";
  }
  syncVizToggleBar(vehicle);
}

export function applyInspector() {
  const selection = selectedObjects();
  if (!selection.length) return;
  const vehicle = selection.find((object) => object.vehicleDefName);
  if (vehicle) {
    setVehicleControllerMode(vehicle, vehicleControllerMode.value);
  }
  if (vehicle && !vehicleParamsPanel.hidden) {
    setVehicleParams(vehicle, {
      speed: vehicleSpeed.value,
      pathSpacing: vehiclePathSpacing.value,
      steeringScale: vehicleSteeringScale.value,
      maxSteerAngle: vehicleMaxSteerAngle.value,
      minLookAhead: vehicleMinLookAhead.value,
      maxLookAhead: vehicleMaxLookAhead.value,
    });
  }
  if (selection.length > 1) {
    const nextCenter = [Number(inspector.posX.value), Number(inspector.posY.value), Number(inspector.posZ.value)];
    if (nextCenter.every((value) => Number.isFinite(value))) {
      const center = selectionCenter();
      const delta = [nextCenter[0] - center[0], nextCenter[1] - center[1], nextCenter[2] - center[2]];
      for (const object of selection) {
        object.position = [
          object.position[0] + delta[0],
          object.position[1] + delta[1],
          object.position[2] + delta[2],
        ];
      }
    }
    if (selectionGroupId() != null && followPath && !followPath.closest("[hidden]")) {
      const value = followPath.value;
      setObjectPath(selection[0], value ? Number(value) : null);
    }
    syncGhostPositionsForTransforms(selection.map(o => ({ id: o.id })));
    return;
  }
  const object = selection[0];
  object.position = [Number(inspector.posX.value), Number(inspector.posY.value), Number(inspector.posZ.value)];
  object.rotation = [Number(inspector.rotX.value), Number(inspector.rotY.value), Number(inspector.rotZ.value)];
  object.scale = [Number(inspector.scaleX.value), Number(inspector.scaleY.value), Number(inspector.scaleZ.value)]
    .map((value) => Math.max(0.01, value));
  if (followPath && !followPath.closest("[hidden]")) {
    const value = followPath.value;
    setObjectPath(object, value ? Number(value) : null);
  }
  syncGhostPositionsForTransforms(selection.map(o => ({ id: o.id })));
}
