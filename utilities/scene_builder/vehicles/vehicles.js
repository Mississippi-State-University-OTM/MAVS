// Vehicle placement, grouping, controller, path, and annotation rules.
import { state } from '../core/state.js';
import { loadModel } from '../rendering/model-loader.js';
import { colorForPath, objectBounds } from '../scene/objects.js';
import { selectedObjects, selectObjects } from '../scene/selection.js';

const defaultVehicleParams = {
  speed: 5.0,
  pathSpacing: 1.0,
  steeringScale: 3.0,
  maxSteerAngle: 0.6,
  minLookAhead: 5.0,
  maxLookAhead: 25.0,
};

let setStatus;
let syncInspector;
let syncCameraGhost;

export function initVehicles(ctx) {
  ({ setStatus, syncInspector, syncCameraGhost } = ctx);
}

export function normalizeVehicleParams(params = {}) {
  const speed = Math.max(0.1, Number(params.speed) || defaultVehicleParams.speed);
  const pathSpacing = Math.max(0.1, Number(params.pathSpacing) || defaultVehicleParams.pathSpacing);
  const steeringScale = Math.max(0.1, Number(params.steeringScale) || defaultVehicleParams.steeringScale);
  const maxSteerAngle = Math.max(0.01, Number(params.maxSteerAngle) || defaultVehicleParams.maxSteerAngle);
  const minLookAhead = Math.max(0.1, Number(params.minLookAhead) || defaultVehicleParams.minLookAhead);
  const maxLookAhead = Math.max(minLookAhead, Number(params.maxLookAhead) || defaultVehicleParams.maxLookAhead);
  return { speed, pathSpacing, steeringScale, maxSteerAngle, minLookAhead, maxLookAhead };
}

export function normalizeControllerMode(mode, pathId = null) {
  if (mode === "human" || mode === "path" || mode === "none") return mode;
  return pathId != null ? "path" : "none";
}

export async function addVehicle(def, position = [0, 0, 0]) {
  const groupId = state.nextGroupId++;
  const newIds = [];
  const vehicleParams = normalizeVehicleParams();
  const chassisOffset = def.chassis_offset;
  const firstAxle = def.axles[0] || { tire_radius: 0.35 };
  const chassisModel = await loadModel(def.chassis_mesh).catch(() => null);
  const scaledChassisMinZ = chassisModel ? chassisModel.min[2] * def.chassis_scale[2] : 0;
  const chassisHeight = firstAxle.tire_radius - chassisOffset[2] - scaledChassisMinZ;
  const chassisPosition = [
    position[0] + chassisOffset[0],
    position[1] + chassisOffset[1],
    position[2] + chassisHeight + chassisOffset[2],
  ];
  const chassis = {
    id: state.nextId++,
    mesh: def.chassis_mesh,
    position: chassisPosition,
    rotation: [...(def.chassis_rotation || [0, 0, 0])],
    scale: [...def.chassis_scale],
    color: colorForPath(def.chassis_mesh),
    pathId: null,
    vehicleGroupId: groupId,
    vehicleDefName: def.name,
    vehicleRole: "chassis",
    groupName: def.name,
    vehicleParams: { ...vehicleParams },
    controllerMode: "none",
  };
  state.objects.push(chassis);
  newIds.push(chassis.id);

  if (def.tire_mesh) {
    await loadModel(def.tire_mesh).catch(() => {});
    const tireOffset = def.tire_offset;
    for (const axle of def.axles) {
      const tireZ = position[2] + axle.tire_radius;
      for (const side of [1, -1]) {
        const tire = {
          id: state.nextId++,
          mesh: def.tire_mesh,
          position: [
            position[0] + axle.longitudinal_offset + tireOffset[0],
            position[1] + side * axle.track_width * 0.5 + tireOffset[1],
            tireZ + tireOffset[2],
          ],
          rotation: [...(def.tire_rotation || [0, 0, 0])],
          scale: [...def.tire_scale],
          color: colorForPath(def.tire_mesh),
          pathId: null,
          vehicleGroupId: groupId,
          vehicleDefName: def.name,
          vehicleRole: "tire",
          groupName: def.name,
          vehicleParams: { ...vehicleParams },
          controllerMode: "none",
        };
        state.objects.push(tire);
        newIds.push(tire.id);
      }
    }
  }

  selectObjects(newIds);
  setStatus(`Placed ${def.name} (${newIds.length} parts)`);
}

function isVehicleGroupMember(object, vehicleDefName) {
  if (object.vehicleRole === "camera_ghost" || object.vehicleRole === "sensor_ghost") {
    return true;
  }
  const def = state.vehicleDefs.find((candidate) => candidate.name === vehicleDefName);
  return Boolean(def && (object.mesh === def.chassis_mesh || object.mesh === def.tire_mesh));
}

export function vehicleGroupObjects(object) {
  if (object?.vehicleGroupId == null) {
    return object ? [object] : [];
  }
  const group = state.objects.filter((candidate) => candidate.vehicleGroupId === object.vehicleGroupId);
  if (!object.vehicleDefName && object.vehicleRole !== "camera_ghost" && object.vehicleRole !== "sensor_ghost") {
    return group;
  }
  const vehicleDefName = object.vehicleDefName
    || group.find((candidate) => candidate.vehicleDefName)?.vehicleDefName;
  return vehicleDefName
    ? group.filter((candidate) => isVehicleGroupMember(candidate, vehicleDefName))
    : group;
}

export function vehicleParamsFor(object) {
  const stored = vehicleGroupObjects(object).find((part) => part.vehicleParams)?.vehicleParams;
  return normalizeVehicleParams(stored);
}

export function setVehicleParams(object, params) {
  const normalized = normalizeVehicleParams(params);
  for (const part of vehicleGroupObjects(object)) {
    part.vehicleParams = { ...normalized };
  }
}

export function controllerModeFor(object) {
  const stored = vehicleGroupObjects(object).find((part) => part.controllerMode)?.controllerMode;
  return normalizeControllerMode(stored, object?.pathId);
}

export function setVehicleControllerMode(object, mode) {
  const normalized = normalizeControllerMode(mode);
  for (const part of vehicleGroupObjects(object)) {
    part.controllerMode = normalized;
    if (normalized !== "path") part.pathId = null;
  }
  syncCameraGhost(object);
}

export function setObjectPath(object, pathId) {
  if (!object?.vehicleDefName) return;
  for (const part of vehicleGroupObjects(object)) {
    part.pathId = pathId;
    if (pathId != null) part.controllerMode = "path";
  }
}

export function vehicleRepresentatives() {
  const seenGroups = new Set();
  return state.objects.filter((object) => {
    if (!object.vehicleDefName) return false;
    if (object.vehicleGroupId == null) return true;
    if (seenGroups.has(object.vehicleGroupId)) return false;
    seenGroups.add(object.vehicleGroupId);
    return object.vehicleRole === "chassis"
      || !state.objects.some((candidate) =>
        candidate.vehicleGroupId === object.vehicleGroupId && candidate.vehicleRole === "chassis");
  });
}

export function ungroupSelection() {
  const selection = selectedObjects();
  if (!selection.length) return;
  const groupId = selection[0].vehicleGroupId;
  if (groupId == null || !selection.every((object) => object.vehicleGroupId === groupId)) return;
  const group = state.objects.filter((object) => object.vehicleGroupId === groupId);
  if (group.length !== selection.length) return;
  for (const object of group) {
    object.vehicleGroupId = null;
    object.vehicleRole = null;
    object.vehicleDefName = null;
    object.controllerMode = "none";
    object.pathId = null;
  }
  syncInspector();
}

export function groupSelection(name) {
  const selection = selectedObjects();
  if (selection.length < 2) return;
  const groupId = state.nextGroupId++;
  for (const object of selection) {
    object.vehicleGroupId = groupId;
    object.groupName = name || "Group";
    object.vehicleRole = null;
    object.vehicleDefName = null;
    object.controllerMode = "none";
    object.pathId = null;
  }
  syncInspector();
}

export function annotateVehicleGroups() {
  const groupIds = [...new Set(state.objects.map((object) => object.vehicleGroupId).filter((id) => id != null))];
  for (const groupId of groupIds) {
    const groupedObjects = state.objects.filter((object) => object.vehicleGroupId === groupId);
    const def = state.vehicleDefs.find((candidate) =>
      groupedObjects.some((object) => object.mesh === candidate.chassis_mesh));
    if (!def) continue;
    const vehicleMeshes = new Set([def.chassis_mesh, def.tire_mesh].filter(Boolean));
    const group = groupedObjects.filter((object) =>
      object.vehicleRole === "camera_ghost"
      || object.vehicleRole === "sensor_ghost"
      || vehicleMeshes.has(object.mesh));
    for (const object of groupedObjects) {
      if (group.includes(object)) continue;
      object.vehicleGroupId = null;
      object.vehicleDefName = null;
      object.vehicleRole = null;
      object.groupName = null;
      object.vehicleParams = null;
      object.controllerMode = "none";
      object.pathId = null;
    }
    const pathId = group.find((object) => object.pathId != null)?.pathId ?? null;
    const vehicleParams = normalizeVehicleParams(group.find((object) => object.vehicleParams)?.vehicleParams);
    const controllerMode = normalizeControllerMode(
      group.find((object) => object.controllerMode)?.controllerMode,
      pathId,
    );
    for (const object of group) {
      object.vehicleDefName = def.name;
      if (object.vehicleRole !== "camera_ghost" && object.vehicleRole !== "sensor_ghost") {
        object.vehicleRole = object.mesh === def.chassis_mesh ? "chassis" : "tire";
      }
      object.pathId = pathId;
      object.vehicleParams = { ...vehicleParams };
      object.controllerMode = controllerMode;
    }
    alignVehicleGroupVisual(group);
  }
}

function alignVehicleGroupVisual(group) {
  const chassis = group.find((object) => object.vehicleRole === "chassis");
  const tires = group.filter((object) => object.vehicleRole === "tire");
  const chassisBounds = chassis ? objectBounds(chassis) : null;
  if (!chassisBounds || !tires.length) return;
  const wheelCenterZ = tires.reduce((sum, tire) => sum + tire.position[2], 0) / tires.length;
  chassis.position[2] += wheelCenterZ - chassisBounds.min[2];
}
