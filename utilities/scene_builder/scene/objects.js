// General scene-object creation, cloning, transforms, bounds, and duplication.
import { state } from '../core/state.js';
import { modelCache, loadModel } from '../rendering/model-loader.js';
import { selectedObjects, selectObject, selectObjects, cloneSelectionData } from './selection.js';

let invalidateLidarForBounds;
let normalizeVehicleParams;
let normalizeControllerMode;
let setStatus;

export function initObjects(ctx) {
  ({
    invalidateLidarForBounds,
    normalizeVehicleParams,
    normalizeControllerMode,
    setStatus,
  } = ctx);
}

export async function addObject(path, position = [0, 0, 0], groupId = null) {
  await loadModel(path);
  const object = {
    id: state.nextId++,
    mesh: path,
    position: [position[0], position[1], Math.max(0, position[2])],
    rotation: [0, 0, 0],
    scale: [1, 1, 1],
    color: colorForPath(path),
    pathId: null,
    vehicleGroupId: groupId,
  };
  state.objects.push(object);
  const bounds = objectWorldAABB(object);
  if (bounds) invalidateLidarForBounds(bounds);
  selectObject(object.id);
  return object;
}

export function objectById(id) {
  return state.objects.find((object) => object.id === id) || null;
}

export function cloneObjectData(object) {
  return {
    mesh: object.mesh,
    position: [...object.position],
    rotation: [...object.rotation],
    scale: [...object.scale],
    color: [...object.color],
    pathId: object.pathId ?? null,
    vehicleGroupId: object.vehicleGroupId ?? null,
    vehicleDefName: object.vehicleDefName ?? null,
    vehicleRole: object.vehicleRole ?? null,
    groupName: object.groupName ?? null,
    vehicleParams: object.vehicleParams ? normalizeVehicleParams(object.vehicleParams) : null,
    controllerMode: normalizeControllerMode(object.controllerMode, object.pathId),
  };
}

export function duplicateSelectionAtSamePosition() {
  const selection = selectedObjects();
  const duplicateIds = [];
  const duplicateGroupIds = new Map();
  for (const source of selection) {
    let vehicleGroupId = null;
    if (source.vehicleGroupId != null) {
      if (!duplicateGroupIds.has(source.vehicleGroupId)) {
        duplicateGroupIds.set(source.vehicleGroupId, state.nextGroupId++);
      }
      vehicleGroupId = duplicateGroupIds.get(source.vehicleGroupId);
    }
    const object = {
      id: state.nextId++,
      mesh: source.mesh,
      position: [...source.position],
      rotation: [...source.rotation],
      scale: [...source.scale],
      color: [...source.color],
      pathId: source.pathId ?? null,
      vehicleGroupId,
      vehicleDefName: source.vehicleDefName ?? null,
      vehicleRole: source.vehicleRole ?? null,
      groupName: source.groupName ?? null,
      vehicleParams: source.vehicleParams ? normalizeVehicleParams(source.vehicleParams) : null,
      controllerMode: normalizeControllerMode(source.controllerMode, source.pathId),
    };
    state.objects.push(object);
    duplicateIds.push(object.id);
  }
  selectObjects(duplicateIds);
  return duplicateIds.length;
}

export async function pasteCopiedObject() {
  if (!state.clipboard.copiedObjects.length) {
    return;
  }
  await Promise.all(state.clipboard.copiedObjects.map((object) => loadModel(object.mesh)));
  const offset = 2;
  const pastedIds = [];
  const pastedGroupIds = new Map();
  for (const copiedObject of state.clipboard.copiedObjects) {
    let vehicleGroupId = null;
    if (copiedObject.vehicleGroupId != null) {
      if (!pastedGroupIds.has(copiedObject.vehicleGroupId)) {
        pastedGroupIds.set(copiedObject.vehicleGroupId, state.nextGroupId++);
      }
      vehicleGroupId = pastedGroupIds.get(copiedObject.vehicleGroupId);
    }
    const object = {
      id: state.nextId++,
      mesh: copiedObject.mesh,
      position: [copiedObject.position[0] + offset, copiedObject.position[1] + offset, copiedObject.position[2]],
      rotation: [...copiedObject.rotation],
      scale: [...copiedObject.scale],
      color: [...copiedObject.color],
      pathId: copiedObject.pathId ?? null,
      vehicleGroupId,
      vehicleDefName: copiedObject.vehicleDefName ?? null,
      vehicleRole: copiedObject.vehicleRole ?? null,
      groupName: copiedObject.groupName ?? null,
      vehicleParams: copiedObject.vehicleParams ? normalizeVehicleParams(copiedObject.vehicleParams) : null,
      controllerMode: normalizeControllerMode(copiedObject.controllerMode, copiedObject.pathId),
      sensors: Array.isArray(copiedObject.sensors)
        ? copiedObject.sensors.map((sensor) => ({ ...sensor, id: state.nextSensorId++, offset: [...sensor.offset] }))
        : null,
    };
    state.objects.push(object);
    pastedIds.push(object.id);
    const bounds = objectWorldAABB(object);
    if (bounds) invalidateLidarForBounds(bounds);
  }
  selectObjects(pastedIds);
  state.clipboard.copiedObjects = cloneSelectionData();
  setStatus(`Pasted ${pastedIds.length} object${pastedIds.length === 1 ? "" : "s"}`);
}

export function objectBounds(object) {
  if (object.vehicleRole === "camera_ghost") {
    const r = 0.6;
    const [x, y, z] = object.position;
    return { min: [x - r, y - r, z - r], max: [x + r, y + r, z + r] };
  }
  if (object.vehicleRole === "sensor_ghost") {
    const r = 0.45;
    const [x, y, z] = object.position;
    return { min: [x - r, y - r, z - r], max: [x + r, y + r, z + r] };
  }
  const model = modelCache.get(object.mesh);
  if (!model) {
    return null;
  }
  const matrix = modelMatrix(object);
  const corners = [];
  for (const x of [model.min[0], model.max[0]]) {
    for (const y of [model.min[1], model.max[1]]) {
      for (const z of [model.min[2], model.max[2]]) {
        corners.push(transformPoint(matrix, [x, y, z]));
      }
    }
  }
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (const point of corners) {
    min[0] = Math.min(min[0], point[0]);
    min[1] = Math.min(min[1], point[1]);
    min[2] = Math.min(min[2], point[2]);
    max[0] = Math.max(max[0], point[0]);
    max[1] = Math.max(max[1], point[1]);
    max[2] = Math.max(max[2], point[2]);
  }
  return { min, max, corners };
}

export function objectWorldAABB(object) {
  const model = modelCache.get(object.mesh);
  if (!model) return null;
  const matrix = modelMatrix(object);
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (const x of [model.min[0], model.max[0]]) {
    for (const y of [model.min[1], model.max[1]]) {
      for (const z of [model.min[2], model.max[2]]) {
        const [worldX, worldY, worldZ] = transformPoint(matrix, [x, y, z]);
        if (worldX < minX) minX = worldX; if (worldX > maxX) maxX = worldX;
        if (worldY < minY) minY = worldY; if (worldY > maxY) maxY = worldY;
        if (worldZ < minZ) minZ = worldZ; if (worldZ > maxZ) maxZ = worldZ;
      }
    }
  }
  return { minX, maxX, minY, maxY, minZ, maxZ };
}

export function modelMatrix(object) {
  let matrix = mat4Translate(object.position);
  matrix = mat4Multiply(matrix, mat4RotateZ(degToRad(object.rotation[2])));
  matrix = mat4Multiply(matrix, mat4RotateY(degToRad(object.rotation[1])));
  matrix = mat4Multiply(matrix, mat4RotateX(degToRad(object.rotation[0])));
  return mat4Multiply(matrix, mat4Scale(object.scale));
}

export function colorForPath(path) {
  let hash = 0;
  for (let i = 0; i < path.length; i++) {
    hash = (hash * 31 + path.charCodeAt(i)) >>> 0;
  }
  const hue = hash % 360;
  const c = 0.55;
  const x = c * (1 - Math.abs((hue / 60) % 2 - 1));
  const m = 0.28;
  const sector = Math.floor(hue / 60);
  const rgb = [
    [c, x, 0], [x, c, 0], [0, c, x],
    [0, x, c], [x, 0, c], [c, 0, x],
  ][sector];
  return [rgb[0] + m, rgb[1] + m, rgb[2] + m];
}
