// Object and zone selection state, group expansion, cloning, and bounds.
import { state } from '../core/state.js';

let vehicleGroupObjects;
let clearPathSelection;
let syncInspector;
let objectBounds;
let normalizeVehicleParams;
let normalizeControllerMode;
let cloneObjectData;

export function initSelection(ctx) {
  ({
    vehicleGroupObjects,
    clearPathSelection,
    syncInspector,
    objectBounds,
    normalizeVehicleParams,
    normalizeControllerMode,
    cloneObjectData,
  } = ctx);
}

export function selectedObject() {
  return selectedObjects()[0] || null;
}

export function selectedObjects() {
  return state.objects.filter((object) => state.selectedIds.has(object.id));
}

export function hasSelection() {
  return state.selectedIds.size > 0;
}

export function selectObject(id, additive = false) {
  state.selectedZoneId = null;
  clearPathSelection();
  if (!additive) {
    state.selectedIds = new Set();
  }
  if (id !== null && id !== undefined) {
    const target = state.objects.find((object) => object.id === id) || null;
    if (target?.vehicleRole === "camera_ghost" || target?.vehicleRole === "sensor_ghost") {
      if (additive && state.selectedIds.has(id)) {
        state.selectedIds.delete(id);
      } else {
        state.selectedIds.add(id);
      }
    } else {
      const groupIds = vehicleGroupObjects(target).map((object) => object.id);
      if (additive && groupIds.every((groupId) => state.selectedIds.has(groupId))) {
        for (const groupId of groupIds) state.selectedIds.delete(groupId);
      } else {
        for (const groupId of groupIds) state.selectedIds.add(groupId);
      }
    }
  }
  syncInspector();
}

export function selectObjects(ids) {
  state.selectedZoneId = null;
  state.selectedIds = new Set(ids);
  syncInspector();
}

export function selectZone(id) {
  state.selectedIds = new Set();
  state.selectedZoneId = id;
  clearPathSelection();
  syncInspector();
}

export function cloneSelectionData() {
  return selectedObjects().map(cloneObjectData);
}

export function selectionTransformState() {
  return selectedObjects().map((object) => ({
    id: object.id,
    position: [...object.position],
    rotation: [...object.rotation],
    scale: [...object.scale],
  }));
}

export function selectionGroupId() {
  const selection = selectedObjects();
  if (!selection.length) return null;
  const groupId = selection[0].vehicleGroupId;
  if (groupId == null) return null;
  if (!selection.every((object) => object.vehicleGroupId === groupId)) return null;
  const groupSize = state.objects.filter((object) => object.vehicleGroupId === groupId).length;
  return groupSize === selection.length ? groupId : null;
}

export function selectionBounds() {
  const selection = selectedObjects();
  if (!selection.length) {
    return null;
  }
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  let found = false;
  for (const object of selection) {
    const bounds = objectBounds(object);
    if (!bounds) {
      continue;
    }
    found = true;
    min[0] = Math.min(min[0], bounds.min[0]);
    min[1] = Math.min(min[1], bounds.min[1]);
    min[2] = Math.min(min[2], bounds.min[2]);
    max[0] = Math.max(max[0], bounds.max[0]);
    max[1] = Math.max(max[1], bounds.max[1]);
    max[2] = Math.max(max[2], bounds.max[2]);
  }
  return found ? { min, max } : null;
}

export function selectionCenter() {
  const bounds = selectionBounds();
  if (bounds) {
    return mul(add(bounds.min, bounds.max), 0.5);
  }
  const selection = selectedObjects();
  if (!selection.length) {
    return [0, 0, 0];
  }
  return mul(selection.reduce((sum, object) => add(sum, object.position), [0, 0, 0]), 1 / selection.length);
}
