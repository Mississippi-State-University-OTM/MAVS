// Reconstruct editor state from parsed draft data.
import { state } from '../core/state.js';
import { loadModel } from '../rendering/model-loader.js';
import { colorForPath } from './objects.js';
import { normalizeControllerMode, normalizeVehicleParams } from '../vehicles/vehicles.js';
import { pathColorPalette } from '../paths/paths.js';

export async function hydrateObjectsFromDraft(draftObjects) {
  const restoredObjects = draftObjects
    .filter((object) => object && object.mesh)
    .map((object, index) => ({
      id: Number.isInteger(object.id) ? object.id : index + 1,
      mesh: object.mesh,
      position: Array.isArray(object.position) ? [...object.position] : [0, 0, 0],
      rotation: Array.isArray(object.rotation) ? [...object.rotation] : [0, 0, 0],
      scale: Array.isArray(object.scale) ? [...object.scale] : [1, 1, 1],
      color: colorForPath(object.mesh),
      pathId: object.pathId ?? null,
      vehicleGroupId: object.vehicleGroupId ?? null,
      vehicleDefName: object.vehicleDefName ?? null,
      vehicleRole: object.vehicleRole ?? null,
      groupName: object.groupName ?? null,
      vehicleParams: object.vehicleParams ? normalizeVehicleParams(object.vehicleParams) : null,
      controllerMode: normalizeControllerMode(object.controllerMode, object.pathId),
      sensors: Array.isArray(object.sensors)
        ? object.sensors.map(sensor => ({
          ...sensor,
          offset: Array.isArray(sensor.offset) ? [...sensor.offset] : [0, 0, 0],
        }))
        : null,
    }));

  const maxSensorId = restoredObjects
    .flatMap(object => object.sensors || [])
    .reduce((max, sensor) => Math.max(max, sensor.id || 0), 0);
  state.nextSensorId = Math.max(state.nextSensorId, maxSensorId + 1);
  await Promise.all([...new Set(restoredObjects.map((object) => object.mesh))]
    .map((mesh) => loadModel(mesh).catch(() => {})));

  state.objects = restoredObjects;
  state.nextId = Math.max(0, ...state.objects.map((object) => object.id)) + 1;
  state.nextGroupId = Math.max(0, ...state.objects.map((object) => object.vehicleGroupId ?? 0)) + 1;
}

export function hydratePathsFromDraft(draftPaths, selectedPathId) {
  state.paths = [];
  if (Array.isArray(draftPaths)) {
    state.paths = draftPaths
      .filter((path) => path && Array.isArray(path.waypoints))
      .map((path) => ({
        id: Number.isInteger(path.id) ? path.id : state.nextPathId++,
        name: String(path.name || "path.json"),
        color: Array.isArray(path.color) && path.color.length === 3 ? [...path.color] : pathColorPalette[0],
        waypoints: path.waypoints.filter((waypoint) => Array.isArray(waypoint) && waypoint.length >= 2),
        visible: path.visible !== false,
      }));
  }
  state.nextPathId = Math.max(1, ...state.paths.map((path) => path.id + 1));
  state.selectedPathId = selectedPathId != null && state.paths.some((path) => path.id === selectedPathId)
    ? selectedPathId
    : null;
}

export function restoreCameraFromDraft(camera) {
  if (!camera) return;
  if (Array.isArray(camera.target)) state.camera.target = [...camera.target];
  if (Number.isFinite(camera.distance)) state.camera.distance = camera.distance;
  if (Number.isFinite(camera.yaw)) state.camera.yaw = camera.yaw;
  if (Number.isFinite(camera.pitch)) state.camera.pitch = camera.pitch;
}

export function restorePlaneFromDraft(planeScale, setPlaneScale) {
  if (Array.isArray(planeScale)) {
    setPlaneScale(Number(planeScale[0]) || 1);
  }
}
