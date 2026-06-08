// Local draft persistence and restoration orchestration.
import { state } from '../core/state.js';
import { sceneNameInput } from '../core/dom.js';
import { planeRenderObject } from '../rendering/renderer.js';
import { normalizeControllerMode, normalizeVehicleParams } from '../vehicles/vehicles.js';
import {
  hydrateObjectsFromDraft, hydratePathsFromDraft,
  restoreCameraFromDraft, restorePlaneFromDraft,
} from './scene-hydration.js';

const draftStorageKey = "mavs-scene-builder-draft-v1";

let loadVehicleDefs;
let annotateVehicleGroups;
let syncAllSensorGhosts;
let setPlaneScale;
let renderPathList;
let syncInspector;
let setStatus;

export function initDraftStorage(ctx) {
  ({
    loadVehicleDefs,
    annotateVehicleGroups,
    syncAllSensorGhosts,
    setPlaneScale,
    renderPathList,
    syncInspector,
    setStatus,
  } = ctx);
}

export function saveDraft() {
  if (!state.draftReady) return;
  try {
    localStorage.setItem(draftStorageKey, JSON.stringify({
      sceneName: sceneNameInput.value,
      objects: state.objects.filter(object => object.vehicleRole !== "sensor_ghost").map((object) => ({
        id: object.id,
        mesh: object.mesh,
        position: object.position,
        rotation: object.rotation,
        scale: object.scale,
        pathId: object.pathId ?? null,
        vehicleGroupId: object.vehicleGroupId ?? null,
        vehicleDefName: object.vehicleDefName ?? null,
        vehicleRole: object.vehicleRole ?? null,
        groupName: object.groupName ?? null,
        vehicleParams: object.vehicleParams ? normalizeVehicleParams(object.vehicleParams) : null,
        controllerMode: normalizeControllerMode(object.controllerMode, object.pathId),
        sensors: Array.isArray(object.sensors)
          ? object.sensors.map(sensor => ({ ...sensor, offset: [...sensor.offset] }))
          : null,
      })),
      randomZones: state.randomZones,
      selectedIds: [...state.selectedIds],
      selectedZoneId: state.selectedZoneId,
      paths: state.paths.map((path) => ({
        id: path.id,
        name: path.name,
        color: path.color,
        waypoints: path.waypoints,
        visible: path.visible,
      })),
      selectedPathId: state.selectedPathId,
      planeScale: [...planeRenderObject.scale],
      camera: {
        target: [...state.camera.target],
        distance: state.camera.distance,
        yaw: state.camera.yaw,
        pitch: state.camera.pitch,
      },
    }));
  } catch (error) {
    console.warn("Could not save scene builder draft", error);
  }
}

export async function restoreDraft() {
  let draft;
  try {
    draft = JSON.parse(localStorage.getItem(draftStorageKey));
  } catch (error) {
    console.warn("Could not read scene builder draft", error);
    return;
  }
  if (!draft || !Array.isArray(draft.objects)) return;

  await hydrateObjectsFromDraft(draft.objects);
  if (!state.vehicleDefs.length) {
    await loadVehicleDefs().catch(() => {});
  }
  annotateVehicleGroups();
  syncAllSensorGhosts();
  for (const object of state.objects) {
    if (!object.vehicleDefName) object.pathId = null;
  }

  state.randomZones = Array.isArray(draft.randomZones) ? draft.randomZones : [];
  state.nextZoneId = Math.max(0, ...state.randomZones.map((zone) => Number(zone.id) || 0)) + 1;
  state.selectedIds = new Set((draft.selectedIds || [])
    .filter((id) => state.objects.some((object) => object.id === id)));
  state.selectedZoneId = state.randomZones.some((zone) => zone.id === draft.selectedZoneId)
    ? draft.selectedZoneId
    : null;
  hydratePathsFromDraft(draft.paths, draft.selectedPathId);
  sceneNameInput.value = draft.sceneName || "";
  restorePlaneFromDraft(draft.planeScale, setPlaneScale);
  restoreCameraFromDraft(draft.camera);

  renderPathList();
  syncInspector();
  setStatus(`Restored draft (${state.objects.length} objects, ${state.randomZones.length} random zones, ${state.paths.length} paths)`);
}
