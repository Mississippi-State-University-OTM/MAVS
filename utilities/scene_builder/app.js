import { state } from './core/state.js';
import {
  initSceneIO,
  sceneJson, simulationActorsJson, simulationVehiclesJson,
  loadAssets, loadVehicleDefs, loadScenes, loadSceneData,
} from './scene/scene-io.js';
import {
  gl,
  scenePicker, sceneNameInput, planeSizeInput,
  assetPaths,
  addRandomZoneButton,
} from './core/dom.js';
import { cameraPosition, projectToScreen } from './rendering/camera.js';
import { initCanvasController, cancelZonePlacement } from './interaction/canvas-controller.js';
import { render, drawPoints } from './rendering/scene-renderer.js';
import { initModelLoader, modelCache, loadModel } from './rendering/model-loader.js';
import { initRenderer, planeRenderObject, setPlaneScale } from './rendering/renderer.js';
import { initSelection } from './scene/selection.js';
import {
  initObjects,
  cloneObjectData,
  objectBounds, objectWorldAABB, modelMatrix,
} from './scene/objects.js';
import {
  initVehicles,
  normalizeVehicleParams, normalizeControllerMode,
  vehicleGroupObjects, vehicleParamsFor,
  controllerModeFor,
  vehicleRepresentatives, annotateVehicleGroups,
} from './vehicles/vehicles.js';
import {
  initSensors,
  sensorsFor, syncCameraGhost, syncAllSensorGhosts,
} from './vehicles/sensors.js';
import {
  initPaths,
  selectedPath, clearPathSelection,
} from './paths/paths.js';
import {
  initPathUI, syncPathInspector, renderPathList,
} from './paths/path-ui.js';
import { initZones, setZonePlacementMode } from './zones/zones.js';
import { initZoneUI } from './zones/zone-ui.js';
import { syncInspector } from './ui/inspector.js';
import { initDraftStorage, saveDraft, restoreDraft } from './scene/draft-storage.js';
import { listen, startEditorLifecycle } from './core/events.js';
import { renderAssets, renderVehicleDefs } from './ui/asset-ui.js';
import { initGroundSnap } from './interaction/ground-snap.js';
import { initLidarPreview, invalidateLidarCache, invalidateLidarForBounds, clearSensorGhostLidarCache } from './vehicles/lidar-preview.js';
import { initSensorEvents } from './vehicles/sensor-events.js';
import { initEditorActions } from './ui/editor-actions.js';
import { setStatus } from './core/status.js';
import { initInspectorEvents } from './ui/inspector-events.js';

const planeMeshPath = "scenes/meshes/surfaces/scene_builder_plane.obj";
initModelLoader({ gl });
initRenderer({ gl, planeMeshPath, planeSizeInput });
initPathUI({ syncInspector });
initZoneUI({ syncInspector });
initZones({ clearPathSelection, syncInspector, setStatus });
initPaths({
  projectToScreen,
  deactivateZonePlacement: () => {
    cancelZonePlacement();
    state.ui.zonePlacementMode = false;
    addRandomZoneButton.classList.toggle("active", false);
  },
  syncInspector,
  syncPathInspector,
  renderPathList,
  setStatus,
});
initObjects({
  invalidateLidarForBounds,
  normalizeVehicleParams,
  normalizeControllerMode,
  setStatus,
});
initVehicles({ setStatus, syncInspector, syncCameraGhost });
initSelection({
  vehicleGroupObjects,
  clearPathSelection,
  syncInspector,
  objectBounds,
  normalizeVehicleParams,
  normalizeControllerMode,
  cloneObjectData,
});
initDraftStorage({
  loadVehicleDefs,
  annotateVehicleGroups,
  syncAllSensorGhosts,
  setPlaneScale,
  renderPathList,
  syncInspector,
  setStatus,
});
initGroundSnap({ setStatus });
initLidarPreview({ gl, planeRenderObject, modelCache, modelMatrix, objectWorldAABB, drawPoints, setStatus });
initSensors({ clearSensorGhostLidarCache });
initCanvasController({ setStatus });
initSensorEvents({ setStatus });
initEditorActions({ setStatus });
initInspectorEvents();



// Scene I/O functions (sceneJson, saveScene, loadSceneData, etc.) live in scene-io.js.


initSceneIO({
  sceneNameInput,
  scenePicker,
  assetPaths,
  planeMeshPath,
  planeRenderObject,
  setStatus,
  loadModel,
  setPlaneScale,
  renderAssets,
  annotateVehicleGroups,
  renderVehicleDefs,
  syncInspector,
  renderPathList,
  invalidateLidarCache,
  clearPathSelection,
  setZonePlacementMode,
  vehicleRepresentatives,
  controllerModeFor,
  vehicleParamsFor,
  sensorsFor,
  selectedPath,
  cameraPosition,
});

startEditorLifecycle({
  gl,
  planeMeshPath,
  loadModel,
  loadAssets,
  loadVehicleDefs,
  loadScenes,
  restoreDraft,
  saveDraft,
  syncInspector,
  setStatus,
  render,
});
