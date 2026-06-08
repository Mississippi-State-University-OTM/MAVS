// Scene serialization, persistence, and server API calls.
import { state } from '../core/state.js';
import { zoneCount } from '../zones/zones.js';
import { colorForPath } from './objects.js';

// Injected by app.js via initSceneIO() to avoid circular imports.
let sceneNameInput, scenePicker, assetPaths;
let planeMeshPath, planeRenderObject;
let setStatus, loadModel, setPlaneScale;
let renderAssets, annotateVehicleGroups, renderVehicleDefs;
let syncInspector, renderPathList, invalidateLidarCache, clearPathSelection;
let setZonePlacementMode;
let vehicleRepresentatives, controllerModeFor, vehicleParamsFor, sensorsFor;
let selectedPath, cameraPosition;

export function initSceneIO(ctx) {
  ({
    sceneNameInput, scenePicker, assetPaths,
    planeMeshPath, planeRenderObject,
    setStatus, loadModel, setPlaneScale,
    renderAssets, annotateVehicleGroups, renderVehicleDefs,
    syncInspector, renderPathList, invalidateLidarCache, clearPathSelection,
    setZonePlacementMode,
    vehicleRepresentatives, controllerModeFor, vehicleParamsFor, sensorsFor,
    selectedPath, cameraPosition,
  } = ctx);
}

// --- Scene JSON serialization ---

export function sceneJson(excludeAssignedVehicles = false) {
  const grouped = new Map();
  for (const object of state.objects) {
    if (object.vehicleRole === "camera_ghost" || object.vehicleRole === "sensor_ghost") continue;
    if (excludeAssignedVehicles && object.vehicleDefName) {
      continue;
    }
    const meshPath = sceneMeshPath(object.mesh);
    if (!grouped.has(meshPath)) {
      grouped.set(meshPath, []);
    }
    grouped.get(meshPath).push({
      YawPitchRoll: [object.rotation[2], object.rotation[1], object.rotation[0]],
      Position: object.position,
      Scale: object.scale,
    });
  }

  const sceneObjects = [
    {
      Mesh: sceneMeshPath(planeMeshPath),
      "Rotate Y to Z": false,
      "Smooth Normals": false,
      Instances: [
        {
          YawPitchRoll: [0, 0, 0],
          Position: [0, 0, 0],
          Scale: [...planeRenderObject.scale],
        },
      ],
    },
  ];

  for (const [mesh, instances] of grouped.entries()) {
    sceneObjects.push({
      Mesh: mesh,
      "Rotate Y to Z": false,
      Instances: instances,
    });
  }
  for (const zone of state.randomZones) {
    const random = {
      Offset: [0, 0, zone.offsetZ],
      Number: zoneCount(zone),
      Polygon: [
        [zone.minX, zone.minY],
        [zone.maxX, zone.minY],
        [zone.maxX, zone.maxY],
        [zone.minX, zone.maxY],
      ],
      Scale: [zone.scaleMin, zone.scaleMax],
    };
    if (zone.minimumSpacing > 0) {
      random["Minimum Spacing"] = zone.minimumSpacing;
    }
    sceneObjects.push({
      Mesh: sceneMeshPath(zone.mesh),
      "Rotate Y to Z": false,
      Random: random,
      "_Scene Builder Zone": {
        Placement: zone.placement,
        Density: zone.density,
      },
    });
  }

  return {
    "Object Labels": "labels.json",
    "Surface Mesh": [
      {
        Mesh: sceneMeshPath(planeMeshPath),
        "Rotate Y to Z": false,
        YawPitchRoll: [0, 0, 0],
        Position: [0, 0, 0],
        Scale: [...planeRenderObject.scale],
        Material: "dry",
      },
    ],
    Objects: sceneObjects,
  };
}

export function simulationActorsJson() {
  const assigned = state.objects.filter((obj) => obj.pathId != null && !obj.vehicleDefName);
  const grouped = new Map();
  for (const obj of assigned) {
    const path = state.paths.find((p) => p.id === obj.pathId);
    if (!path) continue;
    if (!grouped.has(obj.mesh)) grouped.set(obj.mesh, []);
    grouped.get(obj.mesh).push({ obj, path });
  }
  const actors = [];
  for (const [mesh, entries] of grouped.entries()) {
    actors.push({
      Mesh: sceneMeshPath(mesh),
      "Rotate Y to Z": false,
      "Locked to Ground": true,
      Instances: entries.map(({ obj, path }) => ({
        "Initial Position": [...obj.position],
        Offset: [0, 0, 0],
        Scale: [...obj.scale],
        Speed: 5.0,
        "Waypoints File": `waypoints/${path.name}`,
      })),
    });
  }
  return { Actors: actors };
}

export function simulationVehiclesJson() {
  const vehicles = [];
  for (const chassis of vehicleRepresentatives()) {
    const controllerMode = controllerModeFor(chassis);
    const path = controllerMode === "path"
      ? state.paths.find((candidate) => candidate.id === chassis.pathId)
      : null;
    const def = state.vehicleDefs.find((candidate) => candidate.name === chassis.vehicleDefName);
    if (!def || (controllerMode === "path" && (!path || path.waypoints.length < 2))) continue;
    let cameraOffset;
    if (controllerMode === "human") {
      const ghost = state.objects.find(o => o.vehicleRole === "camera_ghost" && o.vehicleGroupId === chassis.vehicleGroupId);
      if (ghost) {
        const vp = vehicleInitialPosition(chassis, def);
        const h = (chassis.rotation[2] - (def.chassis_rotation?.[2] || 0)) * Math.PI / 180;
        const dx = ghost.position[0] - vp[0];
        const dy = ghost.position[1] - vp[1];
        cameraOffset = [
          dx * Math.cos(h) + dy * Math.sin(h),
          -dx * Math.sin(h) + dy * Math.cos(h),
          ghost.position[2],
        ];
      }
    }
    const sensorList = sensorsFor(chassis).map(s => ({
      Name: s.name,
      Type: s.type,
      ...(s.model ? { Model: s.model } : {}),
      Offset: [...s.offset],
      Orientation: eulerToQuat(s.yaw || 0, s.pitch || 0, s.roll || 0),
      "Repitition Rate (Hz)": s.hz,
    }));
    vehicles.push({
      definition_file: def.definition_file,
      initial_position: vehicleInitialPosition(chassis, def),
      initial_heading_degrees: chassis.rotation[2] - (def.chassis_rotation?.[2] || 0),
      controller_mode: controllerMode,
      waypoints: path?.waypoints || [],
      wheelbase: def.wheelbase,
      ...vehicleParamsFor(chassis),
      ...(cameraOffset ? { camera_offset: cameraOffset } : {}),
      ...(sensorList.length ? { sensors: sensorList } : {}),
    });
  }
  return { vehicles };
}

export function vehicleInitialPosition(chassis, def) {
  return [
    chassis.position[0] - def.chassis_offset[0],
    chassis.position[1] - def.chassis_offset[1],
    0,
  ];
}

// --- Server API calls ---

export async function saveWaypoints() {
  const path = selectedPath();
  if (!path || !path.waypoints.length) {
    setStatus(path ? "No waypoints to save" : "No path selected");
    return;
  }
  setStatus("Saving waypoints...");
  try {
    const response = await fetch("/api/save-waypoints", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name: path.name, waypoints: path.waypoints }),
    });
    const result = await response.json();
    if (!response.ok || !result.saved) {
      setStatus(result.error || "Save failed", true);
      return;
    }
    setStatus(`Saved ${result.relative_path}`);
  } catch (error) {
    setStatus(`Save failed: ${error.message}`, true);
  }
}

export async function exportSimulation() {
  const vehicles = simulationVehiclesJson();
  const pathVehicle = vehicles.vehicles.find(
    (v) => v.controller_mode === "path" && v.waypoints.length >= 2
  );
  if (!pathVehicle) {
    setStatus("Export Sim requires a vehicle preset assigned to a path with at least 2 waypoints", true);
    return;
  }
  const baseName = sceneNameInput.value.replace(/\.json$/i, "");
  // Resolve the waypoints filename for the primary vehicle's path
  const representatives = vehicleRepresentatives();
  const chassis = representatives.find((c) => controllerModeFor(c) === "path");
  const path = chassis ? state.paths.find((p) => p.id === chassis.pathId) : null;
  const primaryPathName = path ? path.name : (state.paths[0]?.name || "path.json");
  setStatus("Exporting simulation config...");
  try {
    const response = await fetch("/api/export-sim-config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        name: baseName,
        scene: sceneJson(true),
        vehicles,
        waypoints: state.paths.map((p) => ({ name: p.name, waypoints: p.waypoints })),
        primary_path_name: primaryPathName,
      }),
    });
    const result = await response.json();
    if (!response.ok || !result.saved) {
      setStatus(result.error || "Export failed", true);
      return;
    }
    setStatus(`Exported ${result.sim_config_path} — run: mavs_simulation.exe ${result.sim_config_path}`);
  } catch (error) {
    setStatus(`Export failed: ${error.message}`, true);
  }
}

export async function runSimulation() {
  const vehicles = simulationVehiclesJson();
  if (!vehicles.vehicles.length) {
    setStatus("Run Simulation requires at least one vehicle preset", true);
    return;
  }
  const baseName = sceneNameInput.value.replace(/\.json$/i, "");
  setStatus("Starting simulation...");
  try {
    const response = await fetch("/api/run-simulation", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        name: baseName,
        scene: sceneJson(true),
        vehicles,
        waypoints: state.paths.map((p) => ({ name: p.name, waypoints: p.waypoints })),
      }),
    });
    const result = await response.json();
    if (!response.ok || !result.started) {
      setStatus(result.error || "Simulation failed to start", true);
      return;
    }
    setStatus("Simulation running");
  } catch (error) {
    setStatus(`Simulation failed: ${error.message}`, true);
  }
}

export async function saveScene() {
  const name = sceneNameInput.value;
  setStatus("Saving...");
  const response = await fetch("/api/save-scene", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ name, scene: sceneJson() }),
  });
  const result = await response.json();
  if (!response.ok || !result.saved) {
    setStatus(result.error || "Save failed", true);
    return;
  }
  setStatus(`Saved ${result.relative_path}`);
  loadScenes().catch(() => {});
}

export async function previewScene() {
  setStatus("Starting MAVS preview...");
  const eye = cameraPosition();
  const previewCamera = {
    position: eye,
    heading: state.camera.yaw + Math.PI,
    pitch: state.camera.pitch,
  };
  try {
    const response = await fetch("/api/preview-scene", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name: sceneNameInput.value, scene: sceneJson(), camera: previewCamera }),
    });
    const result = await response.json();
    if (!response.ok || !result.started) {
      setStatus(result.error || "Preview failed", true);
      return;
    }
    setStatus(`Preview launched for ${result.scene}`);
  } catch (error) {
    setStatus(`Preview request failed: ${error.message}. Is the scene-builder server running?`, true);
  }
}

export async function loadAssets() {
  const response = await fetch("/api/assets");
  if (!response.ok) {
    throw new Error(`Could not load assets (HTTP ${response.status}) - is the Python server running on port 8765?`);
  }
  const result = await response.json();
  state.assets = result.assets;
  assetPaths.innerHTML = "";
  for (const asset of state.assets) {
    const option = document.createElement("option");
    option.value = asset.path;
    assetPaths.appendChild(option);
  }
  renderAssets();
}

export async function loadVehicleDefs() {
  const response = await fetch("/api/vehicle-defs");
  if (!response.ok) return;
  const result = await response.json();
  state.vehicleDefs = result.vehicles || [];
  annotateVehicleGroups();
  renderVehicleDefs();
}

export async function loadScenes() {
  const response = await fetch("/api/scenes");
  if (!response.ok) {
    throw new Error(`Could not load scenes (HTTP ${response.status})`);
  }
  const result = await response.json();
  state.savedScenes = result.scenes || [];
  scenePicker.innerHTML = "";
  const placeholder = document.createElement("option");
  placeholder.value = "";
  placeholder.textContent = state.savedScenes.length ? "Choose scene..." : "No saved scenes";
  scenePicker.appendChild(placeholder);
  for (const scene of state.savedScenes) {
    const option = document.createElement("option");
    option.value = scene.path;
    option.textContent = scene.name;
    scenePicker.appendChild(option);
  }
}

export async function loadSceneFromPicker() {
  setZonePlacementMode(false);
  const path = scenePicker.value;
  if (!path) {
    setStatus("Choose a scene to load");
    return;
  }
  setStatus(`Loading ${path}`);
  const response = await fetch(`/api/scene?path=${encodeURIComponent(path)}`);
  const result = await response.json();
  if (!response.ok || result.error) {
    setStatus(result.error || "Scene load failed", true);
    return;
  }
  await loadSceneData(result.scene, result.name || path);
}

export async function loadSceneData(scene, name) {
  const nextObjects = [];
  const nextZones = [];
  const loadPromises = [];
  state.nextId = 1;
  state.nextZoneId = 1;
  for (const sceneObject of scene.Objects || []) {
    const mesh = editorMeshPath(sceneObject.Mesh || "");
    if (!mesh || sceneMeshPath(mesh) === sceneMeshPath(planeMeshPath)) {
      const firstInst = (sceneObject.Instances || [])[0];
      if (firstInst && Array.isArray(firstInst.Scale) && firstInst.Scale[0] > 0) {
        setPlaneScale(firstInst.Scale[0]);
      }
      continue;
    }
    if (sceneObject.Random && Array.isArray(sceneObject.Random.Polygon)) {
      const xs = sceneObject.Random.Polygon.map((point) => Number(point[0])).filter(Number.isFinite);
      const ys = sceneObject.Random.Polygon.map((point) => Number(point[1])).filter(Number.isFinite);
      if (xs.length && ys.length) {
        const metadata = sceneObject["_Scene Builder Zone"] || {};
        const area = (Math.max(...xs) - Math.min(...xs)) * (Math.max(...ys) - Math.min(...ys));
        const number = Math.max(0, Number(sceneObject.Random.Number) || 0);
        const scale = Array.isArray(sceneObject.Random.Scale) && sceneObject.Random.Scale.length === 2
          ? sceneObject.Random.Scale
          : [1, 1];
        nextZones.push({
          id: state.nextZoneId++,
          mesh,
          placement: metadata.Placement === "density" ? "density" : "count",
          density: Number.isFinite(metadata.Density) ? metadata.Density : (area > 0 ? number / area : 0),
          number,
          minimumSpacing: Math.max(0, Number(sceneObject.Random["Minimum Spacing"]) || 0),
          offsetZ: Number(sceneObject.Random.Offset?.[2]) || 0,
          scaleMin: Math.max(0.01, Number(scale[0]) || 1),
          scaleMax: Math.max(0.01, Number(scale[1]) || 1),
          minX: Math.min(...xs),
          maxX: Math.max(...xs),
          minY: Math.min(...ys),
          maxY: Math.max(...ys),
        });
      }
      continue;
    }
    loadPromises.push(loadModel(mesh).catch((error) => {
      setStatus(`Could not load ${mesh}: ${error.message}`, true);
    }));
    for (const instance of sceneObject.Instances || []) {
      const ypr = instance.YawPitchRoll || [0, 0, 0];
      const position = instance.Position || [0, 0, 0];
      const scale = instance.Scale || [1, 1, 1];
      nextObjects.push({
        id: state.nextId++,
        mesh,
        position: [Number(position[0] || 0), Number(position[1] || 0), Number(position[2] || 0)],
        rotation: [Number(ypr[2] || 0), Number(ypr[1] || 0), Number(ypr[0] || 0)],
        scale: [Number(scale[0] || 1), Number(scale[1] || 1), Number(scale[2] || 1)],
        color: colorForPath(mesh),
        pathId: null,
      });
    }
  }
  await Promise.all(loadPromises);
  state.objects = nextObjects;
  state.randomZones = nextZones;
  state.paths = [];
  state.nextPathId = 1;
  state.selectedIds = new Set();
  state.selectedZoneId = null;
  clearPathSelection();
  state.clipboard.copiedObjects = [];
  sceneNameInput.value = name;
  renderPathList();
  syncInspector();
  invalidateLidarCache();
  setStatus(`Loaded ${name} (${state.objects.length} objects, ${state.randomZones.length} random zones)`);
}
