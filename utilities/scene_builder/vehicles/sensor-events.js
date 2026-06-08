import { state } from '../core/state.js';
import { lidarVizBtn, cameraVizBtn, addSensorBtn, sensorItems } from '../core/dom.js';
import { selectedObjects, selectObject } from '../scene/selection.js';
import {
  SENSOR_MODELS, sensorsFor, setSensors,
  syncSensorGhosts, updateSensorOffsetFromGhost,
} from './sensors.js';
import { vehicleGroupObjects } from './vehicles.js';
import { modelCache } from '../rendering/model-loader.js';
import { modelMatrix } from '../scene/objects.js';
import { invalidateLidarCache, clearSensorGhostLidarCache } from './lidar-preview.js';
import { renderSensorPanel, syncVizToggleBar } from './sensor-ui.js';
import { syncInspector } from '../ui/inspector.js';
import { saveDraft } from '../scene/draft-storage.js';
import { listen } from '../core/events.js';

let _setStatus = () => {};

function selectedVehicle() {
  const sel = selectedObjects();
  const direct = sel.find(o => o.vehicleDefName);
  if (direct) return direct;
  const ghost = sel.find(o => o.vehicleRole === "sensor_ghost" || o.vehicleRole === "camera_ghost");
  if (ghost) return state.objects.find(o => o.vehicleGroupId === ghost.vehicleGroupId && o.vehicleRole === "chassis") || null;
  return null;
}

export function initSensorEvents({ setStatus }) {
  _setStatus = setStatus;

  listen(lidarVizBtn, "click", () => {
    const vehicle = selectedVehicle();
    if (!vehicle) return;
    const lidars = sensorsFor(vehicle).filter(s => s.type === "lidar");
    const next = !lidars.some(s => s.showRings);
    for (const s of lidars) s.showRings = next;
    invalidateLidarCache();
    syncVizToggleBar(vehicle);
    syncInspector();
    saveDraft();
  });

  listen(cameraVizBtn, "click", () => {
    const vehicle = selectedVehicle();
    if (!vehicle) return;
    const cameras = sensorsFor(vehicle).filter(s => s.type === "camera");
    const next = !cameras.some(s => s.showFov);
    for (const s of cameras) s.showFov = next;
    syncVizToggleBar(vehicle);
    syncInspector();
    saveDraft();
  });

  listen(addSensorBtn, "click", () => {
    const vehicle = selectedObjects().find(o => o.vehicleDefName);
    if (!vehicle) return;
    const sensors = sensorsFor(vehicle);
    sensors.push({
      id: state.nextSensorId++,
      name: `lidar_${sensors.length + 1}`,
      type: "lidar",
      model: SENSOR_MODELS.lidar[0],
      offset: [0, 0, 0],
      yaw: 0,
      pitch: 0,
      roll: 0,
      hz: 10,
      showRings: false,
      visRange: 20,
    });
    setSensors(vehicle, sensors);
    const addChassis = vehicleGroupObjects(vehicle).find(o => o.vehicleRole === "chassis") || vehicle;
    syncSensorGhosts(addChassis);
    renderSensorPanel(vehicle);
    saveDraft();
  });

  listen(sensorItems, "change", (e) => {
    const item = e.target.closest(".sensor-item");
    if (!item) return;
    const vehicle = selectedObjects().find(o => o.vehicleDefName)
      || state.objects.find(o => o.vehicleRole === "chassis" && vehicleGroupObjects(o).some(g => g.vehicleRole === "sensor_ghost" && state.selectedIds.has(g.id)));
    if (!vehicle) return;
    const sensors = sensorsFor(vehicle);
    const sensor = sensors.find(s => String(s.id) === item.dataset.sensorId);
    if (!sensor) return;
    const cl = e.target.classList;
    const needsGhostSync = cl.contains("sensor-ox") || cl.contains("sensor-oy") || cl.contains("sensor-oz");
    const needsLidarSync = sensor.type === "lidar" && (
      cl.contains("sensor-model")
      || cl.contains("sensor-ox")
      || cl.contains("sensor-oy")
      || cl.contains("sensor-oz")
      || cl.contains("sensor-yaw")
      || cl.contains("sensor-pitch")
      || cl.contains("sensor-roll")
    );
    if (cl.contains("sensor-name")) sensor.name = e.target.value;
    else if (cl.contains("sensor-type")) {
      const oldType = sensor.type;
      const newType = e.target.value;
      sensor.type = newType;
      const nameMatch = sensor.name.match(/^([a-z]+)_(\d+)$/);
      if (nameMatch && nameMatch[1] === oldType) sensor.name = `${newType}_${nameMatch[2]}`;
      const models = SENSOR_MODELS[sensor.type] || [];
      sensor.model = models[0] || "";
      setSensors(vehicle, sensors);
      const typeChassis = vehicleGroupObjects(vehicle).find(o => o.vehicleRole === "chassis") || vehicle;
      syncSensorGhosts(typeChassis);
      invalidateLidarCache();
      renderSensorPanel(vehicle);
      return;
    } else if (cl.contains("sensor-model")) sensor.model = e.target.value;
    else if (cl.contains("sensor-ox")) sensor.offset[0] = Number(e.target.value) || 0;
    else if (cl.contains("sensor-oy")) sensor.offset[1] = Number(e.target.value) || 0;
    else if (cl.contains("sensor-oz")) sensor.offset[2] = Number(e.target.value) || 0;
    else if (cl.contains("sensor-yaw")) sensor.yaw = Number(e.target.value) || 0;
    else if (cl.contains("sensor-pitch")) sensor.pitch = Number(e.target.value) || 0;
    else if (cl.contains("sensor-roll")) sensor.roll = Number(e.target.value) || 0;
    else if (cl.contains("sensor-hz")) sensor.hz = Math.max(0.1, Number(e.target.value) || 1);
    else if (cl.contains("sensor-rings")) {
      sensor.showRings = e.target.checked;
      invalidateLidarCache();
      renderSensorPanel(vehicle, sensor.id);
      saveDraft();
      return;
    } else if (cl.contains("sensor-fov")) {
      sensor.showFov = e.target.checked;
      saveDraft();
      return;
    } else if (cl.contains("sensor-vis-range")) { sensor.visRange = Math.max(5, Number(e.target.value) || 20); invalidateLidarCache(); }
    else if (cl.contains("sensor-point-size")) { sensor.pointSize = Math.max(1, Number(e.target.value) || 3); }
    setSensors(vehicle, sensors);
    if (needsGhostSync) {
      const changeChassis = vehicleGroupObjects(vehicle).find(o => o.vehicleRole === "chassis") || vehicle;
      syncSensorGhosts(changeChassis);
    }
    if (needsLidarSync) invalidateLidarCache();
    saveDraft();
  });

  listen(sensorItems, "click", (e) => {
    const item = e.target.closest(".sensor-item");
    if (!item) return;
    const vehicle = selectedObjects().find(o => o.vehicleDefName)
      || state.objects.find(o => o.vehicleRole === "chassis" && vehicleGroupObjects(o).some(g => g.vehicleRole === "sensor_ghost" && state.selectedIds.has(g.id)));
    if (!vehicle) return;

    if (e.target.classList.contains("sensor-collapse")) {
      const sensors = sensorsFor(vehicle);
      const sensor = sensors.find(s => String(s.id) === item.dataset.sensorId);
      if (!sensor) return;
      sensor.collapsed = !sensor.collapsed;
      setSensors(vehicle, sensors);
      const selId = [...state.selectedIds].length === 1
        ? state.objects.find(o => state.selectedIds.has(o.id) && o.vehicleRole === "sensor_ghost")?.sensorId ?? null
        : null;
      renderSensorPanel(vehicle, selId);
      saveDraft();
      return;
    }

    if (e.target.classList.contains("sensor-snap")) {
      const ghost = state.objects.find(o => o.vehicleRole === "sensor_ghost"
        && o.vehicleGroupId === vehicle.vehicleGroupId
        && String(o.sensorId) === item.dataset.sensorId);
      if (!ghost) return;
      const vehicleObjs = state.objects.filter(o =>
        o.vehicleGroupId === vehicle.vehicleGroupId &&
        o.vehicleRole !== "sensor_ghost" &&
        o.vehicleRole !== "camera_ghost");
      const [gx, gy] = ghost.position;
      let best = null;
      for (const obj of vehicleObjs) {
        const model = modelCache.get(obj.mesh);
        if (!model) continue;
        const m = modelMatrix(obj);
        for (const tri of model.localTriangles) {
          const a = transformPoint(m, tri[0]);
          const b = transformPoint(m, tri[1]);
          const c = transformPoint(m, tri[2]);
          const z = zOnTriangleAtXY([gx, gy], a, b, c);
          if (z !== null && (best === null || z > best)) best = z;
        }
      }
      if (best !== null) {
        ghost.position[2] = best;
        updateSensorOffsetFromGhost(ghost);
        syncInspector();
        saveDraft();
        _setStatus("Sensor snapped to vehicle mesh");
      } else {
        _setStatus("No vehicle mesh surface found below sensor");
      }
      return;
    }

    if (e.target.classList.contains("sensor-remove")) {
      const removedSensorId = Number(item.dataset.sensorId);
      const removedGhost = state.objects.find(o => o.vehicleRole === "sensor_ghost"
        && o.vehicleGroupId === vehicle.vehicleGroupId && o.sensorId === removedSensorId);
      if (removedGhost) clearSensorGhostLidarCache(removedGhost.id);
      const sensors = sensorsFor(vehicle).filter(s => String(s.id) !== item.dataset.sensorId);
      setSensors(vehicle, sensors);
      const rmChassis = vehicleGroupObjects(vehicle).find(o => o.vehicleRole === "chassis") || vehicle;
      syncSensorGhosts(rmChassis);
      invalidateLidarCache();
      renderSensorPanel(vehicle);
      saveDraft();
      return;
    }

    if (e.target.tagName !== "BUTTON" && e.target.tagName !== "INPUT" && e.target.tagName !== "SELECT") {
      const ghost = state.objects.find(o => o.vehicleRole === "sensor_ghost"
        && o.vehicleGroupId === vehicle.vehicleGroupId
        && String(o.sensorId) === item.dataset.sensorId);
      if (ghost) selectObject(ghost.id);
    }
  });
}
