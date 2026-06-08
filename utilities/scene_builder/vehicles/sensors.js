// Sensor data, vehicle-mounted ghost objects, and offset math.
import { state } from '../core/state.js';
import { objectById } from '../scene/objects.js';
import { controllerModeFor, vehicleGroupObjects } from './vehicles.js';

export const SENSOR_TYPES = ["lidar", "camera", "gps", "compass", "fisheye", "radar", "imu"];

export const SENSOR_MODELS = {
  lidar: ["HDL-32E", "HDL-64E", "M8", "OS1", "OS1-16", "OS2", "LMS-291", "VLP-16", "RS32", "OS0", "BPearl", "FourPi"],
  camera: ["XCD-V60", "Flea", "HD1080", "MachineVision", "HDPathTraced", "HalfHDPathTraced", "Sf3325", "UavCamera"],
  fisheye: [],
  gps: [],
  compass: [],
  radar: [],
  imu: [],
};

let clearSensorGhostLidarCache = () => {};

export function initSensors(ctx = {}) {
  clearSensorGhostLidarCache = ctx.clearSensorGhostLidarCache || clearSensorGhostLidarCache;
}

export function sensorsFor(object) {
  const chassis = vehicleGroupObjects(object).find(o => o.vehicleRole === "chassis") || object;
  return Array.isArray(chassis?.sensors) ? chassis.sensors : [];
}

export function setSensors(object, sensors) {
  const chassis = vehicleGroupObjects(object).find(o => o.vehicleRole === "chassis") || object;
  if (chassis) chassis.sensors = sensors;
}

export function syncCameraGhost(vehicle) {
  const gid = vehicle.vehicleGroupId;
  if (gid == null) return;
  const mode = controllerModeFor(vehicle);
  const existing = state.objects.find(o => o.vehicleRole === "camera_ghost" && o.vehicleGroupId === gid);
  if (mode === "human") {
    if (existing) return;
    const chassis = state.objects.find(o => o.vehicleGroupId === gid && o.vehicleRole === "chassis");
    if (!chassis) return;
    const def = state.vehicleDefs.find(d => d.name === chassis.vehicleDefName);
    const h = (chassis.rotation[2] - (def?.chassis_rotation?.[2] || 0)) * Math.PI / 180;
    state.objects.push({
      id: state.nextId++,
      mesh: "",
      position: [
        chassis.position[0] - 8 * Math.cos(h),
        chassis.position[1] - 8 * Math.sin(h),
        3.5,
      ],
      rotation: [0, 0, 0],
      scale: [1, 1, 1],
      vehicleGroupId: gid,
      vehicleRole: "camera_ghost",
      groupName: chassis.groupName,
      controllerMode: "human",
    });
  } else if (existing) {
    state.objects.splice(state.objects.indexOf(existing), 1);
  }
}

export function syncSensorGhosts(chassis) {
  const gid = chassis.vehicleGroupId;
  const existing = state.objects.filter(o => o.vehicleRole === "sensor_ghost" && o.vehicleGroupId === gid);
  for (const ghost of existing) {
    clearSensorGhostLidarCache(ghost.id);
    state.objects.splice(state.objects.indexOf(ghost), 1);
  }
  const def = state.vehicleDefs.find(d => d.name === chassis.vehicleDefName);
  const co = def?.chassis_offset || [0, 0, 0];
  const cgPos = [chassis.position[0] - co[0], chassis.position[1] - co[1], chassis.position[2] - co[2]];
  const vehicleYaw = (chassis.rotation[2] - (def?.chassis_rotation?.[2] || 0)) * Math.PI / 180;
  const cosH = Math.cos(vehicleYaw), sinH = Math.sin(vehicleYaw);
  for (const sensor of sensorsFor(chassis)) {
    const [ox, oy, oz] = sensor.offset;
    state.objects.push({
      id: state.nextId++,
      mesh: "",
      position: [cgPos[0] + cosH*ox - sinH*oy, cgPos[1] + sinH*ox + cosH*oy, cgPos[2] + oz],
      rotation: [0, 0, 0],
      scale: [1, 1, 1],
      vehicleGroupId: gid,
      vehicleRole: "sensor_ghost",
      sensorId: sensor.id,
      groupName: chassis.groupName,
    });
  }
}

export function syncAllSensorGhosts() {
  for (const chassis of state.objects.filter(o => o.vehicleRole === "chassis")) {
    syncSensorGhosts(chassis);
  }
}

// Cheap position-only update during drag: avoids recreating ghost objects and clearing caches.
export function updateSensorGhostPositions(chassis) {
  const gid = chassis.vehicleGroupId;
  const def = state.vehicleDefs.find(d => d.name === chassis.vehicleDefName);
  const co = def?.chassis_offset || [0, 0, 0];
  const cgPos = [chassis.position[0] - co[0], chassis.position[1] - co[1], chassis.position[2] - co[2]];
  const vehicleYaw = (chassis.rotation[2] - (def?.chassis_rotation?.[2] || 0)) * Math.PI / 180;
  const cosH = Math.cos(vehicleYaw), sinH = Math.sin(vehicleYaw);
  for (const ghost of state.objects) {
    if (ghost.vehicleRole !== "sensor_ghost" || ghost.vehicleGroupId !== gid) continue;
    const sensor = sensorsFor(chassis).find(s => s.id === ghost.sensorId);
    if (!sensor) continue;
    const [ox, oy, oz] = sensor.offset;
    ghost.position = [cgPos[0] + cosH*ox - sinH*oy, cgPos[1] + sinH*ox + cosH*oy, cgPos[2] + oz];
  }
}

export function syncGhostPositionsForTransforms(transforms) {
  const seen = new Set();
  for (const { id } of transforms) {
    const obj = objectById(id);
    if (!obj || obj.vehicleGroupId == null) continue;
    if (seen.has(obj.vehicleGroupId)) continue;
    seen.add(obj.vehicleGroupId);
    const chassis = state.objects.find(o => o.vehicleGroupId === obj.vehicleGroupId && o.vehicleRole === "chassis");
    if (chassis) updateSensorGhostPositions(chassis);
  }
}

export function updateSensorOffsetFromGhost(ghost) {
  const chassis = state.objects.find(o => o.vehicleGroupId === ghost.vehicleGroupId && o.vehicleRole === "chassis");
  if (!chassis) return;
  const def = state.vehicleDefs.find(d => d.name === chassis.vehicleDefName);
  const co = def?.chassis_offset || [0, 0, 0];
  const cgPos = [chassis.position[0] - co[0], chassis.position[1] - co[1], chassis.position[2] - co[2]];
  const vehicleYaw = (chassis.rotation[2] - (def?.chassis_rotation?.[2] || 0)) * Math.PI / 180;
  const cosH = Math.cos(vehicleYaw), sinH = Math.sin(vehicleYaw);
  const wx = ghost.position[0] - cgPos[0];
  const wy = ghost.position[1] - cgPos[1];
  const wz = ghost.position[2] - cgPos[2];
  const sensor = sensorsFor(chassis).find(s => s.id === ghost.sensorId);
  if (!sensor) return;
  sensor.offset[0] = Math.round((cosH*wx + sinH*wy) * 100) / 100;
  sensor.offset[1] = Math.round((-sinH*wx + cosH*wy) * 100) / 100;
  sensor.offset[2] = Math.round(wz * 100) / 100;
}
