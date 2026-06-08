import { state } from '../core/state.js';
import { lidarProgress } from '../core/dom.js';
import { sensorsFor } from './sensors.js';

let _gl = null;
let _planeRenderObject = null;
let _modelCache = null;
let _modelMatrix = null;
let _objectWorldAABB = null;
let _drawPoints = null;
let _setStatus = () => {};

export function initLidarPreview({ gl, planeRenderObject, modelCache, modelMatrix, objectWorldAABB, drawPoints, setStatus }) {
  _gl = gl;
  _planeRenderObject = planeRenderObject;
  _modelCache = modelCache;
  _modelMatrix = modelMatrix;
  _objectWorldAABB = objectWorldAABB;
  _drawPoints = drawPoints;
  _setStatus = setStatus;
}

function elevLinear(n, lo, hi) {
  return Array.from({length: n}, (_, i) => n === 1 ? lo : lo + i * (hi - lo) / (n - 1));
}

const LIDAR_RING_ANGLES = {
  "VLP-16":  elevLinear(16, -15, 15),
  "HDL-32E": elevLinear(32, -30.67, 10.67),
  "HDL-64E": elevLinear(32, -24.9, 2.0),
  "M8":      elevLinear(8, -4.0, 8.0),
  "OS1":     elevLinear(16, -16.6, 16.6),
  "OS1-16":  elevLinear(16, -16.6, 16.6),
  "OS2":     elevLinear(32, -22.5, 22.5),
  "LMS-291": [0],
  "RS32":    elevLinear(32, -25.0, 15.0),
  "OS0":     elevLinear(32, -45.0, 45.0),
  "BPearl":  elevLinear(32, -90.0, -1.0),
  "FourPi":  elevLinear(16, -90.0, 90.0),
};

function rayTriIntersect(ox, oy, oz, dx, dy, dz, ax, ay, az, bx, by, bz, cx, cy, cz) {
  const e1x = bx-ax, e1y = by-ay, e1z = bz-az;
  const e2x = cx-ax, e2y = cy-ay, e2z = cz-az;
  const hx = dy*e2z - dz*e2y, hy = dz*e2x - dx*e2z, hz = dx*e2y - dy*e2x;
  const det = e1x*hx + e1y*hy + e1z*hz;
  if (Math.abs(det) < 1e-7) return Infinity;
  const inv = 1.0 / det;
  const sx = ox-ax, sy = oy-ay, sz = oz-az;
  const u = inv * (sx*hx + sy*hy + sz*hz);
  if (u < 0 || u > 1) return Infinity;
  const qx = sy*e1z - sz*e1y, qy = sz*e1x - sx*e1z, qz = sx*e1y - sy*e1x;
  const v = inv * (dx*qx + dy*qy + dz*qz);
  if (v < 0 || u + v > 1) return Infinity;
  const t = inv * (e2x*qx + e2y*qy + e2z*qz);
  return t > 1e-4 ? t : Infinity;
}

let lidarSceneVersion = 0;
const lidarWorldTriCache = { version: -1, dirty: false, tris: null };
const lidarPendingAABBs = [];

const lidarWorker = new Worker(new URL('./lidar_worker.js', import.meta.url));
let lidarJobCounter = 0;

const lidarSegCache = new Map();

lidarWorker.onmessage = function(e) {
  const { jobId, ghostId, rings } = e.data;
  const entry = lidarSegCache.get(ghostId);
  if (!entry || entry.pendingJobId !== jobId) return;
  entry.rings = rings;
  delete entry.pendingJobId;
  updateLidarProgress();
};

lidarWorker.onerror = function(error) {
  for (const entry of lidarSegCache.values()) delete entry.pendingJobId;
  updateLidarProgress();
  _setStatus(`LiDAR preview failed: ${error.message || "worker error"}`, true);
};

function updateLidarProgress() {
  const busy = [...lidarSegCache.values()].some(e => e.pendingJobId != null);
  lidarProgress.hidden = !busy;
}

export function invalidateLidarCache() {
  lidarSceneVersion++;
  lidarWorldTriCache.dirty = true;
  lidarPendingAABBs.length = 0;
  for (const entry of lidarSegCache.values()) delete entry.pendingJobId;
  updateLidarProgress();
}

export function invalidateLidarForBounds(aabb) {
  lidarWorldTriCache.dirty = true;
  lidarPendingAABBs.push(aabb);
}

export function clearSensorGhostLidarCache(id) {
  lidarSegCache.delete(id);
}

function aabbIntersectsSphere(aabb, gx, gy, gz, range) {
  const cx = Math.max(aabb.minX, Math.min(gx, aabb.maxX));
  const cy = Math.max(aabb.minY, Math.min(gy, aabb.maxY));
  const cz = Math.max(aabb.minZ, Math.min(gz, aabb.maxZ));
  return (cx - gx) ** 2 + (cy - gy) ** 2 + (cz - gz) ** 2 <= range * range;
}

function pushSampledWorldTriangles(data, object, triangles, maxTriangles = 12000) {
  const m = _modelMatrix(object);
  const step = Math.max(1, Math.ceil(triangles.length / maxTriangles));
  for (let i = 0; i < triangles.length; i += step) {
    const tri = triangles[i];
    const a = transformPoint(m, tri[0]);
    const b = transformPoint(m, tri[1]);
    const c = transformPoint(m, tri[2]);
    data.push(a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2]);
  }
}

function getLidarWorldTris(gx, gy, gz, range, sensorGroupId) {
  const data = [];
  for (const obj of [_planeRenderObject, ...state.objects]) {
    if (obj.vehicleRole === "sensor_ghost" || obj.vehicleRole === "camera_ghost") continue;
    const model = _modelCache.get(obj.mesh);
    if (!model) continue;
    const bounds = _objectWorldAABB(obj);
    if (!bounds || !aabbIntersectsSphere(bounds, gx, gy, gz, range)) continue;
    if (model.localTriangles.length > 50000) {
      // Dense render meshes still need real-surface collision. Box proxies create
      // false rectangular returns and trap sensors mounted inside vehicles.
      const maxTriangles = sensorGroupId != null && obj.vehicleGroupId === sensorGroupId
        ? 12000
        : 3000;
      pushSampledWorldTriangles(data, obj, model.localTriangles, maxTriangles);
      continue;
    }
    const m = _modelMatrix(obj);
    for (const tri of model.localTriangles) {
      const a = transformPoint(m, tri[0]);
      const b = transformPoint(m, tri[1]);
      const c = transformPoint(m, tri[2]);
      data.push(a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2]);
    }
  }
  return new Float32Array(data);
}

function elevColor(t) {
  if (t < 0.25) { const s = t / 0.25; return [0, 0.5 + 0.5*s, 1]; }
  if (t < 0.5)  { const s = (t - 0.25) / 0.25; return [0, 1, 1 - s]; }
  if (t < 0.75) { const s = (t - 0.5)  / 0.25; return [s, 1, 0]; }
  const s = (t - 0.75) / 0.25; return [1, 1 - s, 0];
}

function buildLidarHitPoints(ghost, sensor, chassis) {
  const [gx, gy, gz] = ghost.position;
  const visRange = Math.max(5, sensor.visRange || 20);
  const entry = lidarSegCache.get(ghost.id);

  if (entry) {
    if (entry.pendingJobId != null) {
      return entry.rings;
    }
    if (entry.version === lidarSceneVersion
      && entry.gx === gx && entry.gy === gy && entry.gz === gz
      && entry.visRange === visRange
      && entry.model === sensor.model
      && entry.yaw === sensor.yaw
      && entry.pitch === sensor.pitch
      && entry.roll === sensor.roll) {
      const newAABBs = lidarPendingAABBs.slice(entry.processedAABBCount);
      if (!newAABBs.some(aabb => aabbIntersectsSphere(aabb, gx, gy, gz, visRange))) {
        entry.processedAABBCount = lidarPendingAABBs.length;
        return entry.rings;
      }
    }
  }

  const jobId = ++lidarJobCounter;
  const elevAngles = LIDAR_RING_ANGLES[sensor.model] || elevLinear(16, -15, 15);
  const def = state.vehicleDefs.find(d => d.name === chassis.vehicleDefName);
  const vehicleYaw = (chassis.rotation[2] - (def?.chassis_rotation?.[2] || 0)) * Math.PI / 180;
  const totalYaw = vehicleYaw + (sensor.yaw || 0) * Math.PI / 180;
  const pitchRad = (sensor.pitch || 0) * Math.PI / 180;
  const rollRad  = (sensor.roll  || 0) * Math.PI / 180;

  const tris = getLidarWorldTris(gx, gy, gz, visRange, ghost.vehicleGroupId);
  const trisCopy = new Float32Array(tris);

  const newEntry = {
    pendingJobId: jobId,
    version: lidarSceneVersion,
    gx, gy, gz, visRange,
    model: sensor.model,
    yaw: sensor.yaw,
    pitch: sensor.pitch,
    roll: sensor.roll,
    processedAABBCount: lidarPendingAABBs.length,
    rings: entry?.rings || [],
  };
  lidarSegCache.set(ghost.id, newEntry);

  lidarWorker.postMessage(
    { jobId, ghostId: ghost.id, gx, gy, gz, visRange, tris: trisCopy, elevAngles, totalYaw, pitchRad, rollRad },
    [trisCopy.buffer]
  );
  updateLidarProgress();

  return newEntry.rings;
}

export function drawLidarRings(viewProj) {
  _gl.disable(_gl.DEPTH_TEST);
  for (const ghost of state.objects) {
    if (ghost.vehicleRole !== "sensor_ghost") continue;
    const chassis = state.objects.find(o => o.vehicleGroupId === ghost.vehicleGroupId && o.vehicleRole === "chassis");
    if (!chassis) continue;
    const sensor = sensorsFor(chassis).find(s => s.id === ghost.sensorId);
    if (!sensor || sensor.type !== "lidar" || !sensor.showRings) continue;
    const rings = buildLidarHitPoints(ghost, sensor, chassis);
    const ptSize = Math.max(1, sensor.pointSize || 3);
    for (const { pts, color } of rings) {
      _drawPoints(pts, color, ptSize, viewProj);
    }
  }
  _gl.enable(_gl.DEPTH_TEST);
}
