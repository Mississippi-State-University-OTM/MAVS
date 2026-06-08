import { state } from '../core/state.js';
import { gl } from '../core/dom.js';
import { loc, gizmoBuffer, cylinderBuffer, cylinderVertexCount } from '../rendering/renderer.js';
import { sensorsFor } from './sensors.js';

// tan(hfov/2) and tan(vfov/2) from camera_models.h: array_size / (2 * focal_length)
const CAMERA_FOV = {
  "XCD-V60":          { th: 0.0012  / 0.0035, tv: 0.0009   / 0.0035 }, // 37.9° x 28.9°
  "Flea":             { th: 0.003392/ 0.004,  tv: 0.002714  / 0.004  }, // 80.6° x 68.3°
  "HD1080":           { th: 0.01125 / 0.009,  tv: 0.0075    / 0.009  }, // 102.7° x 79.6°
  "MachineVision":    { th: 0.00175 / 0.0035, tv: 0.00175   / 0.0035 }, // 53.1° x 53.1°
  "HDPathTraced":     { th: 0.01125 / 0.009,  tv: 0.0075    / 0.009  }, // same sensor as HD1080
  "HalfHDPathTraced": { th: 0.01125 / 0.009,  tv: 0.0075    / 0.009  }, // same sensor, half res
  "Sf3325":           { th: 0.002892/ 0.005,  tv: 0.001812  / 0.005  }, // 60.1° x 39.9°
  "UavCamera":        { th: 0.02784 / 0.035,  tv: 0.01856   / 0.035  }, // 77.0° x 55.9°
};

const SENSOR_GHOST_COLORS = {
  lidar:   [1.0, 0.60, 0.10],
  camera:  [0.30, 0.70, 1.0],
  radar:   [1.0, 0.20, 0.50],
  gps:     [0.20, 1.0, 0.40],
  compass: [0.20, 0.90, 0.90],
  imu:     [1.0, 1.0, 0.30],
  fisheye: [0.80, 0.30, 1.0],
};

function drawLineSegments(points, color, viewProj) {
  gl.uniformMatrix4fv(loc.model, false, new Float32Array(mat4Identity()));
  gl.uniformMatrix4fv(loc.viewProj, false, new Float32Array(viewProj));
  gl.uniform3fv(loc.color, new Float32Array(color));
  gl.uniform1i(loc.selected, 0);
  gl.bindBuffer(gl.ARRAY_BUFFER, gizmoBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(points), gl.DYNAMIC_DRAW);
  gl.enableVertexAttribArray(loc.position);
  gl.vertexAttribPointer(loc.position, 3, gl.FLOAT, false, 0, 0);
  gl.disableVertexAttribArray(loc.normal);
  gl.vertexAttrib3f(loc.normal, 0, 0, 1);
  gl.drawArrays(gl.LINES, 0, points.length / 3);
}

function drawSensorCylinder(pos, color, isSelected, viewProj) {
  gl.uniformMatrix4fv(loc.model, false, new Float32Array(mat4Translate(pos)));
  gl.uniformMatrix4fv(loc.viewProj, false, new Float32Array(viewProj));
  gl.uniform3fv(loc.color, new Float32Array(color));
  gl.uniform1i(loc.selected, isSelected ? 1 : 0);
  gl.bindBuffer(gl.ARRAY_BUFFER, cylinderBuffer);
  gl.enableVertexAttribArray(loc.position);
  gl.enableVertexAttribArray(loc.normal);
  gl.vertexAttribPointer(loc.position, 3, gl.FLOAT, false, 24, 0);
  gl.vertexAttribPointer(loc.normal, 3, gl.FLOAT, false, 24, 12);
  gl.drawArrays(gl.TRIANGLES, 0, cylinderVertexCount);
}

export function drawCameraGhosts(viewProj) {
  const ghosts = state.objects.filter(o => o.vehicleRole === "camera_ghost");
  if (!ghosts.length) return;
  gl.disable(gl.DEPTH_TEST);
  for (const ghost of ghosts) {
    const chassis = state.objects.find(o => o.vehicleGroupId === ghost.vehicleGroupId && o.vehicleRole === "chassis");
    if (!chassis) continue;
    const isSelected = state.selectedIds.has(ghost.id);
    const color = isSelected ? [1.0, 0.86, 0.25] : [0.45, 0.82, 1.0];
    const dimColor = color.map(v => v * 0.38);

    const g = ghost.position;
    const t = chassis.position;

    // Local camera frame: forward = ghost → chassis
    const fwd = norm([t[0] - g[0], t[1] - g[1], t[2] - g[2]]);
    let rightRaw = cross(fwd, [0, 0, 1]);
    if (Math.hypot(...rightRaw) < 0.01) rightRaw = cross(fwd, [1, 0, 0]);
    const right = norm(rightRaw);
    const up = cross(right, fwd);

    // Frustum: 50° horiz FOV, 16:9 aspect
    const dist = Math.hypot(t[0] - g[0], t[1] - g[1], t[2] - g[2]) || 1;
    const hw = dist * Math.tan(25 * Math.PI / 180);
    const hv = hw * 9 / 16;
    const fc = [g[0] + fwd[0] * dist, g[1] + fwd[1] * dist, g[2] + fwd[2] * dist];
    const fcorners = [[-1, 1], [1, 1], [1, -1], [-1, -1]].map(([sr, su]) => [
      fc[0] + right[0] * hw * sr + up[0] * hv * su,
      fc[1] + right[1] * hw * sr + up[1] * hv * su,
      fc[2] + right[2] * hw * sr + up[2] * hv * su,
    ]);
    const frustumPts = [];
    for (const c of fcorners) frustumPts.push(...g, ...c);
    for (let i = 0; i < 4; i++) { const a = fcorners[i], b = fcorners[(i + 1) % 4]; frustumPts.push(...a, ...b); }
    drawLineSegments(frustumPts, dimColor, viewProj);

    // Camera body box
    const bw = 0.6, bh = 0.4, bd = 0.25;
    const bc = [];
    for (const sr of [-1, 1]) for (const su of [-1, 1]) for (const sd of [-1, 1]) {
      bc.push([
        g[0] + right[0]*bw*sr + up[0]*bh*su + fwd[0]*bd*sd,
        g[1] + right[1]*bw*sr + up[1]*bh*su + fwd[1]*bd*sd,
        g[2] + right[2]*bw*sr + up[2]*bh*su + fwd[2]*bd*sd,
      ]);
    }
    const bodyPts = [];
    for (const [a, b] of [[0,1],[2,3],[4,5],[6,7],[0,2],[1,3],[4,6],[5,7],[0,4],[1,5],[2,6],[3,7]]) {
      bodyPts.push(...bc[a], ...bc[b]);
    }
    drawLineSegments(bodyPts, color, viewProj);
  }
  gl.enable(gl.DEPTH_TEST);
}

export function drawSensorGhosts(viewProj) {
  const ghosts = state.objects.filter(o => o.vehicleRole === "sensor_ghost");
  if (!ghosts.length) return;
  gl.disable(gl.DEPTH_TEST);
  for (const ghost of ghosts) {
    const chassis = state.objects.find(o => o.vehicleGroupId === ghost.vehicleGroupId && o.vehicleRole === "chassis");
    if (!chassis) continue;
    const sensor = sensorsFor(chassis).find(s => s.id === ghost.sensorId);
    if (!sensor) continue;
    const def = state.vehicleDefs.find(d => d.name === chassis.vehicleDefName);
    const co = def?.chassis_offset || [0, 0, 0];
    const cgPos = [chassis.position[0]-co[0], chassis.position[1]-co[1], chassis.position[2]-co[2]];
    const isSelected = state.selectedIds.has(ghost.id);
    const base = SENSOR_GHOST_COLORS[sensor.type] || [0.8, 0.8, 0.8];
    const color = isSelected ? base.map(v => Math.min(1, v + 0.25)) : base;
    const dim = color.map(v => v * 0.35);
    const wp = ghost.position;
    drawLineSegments([cgPos[0], cgPos[1], cgPos[2], wp[0], wp[1], wp[2]], dim, viewProj);
    gl.enable(gl.DEPTH_TEST);
    drawSensorCylinder(wp, color, isSelected, viewProj);
    gl.disable(gl.DEPTH_TEST);
  }
  gl.enable(gl.DEPTH_TEST);
}

export function drawCameraFovGhosts(viewProj) {
  gl.disable(gl.DEPTH_TEST);
  for (const ghost of state.objects) {
    if (ghost.vehicleRole !== "sensor_ghost") continue;
    const chassis = state.objects.find(o => o.vehicleGroupId === ghost.vehicleGroupId && o.vehicleRole === "chassis");
    if (!chassis) continue;
    const sensor = sensorsFor(chassis).find(s => s.id === ghost.sensorId);
    if (!sensor || sensor.type !== "camera" || !sensor.showFov) continue;

    const isSelected = state.selectedIds.has(ghost.id);
    const base = SENSOR_GHOST_COLORS.camera;
    const color = isSelected ? base.map(v => Math.min(1, v + 0.25)) : base;
    const dimColor = color.map(v => v * 0.4);

    const def = state.vehicleDefs.find(d => d.name === chassis.vehicleDefName);
    const vehicleYaw = (chassis.rotation[2] - (def?.chassis_rotation?.[2] || 0)) * Math.PI / 180;
    const totalYaw = vehicleYaw + (sensor.yaw || 0) * Math.PI / 180;
    const pitchRad = (sensor.pitch || 0) * Math.PI / 180;
    const rollRad  = (sensor.roll  || 0) * Math.PI / 180;
    const cya = Math.cos(totalYaw), sya = Math.sin(totalYaw);
    const cpa = Math.cos(-pitchRad), spa = Math.sin(-pitchRad);
    const cra = Math.cos(rollRad),   sra = Math.sin(rollRad);

    // Rotate from sensor-local (+X forward, +Z up) to world frame
    const rotSensor = (lx, ly, lz) => {
      let rly = cra*ly - sra*lz, rlz = sra*ly + cra*lz;
      let rlx2 = cpa*lx + spa*rlz, rlz2 = -spa*lx + cpa*rlz;
      let rlx3 = cya*rlx2 - sya*rly, rly3 = sya*rlx2 + cya*rly;
      return [rlx3, rly3, rlz2];
    };

    const fwd = rotSensor(1, 0, 0);
    let rightRaw = cross(fwd, [0, 0, 1]);
    if (Math.hypot(...rightRaw) < 0.01) rightRaw = cross(fwd, [1, 0, 0]);
    const right = norm(rightRaw);
    const up = cross(right, fwd);

    const [gx, gy, gz] = ghost.position;
    const dist = 3.0;
    const fov = CAMERA_FOV[sensor.model] || { th: 1.0, tv: 0.5625 }; // fallback ~90°×56°
    const hw = dist * fov.th;
    const hv = dist * fov.tv;
    const fc = [gx + fwd[0]*dist, gy + fwd[1]*dist, gz + fwd[2]*dist];
    const fcorners = [[-1, 1], [1, 1], [1, -1], [-1, -1]].map(([sr, su]) => [
      fc[0] + right[0]*hw*sr + up[0]*hv*su,
      fc[1] + right[1]*hw*sr + up[1]*hv*su,
      fc[2] + right[2]*hw*sr + up[2]*hv*su,
    ]);
    const pts = [];
    for (const c of fcorners) pts.push(gx, gy, gz, ...c);
    for (let i = 0; i < 4; i++) {
      const a = fcorners[i], b = fcorners[(i + 1) % 4];
      pts.push(...a, ...b);
    }
    drawLineSegments(pts, dimColor, viewProj);
  }
  gl.enable(gl.DEPTH_TEST);
}
