import { state } from '../core/state.js';
import { gl } from '../core/dom.js';
import { loc, gizmoBuffer } from '../rendering/renderer.js';
import { vehicleRepresentatives, controllerModeFor, vehicleParamsFor } from '../vehicles/vehicles.js';
import { vehicleInitialPosition } from '../scene/scene-io.js';
import { objectBounds } from '../scene/objects.js';

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

function drawControllerGhostPoints(path, viewProj) {
  const placed = path.waypoints;
  const ghostLinePoints = [];
  for (const chassis of vehicleRepresentatives()) {
    if (controllerModeFor(chassis) !== "path" || chassis.pathId !== path.id) continue;
    const def = state.vehicleDefs.find((candidate) => candidate.name === chassis.vehicleDefName);
    if (!def) continue;
    const generated = controllerPath(
      vehicleInitialPosition(chassis, def),
      placed,
      vehicleParamsFor(chassis).pathSpacing,
    );
    for (const point of generated) {
      const isPlaced = placed.some((waypoint) =>
        Math.hypot(point[0] - waypoint[0], point[1] - waypoint[1]) < 0.001);
      if (isPlaced) continue;
      const r = 0.16;
      ghostLinePoints.push(
        point[0] - r, point[1], 0.22, point[0] + r, point[1], 0.22,
        point[0], point[1] - r, 0.22, point[0], point[1] + r, 0.22,
      );
    }
  }
  if (ghostLinePoints.length) {
    const ghostColor = path.color.map((value) => value * 0.45 + 0.22);
    drawLineSegments(ghostLinePoints, ghostColor, viewProj);
  }
}

export function drawPaths(viewProj) {
  if (!state.paths.length) return;
  gl.disable(gl.DEPTH_TEST);
  for (const path of state.paths) {
    if (!path.visible || !path.waypoints.length) continue;
    const isSelected = path.id === state.selectedPathId;
    const color = path.color;
    if (path.waypoints.length >= 2) {
      const linePoints = [];
      for (let i = 0; i < path.waypoints.length - 1; i++) {
        const a = path.waypoints[i];
        const b = path.waypoints[i + 1];
        linePoints.push(a[0], a[1], 0.18, b[0], b[1], 0.18);
      }
      drawLineSegments(linePoints, color, viewProj);
    }
    drawControllerGhostPoints(path, viewProj);
    const r = 0.6;
    for (let i = 0; i < path.waypoints.length; i++) {
      const wp = path.waypoints[i];
      const isHighlighted = isSelected && i === state.selectedWaypointIndex;
      const nc = isHighlighted ? [1.0, 1.0, 0.3] : (isSelected ? [1, 1, 1] : color);
      const nr = isHighlighted ? 0.9 : r;
      drawLineSegments([
        wp[0] - nr, wp[1], 0.18, wp[0] + nr, wp[1], 0.18,
        wp[0], wp[1] - nr, 0.18, wp[0], wp[1] + nr, 0.18,
      ], nc, viewProj);
    }
  }
  gl.enable(gl.DEPTH_TEST);
}

export function drawVehiclePathMarkers(viewProj) {
  const assigned = vehicleRepresentatives().filter((obj) =>
    controllerModeFor(obj) === "path" && obj.pathId != null);
  if (!assigned.length) return;
  gl.disable(gl.DEPTH_TEST);
  for (const obj of assigned) {
    const path = state.paths.find((p) => p.id === obj.pathId);
    if (!path) continue;
    const bounds = objectBounds(obj);
    const topZ = bounds ? bounds.max[2] : obj.position[2];
    const cx = obj.position[0];
    const cy = obj.position[1];
    const cz = topZ + 2.4;
    const w = 0.72;
    const h = 1.05;
    const tp = [cx, cy, cz + h];
    const bt = [cx, cy, cz - h];
    const ex = [cx + w, cy, cz];
    const wx = [cx - w, cy, cz];
    const ey = [cx, cy + w, cz];
    const wy = [cx, cy - w, cz];
    drawLineSegments([
      ...tp, ...ex,  ...tp, ...wx,  ...tp, ...ey,  ...tp, ...wy,
      ...bt, ...ex,  ...bt, ...wx,  ...bt, ...ey,  ...bt, ...wy,
      ...ex, ...ey,  ...ey, ...wx,  ...wx, ...wy,  ...wy, ...ex,
    ], path.color, viewProj);
    drawLineSegments([cx, cy, topZ, cx, cy, cz - h], path.color, viewProj);
  }
  gl.enable(gl.DEPTH_TEST);
}
