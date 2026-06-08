import { state } from '../core/state.js';
import { canvas, gl } from '../core/dom.js';
import { program, loc, gridBuffer, gizmoBuffer, gridCount, planeRenderObject } from './renderer.js';
import { modelCache } from './model-loader.js';
import { modelMatrix } from '../scene/objects.js';
import { selectionBounds } from '../scene/selection.js';
import { updateCameraFromKeyboard, currentViewProjection } from './camera.js';
import { drawGizmo } from './gizmo.js';
import { getPlacingZone } from '../interaction/canvas-controller.js';
import { drawRandomZones } from '../zones/zone-visualization.js';
import { drawPaths, drawVehiclePathMarkers } from '../paths/path-visualization.js';
import { drawCameraGhosts, drawSensorGhosts, drawCameraFovGhosts } from '../vehicles/sensor-visualization.js';
import { drawLidarRings } from '../vehicles/lidar-preview.js';

let lastFrameTime = performance.now();

function resize() {
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.floor(canvas.clientWidth * dpr));
  const height = Math.max(1, Math.floor(canvas.clientHeight * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
}

function drawGrid(viewProj) {
  gl.uniformMatrix4fv(loc.model, false, new Float32Array(mat4Identity()));
  gl.uniformMatrix4fv(loc.viewProj, false, new Float32Array(viewProj));
  gl.uniform3fv(loc.color, new Float32Array([0.28, 0.34, 0.38]));
  gl.uniform1i(loc.selected, 0);
  gl.bindBuffer(gl.ARRAY_BUFFER, gridBuffer);
  gl.enableVertexAttribArray(loc.position);
  gl.vertexAttribPointer(loc.position, 3, gl.FLOAT, false, 0, 0);
  gl.disableVertexAttribArray(loc.normal);
  gl.vertexAttrib3f(loc.normal, 0, 0, 1);
  gl.drawArrays(gl.LINES, 0, gridCount);
}

function drawObject(object, viewProj) {
  const model = modelCache.get(object.mesh);
  if (!model) {
    return;
  }
  gl.uniformMatrix4fv(loc.model, false, new Float32Array(modelMatrix(object)));
  gl.uniformMatrix4fv(loc.viewProj, false, new Float32Array(viewProj));
  gl.uniform3fv(loc.color, new Float32Array(object.color));
  gl.uniform1i(loc.selected, state.selectedIds.has(object.id) ? 1 : 0);
  gl.bindBuffer(gl.ARRAY_BUFFER, model.buffer);
  gl.enableVertexAttribArray(loc.position);
  gl.enableVertexAttribArray(loc.normal);
  gl.vertexAttribPointer(loc.position, 3, gl.FLOAT, false, 24, 0);
  gl.vertexAttribPointer(loc.normal, 3, gl.FLOAT, false, 24, 12);
  gl.drawArrays(gl.TRIANGLES, 0, model.count);
}

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

export function drawPoints(pts, color, size, viewProj) {
  gl.uniformMatrix4fv(loc.model, false, new Float32Array(mat4Identity()));
  gl.uniformMatrix4fv(loc.viewProj, false, new Float32Array(viewProj));
  gl.uniform3fv(loc.color, new Float32Array(color));
  gl.uniform1i(loc.selected, 0);
  gl.uniform1f(loc.pointSize, size);
  gl.bindBuffer(gl.ARRAY_BUFFER, gizmoBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, pts, gl.DYNAMIC_DRAW);
  gl.enableVertexAttribArray(loc.position);
  gl.vertexAttribPointer(loc.position, 3, gl.FLOAT, false, 0, 0);
  gl.disableVertexAttribArray(loc.normal);
  gl.vertexAttrib3f(loc.normal, 0, 0, 1);
  gl.drawArrays(gl.POINTS, 0, pts.length / 3);
  gl.uniform1f(loc.pointSize, 1.0);
}

function drawSelectionBounds(viewProj) {
  const bounds = selectionBounds();
  if (!bounds) {
    return;
  }
  const min = bounds.min;
  const max = bounds.max;
  const corners = [
    [min[0], min[1], min[2]],
    [max[0], min[1], min[2]],
    [max[0], max[1], min[2]],
    [min[0], max[1], min[2]],
    [min[0], min[1], max[2]],
    [max[0], min[1], max[2]],
    [max[0], max[1], max[2]],
    [min[0], max[1], max[2]],
  ];
  const edges = [
    [0, 1], [1, 2], [2, 3], [3, 0],
    [4, 5], [5, 6], [6, 7], [7, 4],
    [0, 4], [1, 5], [2, 6], [3, 7],
  ];
  const points = [];
  for (const edge of edges) {
    const a = corners[edge[0]];
    const b = corners[edge[1]];
    points.push(a[0], a[1], a[2], b[0], b[1], b[2]);
  }
  gl.disable(gl.DEPTH_TEST);
  drawLineSegments(points, [1.0, 0.86, 0.25], viewProj);
  gl.enable(gl.DEPTH_TEST);
}

export function render() {
  const now = performance.now();
  const dt = Math.min(0.05, (now - lastFrameTime) / 1000);
  lastFrameTime = now;
  updateCameraFromKeyboard(dt);
  resize();
  gl.viewport(0, 0, canvas.width, canvas.height);
  gl.clearColor(0.08, 0.1, 0.12, 1);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  gl.enable(gl.DEPTH_TEST);
  gl.useProgram(program);
  const viewProj = currentViewProjection();
  drawObject(planeRenderObject, viewProj);
  drawGrid(viewProj);
  drawRandomZones(viewProj, getPlacingZone());
  for (const object of state.objects) {
    if (object.vehicleRole === "camera_ghost" || object.vehicleRole === "sensor_ghost") continue;
    drawObject(object, viewProj);
  }
  drawPaths(viewProj);
  drawVehiclePathMarkers(viewProj);
  drawCameraGhosts(viewProj);
  drawSensorGhosts(viewProj);
  drawCameraFovGhosts(viewProj);
  drawLidarRings(viewProj);
  drawSelectionBounds(viewProj);
  drawGizmo(viewProj);
  requestAnimationFrame(render);
}
