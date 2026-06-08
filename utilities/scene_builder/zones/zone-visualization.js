import { state } from '../core/state.js';
import { gl } from '../core/dom.js';
import { loc, gizmoBuffer } from '../rendering/renderer.js';
import { colorForPath } from '../scene/objects.js';

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

function zoneLinePoints(minX, maxX, minY, maxY, z) {
  return [
    minX, minY, z, maxX, minY, z,
    maxX, minY, z, maxX, maxY, z,
    maxX, maxY, z, minX, maxY, z,
    minX, maxY, z, minX, minY, z,
    minX, minY, z, maxX, maxY, z,
    maxX, minY, z, minX, maxY, z,
  ];
}

export function drawRandomZones(viewProj, placingZone) {
  gl.disable(gl.DEPTH_TEST);
  for (const zone of state.randomZones) {
    const color = zone.id === state.selectedZoneId ? [1.0, 0.86, 0.25] : colorForPath(zone.mesh || "random-zone");
    drawLineSegments(zoneLinePoints(zone.minX, zone.maxX, zone.minY, zone.maxY, zone.offsetZ + 0.08), color, viewProj);
  }
  if (placingZone) {
    const minX = Math.min(placingZone.start[0], placingZone.current[0]);
    const maxX = Math.max(placingZone.start[0], placingZone.current[0]);
    const minY = Math.min(placingZone.start[1], placingZone.current[1]);
    const maxY = Math.max(placingZone.start[1], placingZone.current[1]);
    drawLineSegments(zoneLinePoints(minX, maxX, minY, maxY, 0.1), [0.45, 0.82, 1.0], viewProj);
  }
  gl.enable(gl.DEPTH_TEST);
}
