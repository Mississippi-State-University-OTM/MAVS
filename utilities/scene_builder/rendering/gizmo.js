import { state } from '../core/state.js';
import { hasSelection, selectionCenter, selectionBounds } from '../scene/selection.js';
import { projectToScreen } from './camera.js';
import { gl } from '../core/dom.js';
import { loc, gizmoBuffer } from './renderer.js';

export const gizmoAxes = {
  x: { vector: [1, 0, 0], color: [0.95, 0.18, 0.18] },
  y: { vector: [0, 1, 0], color: [0.2, 0.86, 0.35] },
  z: { vector: [0, 0, 1], color: [0.28, 0.55, 1.0] },
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

export function axisRingPoint(origin, axis, angle, radius) {
  const c = Math.cos(angle) * radius;
  const s = Math.sin(angle) * radius;
  if (axis === "x") {
    return [origin[0], origin[1] + c, origin[2] + s];
  }
  if (axis === "y") {
    return [origin[0] + c, origin[1], origin[2] + s];
  }
  return [origin[0] + c, origin[1] + s, origin[2]];
}

export function gizmoLength() {
  const bounds = selectionBounds();
  if (!bounds) {
    return 3;
  }
  const size = Math.hypot(bounds.max[0] - bounds.min[0], bounds.max[1] - bounds.min[1], bounds.max[2] - bounds.min[2]);
  return Math.max(3, Math.min(12, size * 0.35 + 3));
}

export function pickGizmoAxis(clientX, clientY) {
  if (!hasSelection()) {
    return null;
  }
  const cursor = [clientX, clientY];
  const origin = selectionCenter();
  const length = gizmoLength();
  let best = null;
  let bestDistance = state.ui.transformMode === "rotate" ? 10 : 14;
  for (const [axis, config] of Object.entries(gizmoAxes)) {
    if (state.ui.transformMode === "rotate") {
      const radius = length * 0.78;
      const segments = 48;
      for (let i = 0; i < segments; i++) {
        const a = projectToScreen(axisRingPoint(origin, axis, (i / segments) * Math.PI * 2, radius));
        const b = projectToScreen(axisRingPoint(origin, axis, ((i + 1) / segments) * Math.PI * 2, radius));
        const distance = distanceToSegment(cursor, a, b);
        if (distance < bestDistance) {
          best = axis;
          bestDistance = distance;
        }
      }
    } else {
      const start = projectToScreen(origin);
      const end = projectToScreen(add(origin, mul(config.vector, length)));
      const distance = distanceToSegment(cursor, start, end);
      if (distance < bestDistance) {
        best = axis;
        bestDistance = distance;
      }
    }
  }
  return best;
}

export function drawGizmo(viewProj) {
  if (!hasSelection()) {
    return;
  }
  const origin = selectionCenter();
  const length = gizmoLength();
  const head = Math.min(1.2, length * 0.22);
  gl.disable(gl.DEPTH_TEST);
  for (const [axis, config] of Object.entries(gizmoAxes)) {
    const direction = config.vector;
    const end = add(origin, mul(direction, length));
    let points = [];
    if (state.ui.transformMode === "rotate") {
      const radius = length * 0.78;
      const segments = 64;
      for (let i = 0; i < segments; i++) {
        const a = axisRingPoint(origin, axis, (i / segments) * Math.PI * 2, radius);
        const b = axisRingPoint(origin, axis, ((i + 1) / segments) * Math.PI * 2, radius);
        points.push(a[0], a[1], a[2], b[0], b[1], b[2]);
      }
    } else {
      let sideA = [0, 0, 0];
      let sideB = [0, 0, 0];
      if (axis === "x") {
        sideA = [0, head * 0.35, 0];
        sideB = [0, 0, head * 0.35];
      } else if (axis === "y") {
        sideA = [head * 0.35, 0, 0];
        sideB = [0, 0, head * 0.35];
      } else {
        sideA = [head * 0.35, 0, 0];
        sideB = [0, head * 0.35, 0];
      }
      const back = add(origin, mul(direction, length - head));
      points = [
        origin[0], origin[1], origin[2],
        end[0], end[1], end[2],
      ];
      if (state.ui.transformMode === "translate") {
        points.push(
          end[0], end[1], end[2],
          back[0] + sideA[0], back[1] + sideA[1], back[2] + sideA[2],
          end[0], end[1], end[2],
          back[0] - sideA[0], back[1] - sideA[1], back[2] - sideA[2],
          end[0], end[1], end[2],
          back[0] + sideB[0], back[1] + sideB[1], back[2] + sideB[2],
          end[0], end[1], end[2],
          back[0] - sideB[0], back[1] - sideB[1], back[2] - sideB[2],
        );
      } else {
        points.push(
          end[0] - sideA[0] - sideB[0], end[1] - sideA[1] - sideB[1], end[2] - sideA[2] - sideB[2],
          end[0] + sideA[0] - sideB[0], end[1] + sideA[1] - sideB[1], end[2] + sideA[2] - sideB[2],
          end[0] + sideA[0] - sideB[0], end[1] + sideA[1] - sideB[1], end[2] + sideA[2] - sideB[2],
          end[0] + sideA[0] + sideB[0], end[1] + sideA[1] + sideB[1], end[2] + sideA[2] + sideB[2],
          end[0] + sideA[0] + sideB[0], end[1] + sideA[1] + sideB[1], end[2] + sideA[2] + sideB[2],
          end[0] - sideA[0] + sideB[0], end[1] - sideA[1] + sideB[1], end[2] - sideA[2] + sideB[2],
          end[0] - sideA[0] + sideB[0], end[1] - sideA[1] + sideB[1], end[2] - sideA[2] + sideB[2],
          end[0] - sideA[0] - sideB[0], end[1] - sideA[1] - sideB[1], end[2] - sideA[2] - sideB[2],
        );
      }
    }
    drawLineSegments(points, config.color, viewProj);
  }
  gl.enable(gl.DEPTH_TEST);
}
