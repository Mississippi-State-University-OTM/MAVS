// Pure math and scene utilities — no DOM or global state dependencies.
// Must be loaded before app.js.

// --- Vector math ---

function v3(x = 0, y = 0, z = 0) {
  return [x, y, z];
}

function add(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function sub(a, b) {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function mul(a, s) {
  return [a[0] * s, a[1] * s, a[2] * s];
}

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function norm(a) {
  const length = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0] / length, a[1] / length, a[2] / length];
}

function degToRad(value) {
  return value * Math.PI / 180;
}

// --- Matrix math (column-major 4x4) ---

function mat4Identity() {
  return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}

function mat4Multiply(a, b) {
  const out = new Array(16).fill(0);
  for (let row = 0; row < 4; row++) {
    for (let col = 0; col < 4; col++) {
      for (let i = 0; i < 4; i++) {
        out[col * 4 + row] += a[i * 4 + row] * b[col * 4 + i];
      }
    }
  }
  return out;
}

function mat4Perspective(fovy, aspect, near, far) {
  const f = 1 / Math.tan(fovy / 2);
  return [
    f / aspect, 0, 0, 0,
    0, f, 0, 0,
    0, 0, (far + near) / (near - far), -1,
    0, 0, (2 * far * near) / (near - far), 0,
  ];
}

function mat4LookAt(eye, target, up) {
  const z = norm(sub(eye, target));
  const x = norm(cross(up, z));
  const y = cross(z, x);
  return [
    x[0], y[0], z[0], 0,
    x[1], y[1], z[1], 0,
    x[2], y[2], z[2], 0,
    -dot(x, eye), -dot(y, eye), -dot(z, eye), 1,
  ];
}

function mat4Translate(v) {
  const m = mat4Identity();
  m[12] = v[0];
  m[13] = v[1];
  m[14] = v[2];
  return m;
}

function mat4Scale(v) {
  const m = mat4Identity();
  m[0] = v[0];
  m[5] = v[1];
  m[10] = v[2];
  return m;
}

function mat4RotateX(r) {
  const c = Math.cos(r);
  const s = Math.sin(r);
  return [1, 0, 0, 0, 0, c, s, 0, 0, -s, c, 0, 0, 0, 0, 1];
}

function mat4RotateY(r) {
  const c = Math.cos(r);
  const s = Math.sin(r);
  return [c, 0, -s, 0, 0, 1, 0, 0, s, 0, c, 0, 0, 0, 0, 1];
}

function mat4RotateZ(r) {
  const c = Math.cos(r);
  const s = Math.sin(r);
  return [c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}

// --- Transform and geometry helpers ---

function rotatePointAroundAxis(point, center, axis, degrees) {
  const r = degToRad(degrees);
  const c = Math.cos(r);
  const s = Math.sin(r);
  const p = sub(point, center);
  let rotated = p;
  if (axis === "x") {
    rotated = [p[0], p[1] * c - p[2] * s, p[1] * s + p[2] * c];
  } else if (axis === "y") {
    rotated = [p[0] * c + p[2] * s, p[1], -p[0] * s + p[2] * c];
  } else {
    rotated = [p[0] * c - p[1] * s, p[0] * s + p[1] * c, p[2]];
  }
  return add(center, rotated);
}

function transformPoint(m, p) {
  const x = p[0];
  const y = p[1];
  const z = p[2];
  const w = m[3] * x + m[7] * y + m[11] * z + m[15];
  return [
    (m[0] * x + m[4] * y + m[8] * z + m[12]) / w,
    (m[1] * x + m[5] * y + m[9] * z + m[13]) / w,
    (m[2] * x + m[6] * y + m[10] * z + m[14]) / w,
  ];
}

function distanceToSegment(point, a, b) {
  const ab = [b[0] - a[0], b[1] - a[1]];
  const ap = [point[0] - a[0], point[1] - a[1]];
  const len2 = ab[0] * ab[0] + ab[1] * ab[1] || 1;
  const t = Math.max(0, Math.min(1, (ap[0] * ab[0] + ap[1] * ab[1]) / len2));
  const nearest = [a[0] + ab[0] * t, a[1] + ab[1] * t];
  return Math.hypot(point[0] - nearest[0], point[1] - nearest[1]);
}

function zOnTriangleAtXY(point, a, b, c) {
  const x = point[0], y = point[1];
  const x1 = a[0], y1 = a[1];
  const x2 = b[0], y2 = b[1];
  const x3 = c[0], y3 = c[1];
  const denom = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3);
  if (Math.abs(denom) < 0.000001) return null;
  const w1 = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / denom;
  const w2 = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / denom;
  const w3 = 1 - w1 - w2;
  if (w1 < -0.0001 || w2 < -0.0001 || w3 < -0.0001) return null;
  return w1 * a[2] + w2 * b[2] + w3 * c[2];
}

function rayBoundsDistance(ray, bounds) {
  let near = 0;
  let far = Infinity;
  for (let axis = 0; axis < 3; axis++) {
    if (Math.abs(ray.dir[axis]) < 0.000001) {
      if (ray.origin[axis] < bounds.min[axis] || ray.origin[axis] > bounds.max[axis]) return null;
      continue;
    }
    let a = (bounds.min[axis] - ray.origin[axis]) / ray.dir[axis];
    let b = (bounds.max[axis] - ray.origin[axis]) / ray.dir[axis];
    if (a > b) [a, b] = [b, a];
    near = Math.max(near, a);
    far = Math.min(far, b);
    if (near > far) return null;
  }
  return near;
}

function eulerToQuat(yawDeg, pitchDeg, rollDeg) {
  const y = yawDeg * Math.PI / 360;
  const p = pitchDeg * Math.PI / 360;
  const r = rollDeg * Math.PI / 360;
  return [
    Math.cos(r)*Math.cos(p)*Math.cos(y) + Math.sin(r)*Math.sin(p)*Math.sin(y),
    Math.sin(r)*Math.cos(p)*Math.cos(y) - Math.cos(r)*Math.sin(p)*Math.sin(y),
    Math.cos(r)*Math.sin(p)*Math.cos(y) + Math.sin(r)*Math.cos(p)*Math.sin(y),
    Math.cos(r)*Math.cos(p)*Math.sin(y) - Math.sin(r)*Math.sin(p)*Math.cos(y),
  ];
}

// --- Scene-level helpers ---

function controllerPath(initialPosition, waypoints, spacing = 1.0) {
  const points = [[Number(initialPosition[0]), Number(initialPosition[1])]];
  for (const waypoint of waypoints) {
    const point = [Number(waypoint[0]), Number(waypoint[1])];
    const start = points[points.length - 1];
    const distance = Math.hypot(point[0] - start[0], point[1] - start[1]);
    if (distance < 0.001) continue;
    const steps = Math.max(1, Math.ceil(distance / spacing));
    for (let step = 1; step <= steps; step++) {
      const fraction = step / steps;
      points.push([
        start[0] + (point[0] - start[0]) * fraction,
        start[1] + (point[1] - start[1]) * fraction,
      ]);
    }
  }
  return points;
}

function sceneMeshPath(path) {
  const prefix = "scenes/meshes/";
  return path.startsWith(prefix) ? path.slice(prefix.length) : path;
}

function editorMeshPath(path) {
  if (!path) return "";
  if (path.startsWith("scenes/meshes/")) return path;
  if (path.match(/^[A-Za-z]:[\\/]/) || path.startsWith("/") || path.startsWith("\\")) {
    return path.replaceAll("\\", "/");
  }
  return `scenes/meshes/${path}`;
}
