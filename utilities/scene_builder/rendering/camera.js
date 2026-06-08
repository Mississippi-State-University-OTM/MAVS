// Camera movement, ray casting, and projection helpers.
import { state } from '../core/state.js';
import { canvas } from '../core/dom.js';

let pressedKeys;

export function initCamera(ctx) {
  ({ pressedKeys } = ctx);
}

export function cameraPosition() {
  const cp = Math.cos(state.camera.pitch);
  return [
    state.camera.target[0] + state.camera.distance * cp * Math.cos(state.camera.yaw),
    state.camera.target[1] + state.camera.distance * cp * Math.sin(state.camera.yaw),
    state.camera.target[2] + state.camera.distance * Math.sin(state.camera.pitch),
  ];
}

export function cameraViewAxes() {
  const eye = cameraPosition();
  const forward = norm(sub(state.camera.target, eye));
  const right = norm(cross(forward, [0, 0, 1]));
  return { forward, right };
}

export function updateCameraFromKeyboard(dt) {
  if (!state.ui.orbiting) {
    return;
  }
  let movement = [0, 0, 0];
  const { forward, right } = cameraViewAxes();
  if (pressedKeys.has("w")) {
    movement = add(movement, forward);
  }
  if (pressedKeys.has("s")) {
    movement = sub(movement, forward);
  }
  if (pressedKeys.has("d")) {
    movement = add(movement, right);
  }
  if (pressedKeys.has("a")) {
    movement = sub(movement, right);
  }
  if (Math.hypot(movement[0], movement[1], movement[2]) === 0) {
    return;
  }
  const speed = Math.max(8, state.camera.distance * 0.55);
  state.camera.target = add(state.camera.target, mul(norm(movement), speed * dt));
}

export function screenRay(clientX, clientY) {
  const rect = canvas.getBoundingClientRect();
  const x = ((clientX - rect.left) / rect.width) * 2 - 1;
  const y = 1 - ((clientY - rect.top) / rect.height) * 2;
  const eye = cameraPosition();
  const forward = norm(sub(state.camera.target, eye));
  const right = norm(cross(forward, [0, 0, 1]));
  const up = norm(cross(right, forward));
  const tan = Math.tan(50 * Math.PI / 180 / 2);
  const aspect = canvas.width / Math.max(1, canvas.height);
  return {
    origin: eye,
    dir: norm(add(add(forward, mul(right, x * aspect * tan)), mul(up, y * tan))),
  };
}

export function groundPoint(clientX, clientY) {
  const ray = screenRay(clientX, clientY);
  if (Math.abs(ray.dir[2]) < 0.0001) {
    return [0, 0, 0];
  }
  const t = -ray.origin[2] / ray.dir[2];
  return add(ray.origin, mul(ray.dir, Math.max(0, t)));
}

export function currentViewProjection() {
  const eye = cameraPosition();
  const view = mat4LookAt(eye, state.camera.target, [0, 0, 1]);
  const proj = mat4Perspective(50 * Math.PI / 180, canvas.width / Math.max(1, canvas.height), 0.1, 3000);
  return mat4Multiply(proj, view);
}

export function projectToScreen(point) {
  const clip = transformPoint(currentViewProjection(), point);
  const rect = canvas.getBoundingClientRect();
  return [
    rect.left + (clip[0] * 0.5 + 0.5) * rect.width,
    rect.top + (0.5 - clip[1] * 0.5) * rect.height,
  ];
}
