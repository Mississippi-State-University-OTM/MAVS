// Waypoint-path state, placement mode, and viewport picking.
import { state } from '../core/state.js';
import { togglePathPlacementBtn } from '../core/dom.js';

export const pathColorPalette = [
  [1.0, 0.45, 0.12],
  [0.2, 0.88, 0.3],
  [1.0, 0.88, 0.15],
  [0.9, 0.2, 0.85],
  [0.15, 0.85, 0.9],
  [1.0, 0.5, 0.75],
  [0.5, 0.35, 1.0],
  [1.0, 0.72, 0.2],
];

let projectToScreen;
let deactivateZonePlacement;
let syncInspector;
let syncPathInspector;
let renderPathList;
let setStatus;

export function initPaths(ctx) {
  ({
    projectToScreen,
    deactivateZonePlacement,
    syncInspector,
    syncPathInspector,
    renderPathList,
    setStatus,
  } = ctx);
}

export function selectedPath() {
  return state.paths.find((path) => path.id === state.selectedPathId) || null;
}

export function clearPathSelection() {
  state.selectedPathId = null;
  state.selectedWaypointIndex = null;
  state.ui.pathPlacementMode = false;
  togglePathPlacementBtn.classList.toggle("active", false);
}

export function setPathPlacementMode(active) {
  state.ui.pathPlacementMode = active;
  togglePathPlacementBtn.classList.toggle("active", active);
  if (active) {
    deactivateZonePlacement();
    state.selectedIds = new Set();
    state.selectedZoneId = null;
    syncInspector();
    setStatus("Click on the ground to add waypoints to the path");
  }
}

export function addPath() {
  const path = {
    id: state.nextPathId++,
    name: `path_${state.nextPathId - 1}.json`,
    color: pathColorPalette[state.paths.length % pathColorPalette.length],
    waypoints: [],
    visible: true,
  };
  state.paths.push(path);
  state.selectedPathId = path.id;
  state.selectedWaypointIndex = null;
  state.selectedIds = new Set();
  state.selectedZoneId = null;
  setPathPlacementMode(true);
  renderPathList();
  setStatus(`Added path "${path.name}"`);
}

export function addWaypointToSelectedPath(x, y) {
  const path = selectedPath();
  if (!path) return;
  path.waypoints.push([parseFloat(x.toFixed(3)), parseFloat(y.toFixed(3))]);
  syncPathInspector();
}

export function pickWaypointNode(clientX, clientY) {
  const path = selectedPath();
  if (!path) return null;
  const cursor = [clientX, clientY];
  for (let i = 0; i < path.waypoints.length; i++) {
    const wp = path.waypoints[i];
    const screen = projectToScreen([wp[0], wp[1], 0.18]);
    if (Math.hypot(cursor[0] - screen[0], cursor[1] - screen[1]) < 12) {
      return i;
    }
  }
  return null;
}
