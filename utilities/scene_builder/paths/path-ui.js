// Waypoint-path inspector and sidebar rendering.
import { state } from '../core/state.js';
import {
  pathList, pathNameInput, pathWaypointCount, pathColorBar,
  pathVehicleList, pathAddVehicle, togglePathPlacementBtn,
} from '../core/dom.js';
import { selectedPath } from './paths.js';
import {
  controllerModeFor, setObjectPath, vehicleRepresentatives,
} from '../vehicles/vehicles.js';
import { listen } from '../core/events.js';

const eyeOpenSvg = `<svg viewBox="0 0 16 16" width="13" height="13" fill="none" aria-hidden="true"><path d="M8 3C4.5 3 1.5 6 1.5 8s3 5 6.5 5 6.5-3 6.5-5-3-5-6.5-5z" stroke="currentColor" stroke-width="1.3"/><circle cx="8" cy="8" r="2.2" fill="currentColor"/></svg>`;
const eyeClosedSvg = `<svg viewBox="0 0 16 16" width="13" height="13" fill="none" aria-hidden="true"><path d="M8 3C4.5 3 1.5 6 1.5 8s3 5 6.5 5 6.5-3 6.5-5-3-5-6.5-5z" stroke="currentColor" stroke-width="1.3" opacity="0.3"/><circle cx="8" cy="8" r="2.2" fill="currentColor" opacity="0.3"/><line x1="2.5" y1="2.5" x2="13.5" y2="13.5" stroke="currentColor" stroke-width="1.3"/></svg>`;

let syncInspector;

export function initPathUI(ctx) {
  ({ syncInspector } = ctx);
}

export function syncPathInspector() {
  const path = selectedPath();
  if (!path) return;
  pathNameInput.value = path.name;
  pathWaypointCount.textContent = `${path.waypoints.length} waypoints`;
  const c = path.color;
  pathColorBar.style.background =
    `rgb(${Math.round(c[0] * 255)},${Math.round(c[1] * 255)},${Math.round(c[2] * 255)})`;
  syncPathVehicleList();
}

export function syncPathVehicleList() {
  const path = selectedPath();
  pathVehicleList.innerHTML = "";
  if (!path) {
    pathAddVehicle.innerHTML = "";
    return;
  }

  const assigned = vehicleRepresentatives().filter((obj) =>
    controllerModeFor(obj) === "path" && obj.pathId === path.id);
  for (const obj of assigned) {
    const row = document.createElement("div");
    row.className = "path-vehicle-item";
    const name = document.createElement("span");
    name.className = "path-vehicle-name";
    name.textContent = obj.mesh.split("/").pop() || obj.mesh;
    name.title = obj.mesh;
    const removeBtn = document.createElement("button");
    removeBtn.className = "path-vehicle-remove";
    removeBtn.textContent = "\u00d7";
    removeBtn.title = "Remove from path";
    listen(removeBtn, "click", () => {
      setObjectPath(obj, null);
      syncInspector();
    });
    row.appendChild(name);
    row.appendChild(removeBtn);
    pathVehicleList.appendChild(row);
  }
  if (!assigned.length) {
    const empty = document.createElement("div");
    empty.className = "muted";
    empty.textContent = "No vehicles assigned";
    pathVehicleList.appendChild(empty);
  }

  pathAddVehicle.innerHTML = "Add vehicle\u2026";
  for (const obj of vehicleRepresentatives()) {
    if (obj.pathId === path.id) continue;
    const opt = document.createElement("option");
    opt.value = String(obj.id);
    opt.textContent = obj.mesh.split("/").pop() || obj.mesh;
    opt.title = obj.mesh;
    pathAddVehicle.appendChild(opt);
  }
}

export function renderPathList() {
  pathList.innerHTML = "";
  for (const path of state.paths) {
    const item = document.createElement("div");
    item.className = "path-item" + (path.id === state.selectedPathId ? " active" : "");
    const dot = document.createElement("span");
    dot.className = "path-color-dot";
    const c = path.color;
    dot.style.background = `rgb(${Math.round(c[0] * 255)},${Math.round(c[1] * 255)},${Math.round(c[2] * 255)})`;
    const nameSpan = document.createElement("span");
    nameSpan.className = "path-item-name";
    nameSpan.textContent = path.name;
    const visBtn = document.createElement("button");
    visBtn.className = "path-vis-btn" + (path.visible ? "" : " path-hidden");
    visBtn.title = path.visible ? "Hide path" : "Show path";
    visBtn.innerHTML = path.visible ? eyeOpenSvg : eyeClosedSvg;
    listen(visBtn, "click", (event) => {
      event.stopPropagation();
      path.visible = !path.visible;
      renderPathList();
    });
    item.appendChild(dot);
    item.appendChild(nameSpan);
    item.appendChild(visBtn);
    listen(item, "click", () => {
      state.selectedPathId = path.id;
      state.selectedWaypointIndex = null;
      state.selectedIds = new Set();
      state.selectedZoneId = null;
      state.ui.pathPlacementMode = false;
      togglePathPlacementBtn.classList.toggle("active", false);
      syncInspector();
      renderPathList();
    });
    pathList.appendChild(item);
  }
}
