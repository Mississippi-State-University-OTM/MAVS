// Random-zone selection, placement, creation, and counting.
import { state } from '../core/state.js';
import { addRandomZoneButton } from '../core/dom.js';
import { selectZone } from '../scene/selection.js';

let clearPathSelection;
let syncInspector;
let setStatus;

export function initZones(ctx) {
  ({ clearPathSelection, syncInspector, setStatus } = ctx);
}

export function selectedZone() {
  return state.randomZones.find((zone) => zone.id === state.selectedZoneId) || null;
}

export function setZonePlacementMode(active) {
  state.ui.zonePlacementMode = active;
  addRandomZoneButton.classList.toggle("active", active);
  if (active) {
    clearPathSelection();
    state.selectedIds = new Set();
    state.selectedZoneId = null;
    syncInspector();
    setStatus("Drag on the ground to create a random zone");
  }
}

export function defaultZoneMesh() {
  return state.assets.find((asset) => asset.path.includes("/vegetation/grass/"))?.path
    || state.assets.find((asset) => asset.path.includes("/vegetation/"))?.path
    || "scenes/meshes/vegetation/grass/grass_clump_1.obj";
}

export function finishZonePlacement(placingZone) {
  if (!placingZone) return null;
  const minX = Math.min(placingZone.start[0], placingZone.current[0]);
  const maxX = Math.max(placingZone.start[0], placingZone.current[0]);
  const minY = Math.min(placingZone.start[1], placingZone.current[1]);
  const maxY = Math.max(placingZone.start[1], placingZone.current[1]);
  if (maxX - minX >= 0.1 && maxY - minY >= 0.1) {
    const zone = {
      id: state.nextZoneId++,
      mesh: defaultZoneMesh(),
      placement: "density",
      density: 0.25,
      number: 25,
      minimumSpacing: 0,
      offsetZ: 0,
      scaleMin: 0.85,
      scaleMax: 1.15,
      minX,
      maxX,
      minY,
      maxY,
    };
    state.randomZones.push(zone);
    selectZone(zone.id);
    setStatus(`Added random zone (${zoneCount(zone)} objects)`);
  }
  setZonePlacementMode(false);
  return null;
}

export function zoneArea(zone) {
  return Math.max(0, zone.maxX - zone.minX) * Math.max(0, zone.maxY - zone.minY);
}

export function zoneCount(zone) {
  return zone.placement === "density"
    ? Math.max(0, Math.round(zoneArea(zone) * zone.density))
    : Math.max(0, Math.round(zone.number));
}
