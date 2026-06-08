import { state } from '../core/state.js';
import { planeRenderObject } from '../rendering/renderer.js';
import { modelCache } from '../rendering/model-loader.js';
import { modelMatrix, objectBounds } from '../scene/objects.js';
import { selectedObjects } from '../scene/selection.js';
import { vehicleGroupObjects } from '../vehicles/vehicles.js';
import { syncInspector } from '../ui/inspector.js';

let _setStatus = () => {};

export function initGroundSnap({ setStatus }) {
  _setStatus = setStatus;
}

function surfaceHeightAtXY(point, ignoredIds = new Set()) {
  let best = null;
  const ignored = ignoredIds instanceof Set ? ignoredIds : new Set([ignoredIds]);
  const candidates = [planeRenderObject, ...state.objects.filter((object) => !ignored.has(object.id))];
  for (const object of candidates) {
    const model = modelCache.get(object.mesh);
    if (!model) continue;
    const m = modelMatrix(object);
    for (const triangle of model.localTriangles) {
      const a = transformPoint(m, triangle[0]);
      const b = transformPoint(m, triangle[1]);
      const c = transformPoint(m, triangle[2]);
      const z = zOnTriangleAtXY(point, a, b, c);
      if (z === null) continue;
      if (best === null || z > best) best = z;
    }
  }
  return best;
}

function bottomSamplePoints(bounds) {
  const center = mul(add(bounds.min, bounds.max), 0.5);
  return [
    [center[0], center[1]],
    [bounds.min[0], bounds.min[1]],
    [bounds.min[0], bounds.max[1]],
    [bounds.max[0], bounds.min[1]],
    [bounds.max[0], bounds.max[1]],
  ];
}

export function snapSelectedToGround() {
  const selection = selectedObjects();
  if (!selection.length) return;
  const ignored = new Set(selection.map((object) => object.id));
  let snapped = 0;
  const units = [];
  const seenGroups = new Set();
  for (const object of selection) {
    if (object.vehicleGroupId == null) {
      units.push([object]);
    } else if (!seenGroups.has(object.vehicleGroupId)) {
      seenGroups.add(object.vehicleGroupId);
      units.push(vehicleGroupObjects(object));
    }
  }
  for (const unit of units) {
    const contactObjects = unit.filter((object) => object.vehicleRole === "tire");
    const contacts = contactObjects.length ? contactObjects : unit;
    const contactBounds = contacts.map(objectBounds).filter(Boolean);
    if (!contactBounds.length) continue;
    const bounds = {
      min: [
        Math.min(...contactBounds.map((item) => item.min[0])),
        Math.min(...contactBounds.map((item) => item.min[1])),
        Math.min(...contactBounds.map((item) => item.min[2])),
      ],
      max: [
        Math.max(...contactBounds.map((item) => item.max[0])),
        Math.max(...contactBounds.map((item) => item.max[1])),
        Math.max(...contactBounds.map((item) => item.max[2])),
      ],
    };
    let targetZ = null;
    for (const point of bottomSamplePoints(bounds)) {
      const surfaceZ = surfaceHeightAtXY(point, ignored);
      if (surfaceZ !== null && (targetZ === null || surfaceZ > targetZ)) {
        targetZ = surfaceZ;
      }
    }
    if (targetZ === null) continue;
    const deltaZ = targetZ - bounds.min[2];
    for (const object of unit) {
      object.position[2] += deltaZ;
    }
    snapped += unit.length;
  }
  if (!snapped) {
    _setStatus("No surface found below selection", true);
    return;
  }
  syncInspector();
  _setStatus(`Snapped ${snapped} object${snapped === 1 ? "" : "s"} to ground`);
}
