// Random-zone inspector form synchronization.
import { zoneInspector } from '../core/dom.js';
import { selectedZone, zoneArea, zoneCount } from './zones.js';

let syncInspector;

export function initZoneUI(ctx) {
  ({ syncInspector } = ctx);
}

export function syncZoneInspector(zone = selectedZone()) {
  if (!zone) return;
  zoneInspector.mesh.value = zone.mesh;
  zoneInspector.placement.value = zone.placement;
  zoneInspector.density.value = zone.density;
  zoneInspector.number.value = zone.number;
  zoneInspector.density.disabled = zone.placement !== "density";
  zoneInspector.number.disabled = zone.placement !== "count";
  zoneInspector.spacing.value = zone.minimumSpacing;
  zoneInspector.offsetZ.value = zone.offsetZ;
  zoneInspector.scaleMin.value = zone.scaleMin;
  zoneInspector.scaleMax.value = zone.scaleMax;
  zoneInspector.minX.value = zone.minX.toFixed(2);
  zoneInspector.maxX.value = zone.maxX.toFixed(2);
  zoneInspector.minY.value = zone.minY.toFixed(2);
  zoneInspector.maxY.value = zone.maxY.toFixed(2);
  zoneInspector.summary.textContent = `${zoneArea(zone).toFixed(1)} m^2, exports ${zoneCount(zone)} random objects`;
}

export function applyZoneInspector() {
  const zone = selectedZone();
  if (!zone) return;
  zone.mesh = zoneInspector.mesh.value.trim();
  zone.placement = zoneInspector.placement.value;
  zone.density = Math.max(0, Number(zoneInspector.density.value) || 0);
  zone.number = Math.max(0, Math.round(Number(zoneInspector.number.value) || 0));
  zone.minimumSpacing = Math.max(0, Number(zoneInspector.spacing.value) || 0);
  zone.offsetZ = Number(zoneInspector.offsetZ.value) || 0;
  zone.scaleMin = Math.max(0.01, Number(zoneInspector.scaleMin.value) || 0.01);
  zone.scaleMax = Math.max(zone.scaleMin, Number(zoneInspector.scaleMax.value) || zone.scaleMin);
  const x1 = Number(zoneInspector.minX.value);
  const x2 = Number(zoneInspector.maxX.value);
  const y1 = Number(zoneInspector.minY.value);
  const y2 = Number(zoneInspector.maxY.value);
  if ([x1, x2, y1, y2].every(Number.isFinite)) {
    zone.minX = Math.min(x1, x2);
    zone.maxX = Math.max(x1, x2);
    zone.minY = Math.min(y1, y2);
    zone.maxY = Math.max(y1, y2);
  }
  syncInspector();
}
