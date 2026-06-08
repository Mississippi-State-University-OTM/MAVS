import { state } from '../core/state.js';
import {
  inspector,
  modeButtons,
  vehicleSpeed,
  vehiclePathSpacing,
  vehicleSteeringScale,
  vehicleMaxSteerAngle,
  vehicleMinLookAhead,
  vehicleMaxLookAhead,
  zoneInspector,
  deleteZoneBtn,
} from '../core/dom.js';
import { listen } from '../core/events.js';
import { applyInspector, syncInspector } from './inspector.js';
import { applyZoneInspector } from '../zones/zone-ui.js';
import { selectionGroupId } from '../scene/selection.js';
import { selectedZone } from '../zones/zones.js';
import { setTransformMode } from '../interaction/canvas-controller.js';

export function initInspectorEvents() {
  for (const input of Object.values(inspector)) {
    listen(input, 'input', applyInspector);
  }

  for (const el of [
    vehicleSpeed,
    vehiclePathSpacing,
    vehicleSteeringScale,
    vehicleMaxSteerAngle,
    vehicleMinLookAhead,
    vehicleMaxLookAhead,
  ]) {
    listen(el, 'input', applyInspector);
  }

  for (const [mode, button] of Object.entries(modeButtons)) {
    listen(button, 'click', () => setTransformMode(mode));
  }

  listen(inspector.asset, 'input', () => {
    const gid = selectionGroupId();
    if (gid == null) return;
    const name = inspector.asset.value;
    for (const object of state.objects.filter((o) => o.vehicleGroupId === gid)) {
      object.groupName = name;
    }
  });

  listen(deleteZoneBtn, 'click', () => {
    if (!selectedZone()) return;
    state.randomZones = state.randomZones.filter((zone) => zone.id !== state.selectedZoneId);
    state.selectedZoneId = null;
    syncInspector();
  });

  for (const input of Object.values(zoneInspector)) {
    if (input instanceof HTMLElement && input !== zoneInspector.summary) {
      listen(input, 'input', applyZoneInspector);
      listen(input, 'change', applyZoneInspector);
    }
  }
}
