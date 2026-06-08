import { state } from '../core/state.js';
import { screenRay, groundPoint } from '../rendering/camera.js';
import { objectBounds } from '../scene/objects.js';

export function pickObject(clientX, clientY) {
  const ray = screenRay(clientX, clientY);
  let best = null;
  let bestDistance = Infinity;
  for (const object of state.objects) {
    const bounds = objectBounds(object);
    if (!bounds) continue;
    const distance = rayBoundsDistance(ray, bounds);
    if (distance !== null && distance < bestDistance) {
      best = object;
      bestDistance = distance;
    }
  }
  return best;
}

export function pickZone(clientX, clientY) {
  const point = groundPoint(clientX, clientY);
  for (let i = state.randomZones.length - 1; i >= 0; i--) {
    const zone = state.randomZones[i];
    if (point[0] >= zone.minX && point[0] <= zone.maxX && point[1] >= zone.minY && point[1] <= zone.maxY) {
      return zone;
    }
  }
  return null;
}
