import { state } from '../core/state.js';
import { canvas, selectionMarquee } from '../core/dom.js';
import { projectToScreen } from '../rendering/camera.js';
import { objectBounds, objectById } from '../scene/objects.js';
import { selectObject, selectObjects } from '../scene/selection.js';

const dragThreshold = 6;
let marqueeSelection = null;

function marqueeRect() {
  if (!marqueeSelection) return null;
  const x1 = Math.min(marqueeSelection.start[0], marqueeSelection.current[0]);
  const y1 = Math.min(marqueeSelection.start[1], marqueeSelection.current[1]);
  const x2 = Math.max(marqueeSelection.start[0], marqueeSelection.current[0]);
  const y2 = Math.max(marqueeSelection.start[1], marqueeSelection.current[1]);
  return { x1, y1, x2, y2 };
}

function updateMarqueeElement() {
  const rect = marqueeRect();
  if (!rect) {
    selectionMarquee.hidden = true;
    return;
  }
  const workspaceRect = canvas.parentElement.getBoundingClientRect();
  selectionMarquee.hidden = false;
  selectionMarquee.style.left = `${rect.x1 - workspaceRect.left}px`;
  selectionMarquee.style.top = `${rect.y1 - workspaceRect.top}px`;
  selectionMarquee.style.width = `${rect.x2 - rect.x1}px`;
  selectionMarquee.style.height = `${rect.y2 - rect.y1}px`;
}

function objectIntersectsScreenRect(object, rect) {
  const bounds = objectBounds(object);
  if (!bounds) return false;
  const points = [];
  for (const x of [bounds.min[0], bounds.max[0]]) {
    for (const y of [bounds.min[1], bounds.max[1]]) {
      for (const z of [bounds.min[2], bounds.max[2]]) {
        points.push(projectToScreen([x, y, z]));
      }
    }
  }
  const minX = Math.min(...points.map((p) => p[0]));
  const maxX = Math.max(...points.map((p) => p[0]));
  const minY = Math.min(...points.map((p) => p[1]));
  const maxY = Math.max(...points.map((p) => p[1]));
  return maxX >= rect.x1 && minX <= rect.x2 && maxY >= rect.y1 && minY <= rect.y2;
}

function selectObjectsInMarquee(additive) {
  const rect = marqueeRect();
  if (!rect) return;
  const rawIds = state.objects
    .filter((object) => objectIntersectsScreenRect(object, rect))
    .map((object) => object.id);
  const groupsHit = new Set(
    rawIds.map((id) => objectById(id)?.vehicleGroupId).filter((gid) => gid != null),
  );
  const ids = [...new Set([
    ...rawIds,
    ...state.objects
      .filter((o) => o.vehicleGroupId != null && groupsHit.has(o.vehicleGroupId))
      .map((o) => o.id),
  ])];
  if (additive) {
    selectObjects([...new Set([...state.selectedIds, ...ids])]);
  } else {
    selectObjects(ids);
  }
}

export function isMarqueeActive() {
  return marqueeSelection !== null;
}

export function startMarquee(clientX, clientY, additive) {
  marqueeSelection = {
    start: [clientX, clientY],
    current: [clientX, clientY],
    moved: false,
    additive,
  };
}

export function updateMarquee(clientX, clientY) {
  if (!marqueeSelection) return;
  marqueeSelection.current = [clientX, clientY];
  marqueeSelection.moved ||= Math.hypot(
    clientX - marqueeSelection.start[0],
    clientY - marqueeSelection.start[1],
  ) >= dragThreshold;
  if (marqueeSelection.moved) {
    updateMarqueeElement();
  }
}

export function finishMarquee() {
  if (!marqueeSelection) return;
  if (marqueeSelection.moved) {
    selectObjectsInMarquee(marqueeSelection.additive);
  } else {
    selectObject(null);
  }
  marqueeSelection = null;
  updateMarqueeElement();
}

export function clearMarquee() {
  marqueeSelection = null;
  updateMarqueeElement();
}
