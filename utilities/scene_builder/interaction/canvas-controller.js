import { state } from '../core/state.js';
import { canvas, addRandomZoneButton, modeButtons } from '../core/dom.js';
import { initCamera, cameraPosition, groundPoint } from '../rendering/camera.js';
import { listen } from '../core/events.js';
import { gizmoAxes, pickGizmoAxis } from '../rendering/gizmo.js';
import { dragState } from '../ui/asset-ui.js';
import { pickObject, pickZone } from './picking.js';
import {
  selectedObjects, hasSelection,
  selectObject, selectZone,
  selectionTransformState, selectionCenter, selectionBounds,
  cloneSelectionData,
} from '../scene/selection.js';
import {
  addObject, objectById, duplicateSelectionAtSamePosition, pasteCopiedObject,
  objectWorldAABB,
} from '../scene/objects.js';
import { addVehicle, vehicleGroupObjects } from '../vehicles/vehicles.js';
import { sensorsFor, updateSensorOffsetFromGhost, syncGhostPositionsForTransforms } from '../vehicles/sensors.js';
import { invalidateLidarCache, invalidateLidarForBounds } from '../vehicles/lidar-preview.js';
import { finishZonePlacement, setZonePlacementMode, selectedZone } from '../zones/zones.js';
import {
  selectedPath, setPathPlacementMode, addWaypointToSelectedPath, pickWaypointNode,
} from '../paths/paths.js';
import { syncPathInspector } from '../paths/path-ui.js';
import { syncInspector } from '../ui/inspector.js';
import { saveDraft } from '../scene/draft-storage.js';
import { startMarquee, updateMarquee, finishMarquee, clearMarquee, isMarqueeActive } from './marquee-selection.js';

let _setStatus = () => {};

let draggingSelection = null;
let draggingGizmo = null;
let placingZone = null;
let draggingWaypoint = null;
const pressedKeys = new Set();
const dragThreshold = 6;

function disableLidarRingsForDrag(transforms) {
  if (!transforms) return;
  const affectedGids = new Set();
  for (const { id } of transforms) {
    const obj = objectById(id);
    if (obj?.vehicleGroupId != null) affectedGids.add(obj.vehicleGroupId);
  }
  if (!affectedGids.size) return;
  let changed = false;
  for (const gid of affectedGids) {
    const chassis = state.objects.find(o => o.vehicleGroupId === gid && o.vehicleRole === "chassis");
    if (!chassis) continue;
    for (const sensor of sensorsFor(chassis)) {
      if (sensor.showRings) { sensor.showRings = false; changed = true; }
    }
  }
  if (changed) { syncInspector(); saveDraft(); }
}

export function setTransformMode(mode) {
  setZonePlacementMode(false);
  state.ui.transformMode = mode;
  for (const [buttonMode, button] of Object.entries(modeButtons)) {
    button.classList.toggle("active", buttonMode === mode);
  }
  const label = mode === "translate" ? "Move" : mode[0].toUpperCase() + mode.slice(1);
  _setStatus(`${label} gizmo active`);
}

export function getPlacingZone() {
  return placingZone;
}

export function cancelZonePlacement() {
  placingZone = null;
}

export function initCanvasController({ setStatus }) {
  _setStatus = setStatus;
  initCamera({ pressedKeys });

  listen(canvas, "dragover", (event) => {
    event.preventDefault();
  });

  listen(canvas, "drop", async (event) => {
    event.preventDefault();
    const dropPos = groundPoint(event.clientX, event.clientY);
    if (dragState.vehicleDef) {
      const def = dragState.vehicleDef;
      dragState.vehicleDef = null;
      await addVehicle(def, dropPos);
      return;
    }
    const path = event.dataTransfer.getData("text/plain") || dragState.assetPath;
    if (!path || path.startsWith("vehicle")) {
      dragState.vehicleDef = null;
      return;
    }
    _setStatus(`Loading ${path}`);
    await addObject(path, dropPos);
    _setStatus(`Placed ${path}`);
  });

  listen(canvas, "pointerdown", (event) => {
    canvas.setPointerCapture(event.pointerId);
    state.ui.lastPointer = [event.clientX, event.clientY];
    if (event.button === 2) {
      state.ui.orbiting = true;
      return;
    }
    if (event.button === 1 || event.shiftKey) {
      state.ui.panning = true;
      return;
    }
    if (state.ui.zonePlacementMode) {
      const point = groundPoint(event.clientX, event.clientY);
      placingZone = { start: point, current: point };
      return;
    }
    if (state.ui.pathPlacementMode) {
      const nodeIndex = pickWaypointNode(event.clientX, event.clientY);
      if (nodeIndex !== null) {
        state.selectedWaypointIndex = nodeIndex;
        draggingWaypoint = {
          index: nodeIndex,
          startPointer: [event.clientX, event.clientY],
          moved: false,
        };
        syncPathInspector();
      } else {
        const point = groundPoint(event.clientX, event.clientY);
        addWaypointToSelectedPath(point[0], point[1]);
      }
      return;
    }
    const axis = pickGizmoAxis(event.clientX, event.clientY);
    if (axis) {
      if (event.altKey && state.ui.transformMode === "translate") {
        const count = duplicateSelectionAtSamePosition();
        if (count) {
          _setStatus(`Duplicated ${count} object${count === 1 ? "" : "s"}`);
        }
      }
      const solo = selectedObjects();
      let gizmoCenter = selectionCenter();
      if (state.ui.transformMode === "rotate" && solo.length === 1 && solo[0].vehicleRole === "camera_ghost") {
        const chassis = state.objects.find(o => o.vehicleGroupId === solo[0].vehicleGroupId && o.vehicleRole === "chassis");
        if (chassis) gizmoCenter = [...chassis.position];
      }
      draggingGizmo = {
        axis,
        mode: state.ui.transformMode,
        moved: false,
        startTransforms: selectionTransformState(),
        center: gizmoCenter,
        startGround: groundPoint(event.clientX, event.clientY),
        startPointerX: event.clientX,
        startPointerY: event.clientY,
      };
      return;
    }
    const picked = pickObject(event.clientX, event.clientY);
    if (picked) {
      if (!state.selectedIds.has(picked.id)) {
        selectObject(picked.id);
      }
      draggingSelection = {
        pickedId: picked.id,
        startPointer: [event.clientX, event.clientY],
        moved: false,
        startTransforms: selectionTransformState(),
        startGround: groundPoint(event.clientX, event.clientY),
      };
    } else {
      const zone = pickZone(event.clientX, event.clientY);
      if (zone) {
        selectZone(zone.id);
        return;
      }
      startMarquee(event.clientX, event.clientY, event.ctrlKey);
    }
  });

  listen(canvas, "pointermove", (event) => {
    if (!state.ui.lastPointer) {
      return;
    }
    const dx = event.clientX - state.ui.lastPointer[0];
    const dy = event.clientY - state.ui.lastPointer[1];
    state.ui.lastPointer = [event.clientX, event.clientY];
    if (state.ui.orbiting) {
      state.camera.yaw -= dx * 0.006;
      state.camera.pitch = Math.max(0.1, Math.min(1.45, state.camera.pitch + dy * 0.006));
    } else if (state.ui.panning) {
      const eye = cameraPosition();
      const forward = norm(sub(state.camera.target, eye));
      const right = norm(cross(forward, [0, 0, 1]));
      const up = norm(cross(right, forward));
      state.camera.target = add(state.camera.target, add(mul(right, -dx * 0.06), mul(up, dy * 0.06)));
    } else if (placingZone) {
      placingZone.current = groundPoint(event.clientX, event.clientY);
    } else if (draggingWaypoint) {
      const path = selectedPath();
      if (path && path.waypoints[draggingWaypoint.index]) {
        draggingWaypoint.moved = true;
        const point = groundPoint(event.clientX, event.clientY);
        path.waypoints[draggingWaypoint.index] = [
          parseFloat(point[0].toFixed(3)),
          parseFloat(point[1].toFixed(3)),
        ];
        syncPathInspector();
      }
    } else if (draggingGizmo) {
      draggingGizmo.moved ||= Math.hypot(
        event.clientX - draggingGizmo.startPointerX,
        event.clientY - draggingGizmo.startPointerY,
      ) >= dragThreshold;
      if (!draggingGizmo.moved) {
        return;
      }
      if (draggingGizmo.mode === "rotate") {
        const rotationIndex = { x: 0, y: 1, z: 2 }[draggingGizmo.axis];
        const amount = (event.clientX - draggingGizmo.startPointerX - (event.clientY - draggingGizmo.startPointerY)) * 0.45;
        for (const start of draggingGizmo.startTransforms) {
          const object = objectById(start.id);
          if (!object) continue;
          object.position = rotatePointAroundAxis(start.position, draggingGizmo.center, draggingGizmo.axis, amount);
          object.rotation = [...start.rotation];
          object.rotation[rotationIndex] = start.rotation[rotationIndex] + amount;
        }
        for (const start of draggingGizmo.startTransforms) {
          const object = objectById(start.id);
          if (object?.vehicleRole === "sensor_ghost") updateSensorOffsetFromGhost(object);
        }
        syncGhostPositionsForTransforms(draggingGizmo.startTransforms);
        invalidateLidarCache();
      } else if (draggingGizmo.mode === "scale") {
        const scaleIndex = { x: 0, y: 1, z: 2 }[draggingGizmo.axis];
        let amount = 0;
        if (draggingGizmo.axis === "z") {
          amount = -(event.clientY - draggingGizmo.startPointerY) * 0.02;
        } else {
          const point = groundPoint(event.clientX, event.clientY);
          const delta = sub(point, draggingGizmo.startGround);
          amount = dot(delta, gizmoAxes[draggingGizmo.axis].vector) * 0.08;
        }
        const factor = Math.max(0.05, 1 + amount);
        for (const start of draggingGizmo.startTransforms) {
          const object = objectById(start.id);
          if (!object) continue;
          const relative = sub(start.position, draggingGizmo.center);
          const scaledRelative = [...relative];
          scaledRelative[scaleIndex] *= factor;
          object.position = add(draggingGizmo.center, scaledRelative);
          object.scale = [...start.scale];
          object.scale[scaleIndex] = Math.max(0.01, start.scale[scaleIndex] * factor);
        }
      } else {
        let deltaMove = [0, 0, 0];
        if (draggingGizmo.axis === "z") {
          const scale = Math.max(0.02, state.camera.distance * 0.0025);
          deltaMove = [0, 0, -(event.clientY - draggingGizmo.startPointerY) * scale];
        } else {
          const point = groundPoint(event.clientX, event.clientY);
          const delta = sub(point, draggingGizmo.startGround);
          const axisVector = gizmoAxes[draggingGizmo.axis].vector;
          deltaMove = mul(axisVector, dot(delta, axisVector));
        }
        for (const start of draggingGizmo.startTransforms) {
          const object = objectById(start.id);
          if (!object) continue;
          object.position = add(start.position, deltaMove);
        }
        for (const start of draggingGizmo.startTransforms) {
          const object = objectById(start.id);
          if (object?.vehicleRole === "sensor_ghost") updateSensorOffsetFromGhost(object);
        }
        syncGhostPositionsForTransforms(draggingGizmo.startTransforms);
        invalidateLidarCache();
      }
      syncInspector();
    } else if (draggingSelection) {
      draggingSelection.moved ||= Math.hypot(
        event.clientX - draggingSelection.startPointer[0],
        event.clientY - draggingSelection.startPointer[1],
      ) >= dragThreshold;
      if (!draggingSelection.moved) {
        return;
      }
      const point = groundPoint(event.clientX, event.clientY);
      const delta = sub(point, draggingSelection.startGround);
      for (const start of draggingSelection.startTransforms) {
        const object = objectById(start.id);
        if (!object) continue;
        object.position = [start.position[0] + delta[0], start.position[1] + delta[1], start.position[2]];
      }
      syncGhostPositionsForTransforms(draggingSelection.startTransforms);
      invalidateLidarCache();
      syncInspector();
    } else if (isMarqueeActive()) {
      updateMarquee(event.clientX, event.clientY);
    }
  });

  listen(canvas, "pointerup", () => {
    if (placingZone) {
      placingZone = finishZonePlacement(placingZone);
    } else if (draggingSelection && !draggingSelection.moved) {
      selectObject(draggingSelection.pickedId);
    } else if (draggingGizmo && !draggingGizmo.moved) {
      const picked = pickObject(draggingGizmo.startPointerX, draggingGizmo.startPointerY);
      if (picked) selectObject(picked.id);
    } else {
      finishMarquee();
    }
    if (draggingSelection?.moved) disableLidarRingsForDrag(draggingSelection.startTransforms);
    if (draggingGizmo?.moved) disableLidarRingsForDrag(draggingGizmo.startTransforms);
    draggingWaypoint = null;
    draggingSelection = null;
    draggingGizmo = null;
    state.ui.orbiting = false;
    state.ui.panning = false;
    state.ui.lastPointer = null;
  });

  listen(canvas, "pointercancel", () => {
    draggingWaypoint = null;
    draggingSelection = null;
    draggingGizmo = null;
    clearMarquee();
    placingZone = null;
    state.ui.orbiting = false;
    state.ui.panning = false;
    state.ui.lastPointer = null;
  });

  listen(canvas, "contextmenu", (event) => event.preventDefault());

  listen(canvas, "wheel", (event) => {
    event.preventDefault();
    state.camera.distance = Math.max(5, Math.min(500, state.camera.distance * (event.deltaY > 0 ? 1.08 : 0.92)));
  }, { passive: false });

  listen(window, "keydown", (event) => {
    const tag = event.target && event.target.tagName;
    const isTyping = tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT";
    const key = event.key.toLowerCase();
    if (!isTyping && key === "escape" && state.ui.pathPlacementMode) {
      setPathPlacementMode(false);
      _setStatus("Path placement paused");
      return;
    }
    if (!isTyping && key === "escape" && state.ui.zonePlacementMode) {
      placingZone = null;
      setZonePlacementMode(false);
      _setStatus("Random zone placement cancelled");
      return;
    }
    if (!isTyping && event.ctrlKey && key === "c") {
      if (hasSelection()) {
        state.clipboard.copiedObjects = cloneSelectionData();
        _setStatus(`Copied ${state.clipboard.copiedObjects.length} object${state.clipboard.copiedObjects.length === 1 ? "" : "s"}`);
        event.preventDefault();
      }
      return;
    }
    if (!isTyping && event.ctrlKey && key === "v") {
      pasteCopiedObject();
      event.preventDefault();
      return;
    }
    if (!isTyping) {
      if (["w", "a", "s", "d"].includes(key)) {
        if (key === "w" && hasSelection() && !state.ui.orbiting) {
          setTransformMode("translate");
        } else {
          pressedKeys.add(key);
        }
        event.preventDefault();
        return;
      }
      if (key === "e") {
        setTransformMode("rotate");
        return;
      }
      if (key === "r") {
        setTransformMode("scale");
        return;
      }
      if (key === "v") {
        const sel = selectedObjects();
        const vehicle = sel.find(o => o.vehicleDefName)
          || state.objects.find(o => o.vehicleRole === "chassis" && vehicleGroupObjects(o).some(g => g.vehicleRole === "sensor_ghost" && state.selectedIds.has(g.id)));
        if (vehicle) {
          const lidars = sensorsFor(vehicle).filter(s => s.type === "lidar");
          if (lidars.length) {
            const next = !lidars.some(s => s.showRings);
            for (const s of lidars) s.showRings = next;
            syncInspector();
            saveDraft();
            _setStatus(next ? "LiDAR preview on" : "LiDAR preview off");
          }
        }
        return;
      }
      if (key === "f" && hasSelection()) {
        state.camera.target = selectionCenter();
        const bounds = selectionBounds();
        if (bounds) {
          const dx = bounds.max[0] - bounds.min[0];
          const dy = bounds.max[1] - bounds.min[1];
          const dz = bounds.max[2] - bounds.min[2];
          state.camera.distance = Math.max(5, Math.sqrt(dx*dx + dy*dy + dz*dz) * 1.5);
        } else {
          state.camera.distance = 20;
        }
        return;
      }
    }
    if (!isTyping && (event.key === "Delete" || event.key === "Backspace")) {
      if (state.selectedWaypointIndex !== null && state.selectedPathId !== null) {
        const path = selectedPath();
        if (path) {
          path.waypoints.splice(state.selectedWaypointIndex, 1);
          state.selectedWaypointIndex = state.selectedWaypointIndex >= path.waypoints.length ? null : state.selectedWaypointIndex;
          syncPathInspector();
        }
      } else if (selectedZone()) {
        state.randomZones = state.randomZones.filter((zone) => zone.id !== state.selectedZoneId);
        state.selectedZoneId = null;
        syncInspector();
      } else if (hasSelection()) {
        for (const id of state.selectedIds) {
          const _delObj = objectById(id);
          if (_delObj && _delObj.vehicleRole !== "sensor_ghost" && _delObj.vehicleRole !== "camera_ghost") {
            const _delAABB = objectWorldAABB(_delObj);
            if (_delAABB) invalidateLidarForBounds(_delAABB);
          }
        }
        state.objects = state.objects.filter((item) => !state.selectedIds.has(item.id));
        selectObject(null);
      }
    }
  });

  listen(window, "keyup", (event) => {
    pressedKeys.delete(event.key.toLowerCase());
  });
}
