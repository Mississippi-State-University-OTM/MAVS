// Centralised application state shared across all modules.
// Always access via state.xxx — never destructure mutable arrays or objects.

export const state = {
  draftReady: false,

  assets: [],
  savedScenes: [],
  objects: [],
  randomZones: [],
  paths: [],
  vehicleDefs: [],

  selectedIds: new Set(),
  selectedZoneId: null,
  selectedPathId: null,
  selectedWaypointIndex: null,

  nextId: 1,
  nextZoneId: 1,
  nextPathId: 1,
  nextGroupId: 1,
  nextSensorId: 1,

  camera: {
    target: [0, 0, 0],
    distance: 70,
    yaw: -45 * Math.PI / 180,
    pitch: 52 * Math.PI / 180,
  },

  ui: {
    transformMode: "translate",
    zonePlacementMode: false,
    pathPlacementMode: false,
    orbiting: false,
    panning: false,
    lastPointer: null,
  },

  clipboard: {
    copiedObjects: [],
  },
};
