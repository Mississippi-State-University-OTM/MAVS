// All DOM element references for the scene builder.

// Viewport
export const canvas = document.getElementById("viewport");
export const gl = canvas.getContext("webgl", { antialias: true });

// Menubar
export const newSceneBtn = document.getElementById("newScene");
export const loadSceneBtn = document.getElementById("loadScene");
export const saveSceneBtn = document.getElementById("saveScene");
export const exportSimulationBtn = document.getElementById("exportSimulation");
export const previewSceneBtn = document.getElementById("previewScene");
export const runSimulationBtn = document.getElementById("runSimulation");
export const scenePicker = document.getElementById("scenePicker");
export const sceneNameInput = document.getElementById("sceneName");

// Statusbar
export const statusEl = document.getElementById("status");
export const planeSizeInput = document.getElementById("planeSize");
export const lidarProgress = document.getElementById("lidarProgress");
export const vizToggleBar = document.getElementById("vizToggleBar");
export const lidarVizBtn = document.getElementById("toggleLidarViz");
export const cameraVizBtn = document.getElementById("toggleCameraViz");

// Sidebar
export const assetList = document.getElementById("assetList");
export const assetSearch = document.getElementById("assetSearch");
export const assetCount = document.getElementById("assetCount");
export const assetPaths = document.getElementById("assetPaths");
export const vehicleDefList = document.getElementById("vehicleDefList");
export const pathList = document.getElementById("pathList");
export const addPathBtn = document.getElementById("addPath");
export const addRandomZoneButton = document.getElementById("addRandomZone");

// Inspector shell
export const inspectorEl = document.querySelector(".inspector");
export const inspectorTitle = document.getElementById("inspectorTitle");
export const selectionPanel = document.getElementById("selectionPanel");
export const emptySelection = document.getElementById("emptySelection");
export const selectionMarquee = document.getElementById("selectionMarquee");
export const zonePanel = document.getElementById("zonePanel");

// Selection inspector fields
export const inspector = {
  asset:  document.getElementById("selectedAsset"),
  posX:   document.getElementById("posX"),
  posY:   document.getElementById("posY"),
  posZ:   document.getElementById("posZ"),
  rotZ:   document.getElementById("rotZ"),
  rotY:   document.getElementById("rotY"),
  rotX:   document.getElementById("rotX"),
  scaleX: document.getElementById("scaleX"),
  scaleY: document.getElementById("scaleY"),
  scaleZ: document.getElementById("scaleZ"),
};

// Gizmo mode buttons
export const modeButtons = {
  translate: document.getElementById("modeTranslate"),
  rotate:    document.getElementById("modeRotate"),
  scale:     document.getElementById("modeScale"),
};

// Object actions
export const deleteObjectBtn   = document.getElementById("deleteObject");
export const ungroupObjectsBtn = document.getElementById("ungroupObjects");
export const groupObjectsBtn   = document.getElementById("groupObjects");
export const newGroupNameInput = document.getElementById("newGroupNameInput");
export const snapToGroundBtn   = document.getElementById("snapToGround");

// Group / vehicle rows
export const groupInfoRow  = document.getElementById("groupInfoRow");
export const createGroupRow = document.getElementById("createGroupRow");

// Sensor panel
export const sensorListPanel = document.getElementById("sensorListPanel");
export const sensorItems     = document.getElementById("sensorItems");
export const addSensorBtn    = document.getElementById("addSensorBtn");

// Vehicle controller panel
export const vehicleControllerPanel = document.getElementById("vehicleControllerPanel");
export const vehicleControllerMode  = document.getElementById("vehicleControllerMode");
export const followPathRow          = document.getElementById("followPathRow");
export const followPath             = document.getElementById("followPath");
export const vehicleParamsPanel     = document.getElementById("vehicleParams");
export const vehicleSpeed           = document.getElementById("vehicleSpeed");
export const vehiclePathSpacing     = document.getElementById("vehiclePathSpacing");
export const vehicleSteeringScale   = document.getElementById("vehicleSteeringScale");
export const vehicleMaxSteerAngle   = document.getElementById("vehicleMaxSteerAngle");
export const vehicleMinLookAhead    = document.getElementById("vehicleMinLookAhead");
export const vehicleMaxLookAhead    = document.getElementById("vehicleMaxLookAhead");

// Path inspector
export const pathPanel             = document.getElementById("pathPanel");
export const pathNameInput         = document.getElementById("pathName");
export const pathWaypointCount     = document.getElementById("pathWaypointCount");
export const pathColorBar          = document.getElementById("pathColorBar");
export const pathVehicleList       = document.getElementById("pathVehicleList");
export const pathAddVehicle        = document.getElementById("pathAddVehicle");
export const pathAddVehicleBtn     = document.getElementById("pathAddVehicleBtn");
export const togglePathPlacementBtn = document.getElementById("togglePathPlacement");
export const clearLastWaypointBtn  = document.getElementById("clearLastWaypoint");
export const saveWaypointsBtn      = document.getElementById("saveWaypoints");
export const deletePathBtn         = document.getElementById("deletePath");

// Zone inspector fields
export const zoneInspector = {
  mesh:      document.getElementById("zoneMesh"),
  placement: document.getElementById("zonePlacement"),
  density:   document.getElementById("zoneDensity"),
  number:    document.getElementById("zoneNumber"),
  spacing:   document.getElementById("zoneSpacing"),
  offsetZ:   document.getElementById("zoneOffsetZ"),
  scaleMin:  document.getElementById("zoneScaleMin"),
  scaleMax:  document.getElementById("zoneScaleMax"),
  minX:      document.getElementById("zoneMinX"),
  maxX:      document.getElementById("zoneMaxX"),
  minY:      document.getElementById("zoneMinY"),
  maxY:      document.getElementById("zoneMaxY"),
  summary:   document.getElementById("zoneSummary"),
};

// Zone actions
export const deleteZoneBtn = document.getElementById("deleteZone");
