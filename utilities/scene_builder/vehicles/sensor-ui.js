// Sensor inspector rendering and visualization toggle state.
import { sensorItems, vizToggleBar, lidarVizBtn, cameraVizBtn } from '../core/dom.js';
import { SENSOR_MODELS, SENSOR_TYPES, sensorsFor } from './sensors.js';

export function renderSensorPanel(vehicle, selectedSensorId = null) {
  const sensors = sensorsFor(vehicle);
  sensorItems.innerHTML = "";
  for (const sensor of sensors) {
    const models = SENSOR_MODELS[sensor.type] || [];
    const isCollapsed = sensor.collapsed === true;
    const item = document.createElement("div");
    item.className = "sensor-item" + (sensor.id === selectedSensorId ? " selected" : "");
    item.dataset.sensorId = sensor.id;
    item.innerHTML = `
      <div class="sensor-header">
        <span class="sensor-type-badge">${sensor.type}</span>
        <input class="sensor-name" placeholder="Sensor name">
        <button class="sensor-collapse" title="${isCollapsed ? "Expand" : "Collapse"}">${isCollapsed ? "&#9654;" : "&#9660;"}</button>
        <button class="sensor-remove">&times;</button>
      </div>
      ${isCollapsed ? "" : `
      <div class="sensor-body">
        <select class="sensor-type">
          ${SENSOR_TYPES.map(t => `<option value="${t}"${t === sensor.type ? " selected" : ""}>${t}</option>`).join("")}
        </select>
        ${models.length ? `<select class="sensor-model">${models.map(m => `<option value="${m}"${m === sensor.model ? " selected" : ""}>${m}</option>`).join("")}</select>` : ""}
        <div class="path-vehicles-label">Offset from vehicle CG (m)</div>
        <div class="row">
          <label>X<input type="number" class="sensor-ox" step="0.1" value="${sensor.offset[0]}"></label>
          <label>Y<input type="number" class="sensor-oy" step="0.1" value="${sensor.offset[1]}"></label>
          <label>Z<input type="number" class="sensor-oz" step="0.1" value="${sensor.offset[2]}"></label>
        </div>
        <div class="path-vehicles-label">Orientation (&deg;)</div>
        <div class="row">
          <label>Yaw<input type="number" class="sensor-yaw" step="1" value="${sensor.yaw}"></label>
          <label>Pitch<input type="number" class="sensor-pitch" step="1" value="${sensor.pitch}"></label>
          <label>Roll<input type="number" class="sensor-roll" step="1" value="${sensor.roll}"></label>
        </div>
        <label>Update Rate (Hz)<input type="number" class="sensor-hz" step="0.5" min="0.1" value="${sensor.hz}"></label>
        ${sensor.type === "lidar" ? `
        <label class="sensor-rings-label"><input type="checkbox" class="sensor-rings"${sensor.showRings ? " checked" : ""}> Show point cloud preview</label>
        ${sensor.showRings ? `
        <div class="row two">
          <label>Vis range (m)<input type="number" class="sensor-vis-range" step="5" min="5" max="200" value="${sensor.visRange || 20}"></label>
          <label>Point size<input type="number" class="sensor-point-size" step="1" min="1" max="20" value="${sensor.pointSize || 3}"></label>
        </div>` : ""}
        ` : ""}
        ${sensor.type === "camera" ? `
        <label class="sensor-rings-label"><input type="checkbox" class="sensor-fov"${sensor.showFov ? " checked" : ""}> Show FOV</label>
        ` : ""}
        <button class="sensor-snap">Snap to mesh</button>
      </div>`}
    `;
    item.querySelector(".sensor-name").value = sensor.name;
    sensorItems.appendChild(item);
  }
}

export function syncVizToggleBar(vehicle) {
  const sensors = vehicle ? sensorsFor(vehicle) : [];
  const lidars  = sensors.filter(s => s.type === "lidar");
  const cameras = sensors.filter(s => s.type === "camera");
  vizToggleBar.hidden = !vehicle || (!lidars.length && !cameras.length);
  lidarVizBtn.hidden  = !lidars.length;
  cameraVizBtn.hidden = !cameras.length;
  lidarVizBtn.classList.toggle("active",  lidars.some(s => s.showRings));
  cameraVizBtn.classList.toggle("active", cameras.some(s => s.showFov));
}
