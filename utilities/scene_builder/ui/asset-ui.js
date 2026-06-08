import { state } from '../core/state.js';
import { assetList, assetSearch, assetCount, vehicleDefList } from '../core/dom.js';
import { loadModel } from '../rendering/model-loader.js';
import { addObject } from '../scene/objects.js';
import { addVehicle } from '../vehicles/vehicles.js';
import { listen } from '../core/events.js';

export const dragState = { assetPath: null, vehicleDef: null };

const thumbnailObserver = new IntersectionObserver((entries) => {
  for (const entry of entries) {
    if (!entry.isIntersecting) continue;
    const canvas = entry.target;
    thumbnailObserver.unobserve(canvas);
    renderAssetThumbnail(canvas.dataset.path, canvas);
  }
}, { root: assetList, rootMargin: "180px" });

export function assetType(asset) {
  const parts = asset.path.split("/");
  const meshIndex = parts.indexOf("meshes");
  if (meshIndex >= 0 && parts[meshIndex + 1]) {
    return parts[meshIndex + 1];
  }
  if (parts[0] === "actors") {
    return "actors";
  }
  return parts[0] || "other";
}

async function renderAssetThumbnail(path, canvas) {
  const ctx = canvas.getContext("2d");
  const size = canvas.width;
  ctx.clearRect(0, 0, size, size);
  ctx.fillStyle = "#15191d";
  ctx.fillRect(0, 0, size, size);
  ctx.strokeStyle = "#303942";
  ctx.strokeRect(0.5, 0.5, size - 1, size - 1);
  try {
    const model = await loadModel(path);
    const triangles = model.localTriangles;
    if (!triangles.length) {
      throw new Error("No triangles");
    }
    const center = model.center;
    const radius = model.radius || 1;
    const projected = [];
    const limit = Math.max(1, Math.ceil(triangles.length / 900));
    for (let i = 0; i < triangles.length; i += limit) {
      const tri = triangles[i];
      projected.push(tri.map((point) => {
        const x = point[0] - center[0];
        const y = point[1] - center[1];
        const z = point[2] - center[2];
        return [
          size * 0.5 + ((x * 0.72) - (y * 0.48)) / radius * size * 0.34,
          size * 0.56 + ((x * 0.18) + (y * 0.18) - (z * 0.82)) / radius * size * 0.34,
        ];
      }));
    }
    ctx.fillStyle = "rgba(111, 182, 255, 0.42)";
    ctx.strokeStyle = "rgba(222, 240, 255, 0.82)";
    ctx.lineWidth = 1;
    for (const tri of projected) {
      ctx.beginPath();
      ctx.moveTo(tri[0][0], tri[0][1]);
      ctx.lineTo(tri[1][0], tri[1][1]);
      ctx.lineTo(tri[2][0], tri[2][1]);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
    }
  } catch (error) {
    ctx.fillStyle = "#2a323a";
    ctx.fillRect(10, 14, size - 20, size - 28);
    ctx.strokeStyle = "#6fb6ff";
    ctx.strokeRect(10.5, 14.5, size - 21, size - 29);
  }
}

export function renderAssets() {
  const query = assetSearch.value.trim().toLowerCase();
  const visible = state.assets.filter((asset) => asset.path.toLowerCase().includes(query));
  const groups = new Map();
  for (const asset of visible) {
    const group = assetType(asset);
    if (!groups.has(group)) {
      groups.set(group, []);
    }
    groups.get(group).push(asset);
  }
  assetList.innerHTML = "";
  const sortedGroups = [...groups.entries()].sort(([a], [b]) => a.localeCompare(b));
  for (const [group, groupAssets] of sortedGroups) {
    const details = document.createElement("details");
    details.className = "asset-group";
    details.open = Boolean(query);
    const summary = document.createElement("summary");
    summary.innerHTML = `<span>${group}</span><small>${groupAssets.length}</small>`;
    details.appendChild(summary);
    for (const asset of groupAssets.slice(0, 300)) {
      const button = document.createElement("button");
      button.className = "asset";
      button.draggable = true;
      const thumb = document.createElement("canvas");
      thumb.className = "asset-thumb";
      thumb.width = 56;
      thumb.height = 56;
      thumb.dataset.path = asset.path;
      const copy = document.createElement("span");
      copy.className = "asset-copy";
      copy.innerHTML = `<strong>${asset.name}</strong><small>${asset.folder}</small>`;
      button.appendChild(thumb);
      button.appendChild(copy);
      listen(button, "dragstart", (event) => {
        dragState.assetPath = asset.path;
        event.dataTransfer.setData("text/plain", asset.path);
      });
      listen(button, "dblclick", () => addObject(asset.path, [0, 0, 0]));
      details.appendChild(button);
      thumbnailObserver.observe(thumb);
    }
    assetList.appendChild(details);
  }
  assetCount.textContent = `${state.assets.length} OBJ assets`;
}

export function renderVehicleDefs() {
  if (!vehicleDefList) return;
  vehicleDefList.innerHTML = "";
  if (!state.vehicleDefs.length) {
    vehicleDefList.textContent = "No vehicle definitions found";
    return;
  }
  for (const def of state.vehicleDefs) {
    const button = document.createElement("button");
    button.className = "asset vehicle-def-item";
    button.draggable = true;
    const thumb = document.createElement("canvas");
    thumb.className = "asset-thumb";
    thumb.width = 56;
    thumb.height = 56;
    if (def.chassis_mesh) {
      thumb.dataset.path = def.chassis_mesh;
      thumbnailObserver.observe(thumb);
    }
    const copy = document.createElement("span");
    copy.className = "asset-copy";
    const axleCount = def.axles.length;
    copy.innerHTML = `<strong>${def.name}</strong><small>${axleCount} axle${axleCount === 1 ? "" : "s"}</small>`;
    button.appendChild(thumb);
    button.appendChild(copy);
    listen(button, "dragstart", (event) => {
      dragState.vehicleDef = def;
      dragState.assetPath = null;
      event.dataTransfer.setData("text/plain", `__vehicle__${def.name}`);
    });
    listen(button, "dragend", () => { dragState.vehicleDef = null; });
    listen(button, "dblclick", () => addVehicle(def, [0, 0, 0]));
    vehicleDefList.appendChild(button);
  }
}
