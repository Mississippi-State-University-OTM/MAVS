// OBJ loading, parsing, WebGL buffer creation, and model caching.

let gl;

export const modelCache = new Map();

export function initModelLoader(ctx) {
  ({ gl } = ctx);
}

export function makeModelFromObj(text) {
  const vertices = [];
  const triangles = [];
  const localTriangles = [];
  const rawPositions = [];
  const lines = text.split(/\r?\n/);
  for (const line of lines) {
    const trimmed = line.trim();
    if (trimmed.startsWith("v ")) {
      const parts = trimmed.split(/\s+/).slice(1).map(Number);
      vertices.push(parts);
    } else if (trimmed.startsWith("f ")) {
      const refs = trimmed.split(/\s+/).slice(1).map((part) => {
        const raw = Number(part.split("/")[0]);
        return raw < 0 ? vertices.length + raw : raw - 1;
      });
      for (let i = 1; i < refs.length - 1; i++) {
        triangles.push([refs[0], refs[i], refs[i + 1]]);
      }
    }
  }

  let min = [Infinity, Infinity, Infinity];
  let max = [-Infinity, -Infinity, -Infinity];
  for (const tri of triangles) {
    const a = vertices[tri[0]];
    const b = vertices[tri[1]];
    const c = vertices[tri[2]];
    const n = norm(cross(sub(b, a), sub(c, a)));
    localTriangles.push([a, b, c]);
    for (const p of [a, b, c]) {
      rawPositions.push(p[0], p[1], p[2], n[0], n[1], n[2]);
      min = [Math.min(min[0], p[0]), Math.min(min[1], p[1]), Math.min(min[2], p[2])];
      max = [Math.max(max[0], p[0]), Math.max(max[1], p[1]), Math.max(max[2], p[2])];
    }
  }
  const buffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(rawPositions), gl.STATIC_DRAW);
  const center = mul(add(min, max), 0.5);
  const radius = Math.max(1, Math.hypot(max[0] - min[0], max[1] - min[1], max[2] - min[2]) * 0.5);
  return { buffer, count: rawPositions.length / 6, min, max, center, radius, localTriangles };
}

export async function loadModel(path) {
  if (modelCache.has(path)) {
    return modelCache.get(path);
  }
  const response = await fetch(`/api/file?path=${encodeURIComponent(path)}`);
  if (!response.ok) {
    throw new Error(`Could not load ${path}`);
  }
  const model = makeModelFromObj(await response.text());
  modelCache.set(path, model);
  return model;
}
