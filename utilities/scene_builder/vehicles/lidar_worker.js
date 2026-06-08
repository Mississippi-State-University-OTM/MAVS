// LiDAR ray-casting worker — runs off the main thread so the UI stays responsive

function elevLinear(n, lo, hi) {
  return Array.from({length: n}, (_, i) => n === 1 ? lo : lo + i * (hi - lo) / (n - 1));
}

const LIDAR_RING_ANGLES = {
  "VLP-16":  elevLinear(16, -15, 15),
  "HDL-32E": elevLinear(32, -30.67, 10.67),
  "HDL-64E": elevLinear(32, -24.9, 2.0),
  "M8":      elevLinear(8, -4.0, 8.0),
  "OS1":     elevLinear(16, -16.6, 16.6),
  "OS1-16":  elevLinear(16, -16.6, 16.6),
  "OS2":     elevLinear(32, -22.5, 22.5),
  "LMS-291": [0],
  "RS32":    elevLinear(32, -25.0, 15.0),
  "OS0":     elevLinear(32, -45.0, 45.0),
  "BPearl":  elevLinear(32, -90.0, -1.0),
  "FourPi":  elevLinear(16, -90.0, 90.0),
};

function rayTriIntersect(ox, oy, oz, dx, dy, dz, ax, ay, az, bx, by, bz, cx, cy, cz) {
  const e1x = bx-ax, e1y = by-ay, e1z = bz-az;
  const e2x = cx-ax, e2y = cy-ay, e2z = cz-az;
  const hx = dy*e2z - dz*e2y, hy = dz*e2x - dx*e2z, hz = dx*e2y - dy*e2x;
  const det = e1x*hx + e1y*hy + e1z*hz;
  if (Math.abs(det) < 1e-7) return Infinity;
  const inv = 1.0 / det;
  const sx = ox-ax, sy = oy-ay, sz = oz-az;
  const u = inv * (sx*hx + sy*hy + sz*hz);
  if (u < 0 || u > 1) return Infinity;
  const qx = sy*e1z - sz*e1y, qy = sz*e1x - sx*e1z, qz = sx*e1y - sy*e1x;
  const v = inv * (dx*qx + dy*qy + dz*qz);
  if (v < 0 || u + v > 1) return Infinity;
  const t = inv * (e2x*qx + e2y*qy + e2z*qz);
  return t > 1e-4 ? t : Infinity;
}

function elevColor(t) {
  if (t < 0.25) { const s = t / 0.25; return [0, 0.5 + 0.5*s, 1]; }
  if (t < 0.5)  { const s = (t - 0.25) / 0.25; return [0, 1, 1 - s]; }
  if (t < 0.75) { const s = (t - 0.5)  / 0.25; return [s, 1, 0]; }
  const s = (t - 0.75) / 0.25; return [1, 1 - s, 0];
}

self.onmessage = function(e) {
  const { jobId, ghostId, gx, gy, gz, visRange, tris, elevAngles, totalYaw, pitchRad, rollRad } = e.data;
  const AZ_STEPS = 72;

  const cy = Math.cos(totalYaw), sy = Math.sin(totalYaw);
  const cp = Math.cos(-pitchRad), sp = Math.sin(-pitchRad);
  const cr = Math.cos(rollRad),   sr = Math.sin(rollRad);

  // Pre-filter world triangles within visRange using AABB-sphere test
  const rangeSq = visRange * visRange;
  const nearTriIdx = [];
  for (let j = 0; j < tris.length; j += 9) {
    const minX = Math.min(tris[j], tris[j+3], tris[j+6]);
    const maxX = Math.max(tris[j], tris[j+3], tris[j+6]);
    const minY = Math.min(tris[j+1], tris[j+4], tris[j+7]);
    const maxY = Math.max(tris[j+1], tris[j+4], tris[j+7]);
    const minZ = Math.min(tris[j+2], tris[j+5], tris[j+8]);
    const maxZ = Math.max(tris[j+2], tris[j+5], tris[j+8]);
    const cx_ = Math.max(minX, Math.min(gx, maxX));
    const cy_ = Math.max(minY, Math.min(gy, maxY));
    const cz_ = Math.max(minZ, Math.min(gz, maxZ));
    if ((cx_-gx)**2 + (cy_-gy)**2 + (cz_-gz)**2 <= rangeSq) nearTriIdx.push(j);
  }

  const rings = [];
  for (let ri = 0; ri < elevAngles.length; ri++) {
    const elev = elevAngles[ri];
    const t01 = elevAngles.length < 2 ? 0.5 : ri / (elevAngles.length - 1);
    const color = elevColor(t01);
    const elevRad = elev * Math.PI / 180;
    const cosEl = Math.cos(elevRad), sinEl = Math.sin(elevRad);
    const hitPts = [];
    for (let i = 0; i < AZ_STEPS; i++) {
      const az = (i / AZ_STEPS) * 2 * Math.PI;
      let dx = cosEl * Math.cos(az), dy = cosEl * Math.sin(az), dz = sinEl;
      const dy1 = cr*dy - sr*dz, dz1 = sr*dy + cr*dz; dy = dy1; dz = dz1;
      const dx2 = cp*dx + sp*dz, dz2 = -sp*dx + cp*dz; dx = dx2; dz = dz2;
      const dx3 = cy*dx - sy*dy, dy3 = sy*dx + cy*dy; dx = dx3; dy = dy3;
      let tMin = visRange;
      for (const j of nearTriIdx) {
        const t = rayTriIntersect(gx, gy, gz, dx, dy, dz,
          tris[j], tris[j+1], tris[j+2],
          tris[j+3], tris[j+4], tris[j+5],
          tris[j+6], tris[j+7], tris[j+8]);
        if (t < tMin) tMin = t;
      }
      if (tMin < visRange) {
        hitPts.push(gx + dx*tMin, gy + dy*tMin, gz + dz*tMin);
      }
    }
    if (hitPts.length > 0) rings.push({ pts: new Float32Array(hitPts), color });
  }

  const transferables = rings.map(r => r.pts.buffer);
  self.postMessage({ jobId, ghostId, rings }, transferables);
};
