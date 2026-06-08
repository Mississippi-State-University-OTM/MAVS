// Shared WebGL program, buffers, primitive geometry, grid, and plane state.

let gl;
let planeSizeInput;

export let program;
export let loc;
export let gridBuffer;
export let gizmoBuffer;
export let cylinderBuffer;
export let cylinderVertexCount = 0;
export let gridCount = 0;

export const planeRenderObject = {
  id: -1,
  mesh: "",
  position: [0, 0, 0],
  rotation: [0, 0, 0],
  scale: [1, 1, 1],
  color: [0.18, 0.22, 0.21],
};

export function initRenderer(ctx) {
  ({ gl, planeSizeInput } = ctx);
  planeRenderObject.mesh = ctx.planeMeshPath;
  if (!gl) return;

  program = makeProgram();
  loc = {
    position: gl.getAttribLocation(program, "aPosition"),
    normal: gl.getAttribLocation(program, "aNormal"),
    model: gl.getUniformLocation(program, "uModel"),
    viewProj: gl.getUniformLocation(program, "uViewProj"),
    color: gl.getUniformLocation(program, "uColor"),
    selected: gl.getUniformLocation(program, "uSelected"),
    pointSize: gl.getUniformLocation(program, "uPointSize"),
  };
  gl.useProgram(program);
  gl.uniform1f(loc.pointSize, 1.0);

  gridBuffer = gl.createBuffer();
  gizmoBuffer = gl.createBuffer();

  const cylinderGeoData = buildCylinderGeometry(0.1, 0.06, 14);
  cylinderBuffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, cylinderBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(cylinderGeoData), gl.STATIC_DRAW);
  cylinderVertexCount = cylinderGeoData.length / 6;

  gridCount = buildGrid();
}

export function compileShader(type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    throw new Error(gl.getShaderInfoLog(shader));
  }
  return shader;
}

export function makeProgram() {
  const vertex = compileShader(gl.VERTEX_SHADER, `
    attribute vec3 aPosition;
    attribute vec3 aNormal;
    uniform mat4 uModel;
    uniform mat4 uViewProj;
    uniform float uPointSize;
    varying vec3 vNormal;
    varying vec3 vWorld;
    void main() {
      vec4 world = uModel * vec4(aPosition, 1.0);
      vWorld = world.xyz;
      vNormal = mat3(uModel) * aNormal;
      gl_Position = uViewProj * world;
      gl_PointSize = uPointSize;
    }
  `);
  const fragment = compileShader(gl.FRAGMENT_SHADER, `
    precision mediump float;
    uniform vec3 uColor;
    uniform bool uSelected;
    varying vec3 vNormal;
    varying vec3 vWorld;
    void main() {
      vec3 light = normalize(vec3(-0.4, -0.5, 0.8));
      float shade = max(dot(normalize(vNormal), light), 0.0) * 0.55 + 0.35;
      vec3 color = uColor * shade;
      if (uSelected) {
        color = mix(color, vec3(0.45, 0.72, 1.0), 0.45);
      }
      gl_FragColor = vec4(color, 1.0);
    }
  `);
  const nextProgram = gl.createProgram();
  gl.attachShader(nextProgram, vertex);
  gl.attachShader(nextProgram, fragment);
  gl.linkProgram(nextProgram);
  if (!gl.getProgramParameter(nextProgram, gl.LINK_STATUS)) {
    throw new Error(gl.getProgramInfoLog(nextProgram));
  }
  return nextProgram;
}

export function buildCylinderGeometry(radius, halfHeight, segments) {
  const data = [];
  const v = (x, y, z, nx, ny, nz) => data.push(x, y, z, nx, ny, nz);
  for (let i = 0; i < segments; i++) {
    const a0 = (i / segments) * 2 * Math.PI;
    const a1 = ((i + 1) / segments) * 2 * Math.PI;
    const c0 = Math.cos(a0), s0 = Math.sin(a0);
    const c1 = Math.cos(a1), s1 = Math.sin(a1);
    v(c0*radius, s0*radius, -halfHeight, c0, s0, 0);
    v(c1*radius, s1*radius, -halfHeight, c1, s1, 0);
    v(c1*radius, s1*radius,  halfHeight, c1, s1, 0);
    v(c0*radius, s0*radius, -halfHeight, c0, s0, 0);
    v(c1*radius, s1*radius,  halfHeight, c1, s1, 0);
    v(c0*radius, s0*radius,  halfHeight, c0, s0, 0);
    v(0, 0,  halfHeight, 0, 0,  1);
    v(c0*radius, s0*radius,  halfHeight, 0, 0,  1);
    v(c1*radius, s1*radius,  halfHeight, 0, 0,  1);
    v(0, 0, -halfHeight, 0, 0, -1);
    v(c1*radius, s1*radius, -halfHeight, 0, 0, -1);
    v(c0*radius, s0*radius, -halfHeight, 0, 0, -1);
  }
  return data;
}

export function buildGrid(halfSize = 50) {
  const rawStep = halfSize / 10;
  const magnitude = Math.pow(10, Math.floor(Math.log10(rawStep)));
  const n = rawStep / magnitude;
  const step = n <= 1 ? magnitude : n <= 2 ? 2 * magnitude : n <= 5 ? 5 * magnitude : 10 * magnitude;
  const lines = [];
  for (let i = -halfSize; i <= halfSize; i += step) {
    lines.push(-halfSize, i, 0.01, halfSize, i, 0.01);
    lines.push(i, -halfSize, 0.01, i, halfSize, 0.01);
  }
  gl.bindBuffer(gl.ARRAY_BUFFER, gridBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(lines), gl.DYNAMIC_DRAW);
  return lines.length / 3;
}

export function setPlaneScale(s) {
  planeRenderObject.scale[0] = s;
  planeRenderObject.scale[1] = s;
  if (planeSizeInput) {
    planeSizeInput.value = Math.round(s * 100);
  }
  if (gl && gridBuffer) {
    gridCount = buildGrid(s * 50);
  }
}
