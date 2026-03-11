#!/usr/bin/env python3
"""
Wavefront OBJ Viewer & Transformer
─────────────────────────────────────────────────────────────────────────────
Fixed front view (camera looks along +Y, Z is up).
On-screen HUD shows current mesh center and dimensions.
All transforms are entered via keyboard prompts.

Requirements:
    pip install PyOpenGL PyOpenGL_accelerate pygame numpy Pillow
─────────────────────────────────────────────────────────────────────────────
python obj_viewer.py your_model.obj
"""

import sys
import os
import math
import shutil
import numpy as np

try:
    import pygame
    from pygame.locals import *
    from OpenGL.GL import *
    from OpenGL.GLU import *
    from PIL import Image
except ImportError as e:
    print(f"Missing dependency: {e}")
    print("Install with: pip install PyOpenGL PyOpenGL_accelerate pygame numpy Pillow")
    sys.exit(1)


# ─────────────────────────────────────────────────────────────────────────────
# OBJ / MTL Parsing
# ─────────────────────────────────────────────────────────────────────────────

def parse_mtl(mtl_path):
    materials = {}
    current = None
    base_dir = os.path.dirname(mtl_path)
    try:
        with open(mtl_path, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                cmd = parts[0].lower()
                if cmd == "newmtl":
                    current = " ".join(parts[1:])
                    materials[current] = {
                        "Ka": (0.2, 0.2, 0.2), "Kd": (0.8, 0.8, 0.8),
                        "Ks": (0.0, 0.0, 0.0), "Ns": 0.0, "d": 1.0,
                        "map_Kd_path": None,
                    }
                elif current:
                    mat = materials[current]
                    if cmd == "ka" and len(parts) >= 4:
                        mat["Ka"] = tuple(float(x) for x in parts[1:4])
                    elif cmd == "kd" and len(parts) >= 4:
                        mat["Kd"] = tuple(float(x) for x in parts[1:4])
                    elif cmd == "ks" and len(parts) >= 4:
                        mat["Ks"] = tuple(float(x) for x in parts[1:4])
                    elif cmd == "ns" and len(parts) >= 2:
                        mat["Ns"] = float(parts[1])
                    elif cmd in ("d", "tr") and len(parts) >= 2:
                        mat["d"] = float(parts[1])
                    elif cmd == "map_kd" and len(parts) >= 2:
                        tex_file = " ".join(parts[1:])
                        mat["map_Kd_path"] = os.path.join(base_dir, tex_file)
    except FileNotFoundError:
        print(f"Warning: MTL file not found: {mtl_path}")
    return materials


def parse_obj(obj_path):
    base_dir = os.path.dirname(os.path.abspath(obj_path))
    vertices, texcoords, normals = [], [], []
    groups, mtl_files, materials = [], [], {}
    current_group = None
    current_mat = None

    def new_group(name, mat):
        g = {"name": name, "material": mat, "faces": []}
        groups.append(g)
        return g

    with open(obj_path, "r", encoding="utf-8", errors="replace") as f:
        raw_lines = f.readlines()

    for line in raw_lines:
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        parts = stripped.split()
        cmd = parts[0].lower()

        if cmd == "mtllib":
            for mtl_name in parts[1:]:
                mtl_files.append(mtl_name)
                materials.update(parse_mtl(os.path.join(base_dir, mtl_name)))
        elif cmd == "v":
            vertices.append(tuple(float(x) for x in parts[1:4]))
        elif cmd == "vt":
            texcoords.append((float(parts[1]) if len(parts) > 1 else 0.0,
                              float(parts[2]) if len(parts) > 2 else 0.0))
        elif cmd == "vn":
            normals.append(tuple(float(x) for x in parts[1:4]))
        elif cmd in ("g", "o"):
            current_group = new_group(
                " ".join(parts[1:]) if len(parts) > 1 else "default", current_mat)
        elif cmd == "usemtl":
            current_mat = " ".join(parts[1:])
            if current_group is None or current_group["faces"]:
                current_group = new_group("auto", current_mat)
            else:
                current_group["material"] = current_mat
        elif cmd == "f":
            if current_group is None:
                current_group = new_group("default", current_mat)
            face = []
            for token in parts[1:]:
                refs = token.split("/")
                vi = int(refs[0]) - 1 if refs[0] else None
                ti = int(refs[1]) - 1 if len(refs) > 1 and refs[1] else None
                ni = int(refs[2]) - 1 if len(refs) > 2 and refs[2] else None
                face.append((vi, ti, ni))
            current_group["faces"].append(face)

    return raw_lines, vertices, texcoords, normals, groups, mtl_files, materials, base_dir


# ─────────────────────────────────────────────────────────────────────────────
# Texture loading
# ─────────────────────────────────────────────────────────────────────────────

def load_texture(path):
    if not path or not os.path.isfile(path):
        return None
    try:
        img = Image.open(path).convert("RGBA").transpose(Image.FLIP_TOP_BOTTOM)
        tex_id = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, tex_id)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
        gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA, img.width, img.height,
                          GL_RGBA, GL_UNSIGNED_BYTE, img.tobytes())
        return tex_id
    except Exception as e:
        print(f"Warning: could not load texture {path}: {e}")
        return None


# ─────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ─────────────────────────────────────────────────────────────────────────────

def compute_transformed_bounds(vertices, matrix):
    """Apply 4x4 matrix to vertices; return (center_xyz, dims_xyz) as ndarrays."""
    if not vertices:
        return np.zeros(3), np.ones(3)
    arr = np.array(vertices, dtype=np.float64)
    ones = np.ones((len(arr), 1))
    t = (matrix @ np.hstack([arr, ones]).T).T[:, :3]
    mn, mx = t.min(axis=0), t.max(axis=0)
    return (mn + mx) / 2.0, mx - mn


def triangulate(face):
    return [(face[0], face[i], face[i + 1]) for i in range(1, len(face) - 1)]


def face_normal(v0, v1, v2):
    n = np.cross(np.subtract(v1, v0), np.subtract(v2, v0))
    ln = np.linalg.norm(n)
    return (n / ln).tolist() if ln > 1e-9 else [0.0, 0.0, 1.0]


# ─────────────────────────────────────────────────────────────────────────────
# Accumulated transform (4×4 matrix)
# ─────────────────────────────────────────────────────────────────────────────

class Transform:
    def __init__(self):
        self.matrix = np.eye(4, dtype=np.float64)

    def reset(self):
        self.matrix = np.eye(4, dtype=np.float64)

    @staticmethod
    def _rot(axis, deg):
        a = math.radians(deg)
        c, s = math.cos(a), math.sin(a)
        if axis == "x":
            return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]], dtype=np.float64)
        elif axis == "y":
            return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]], dtype=np.float64)
        else:  # z
            return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]], dtype=np.float64)

    @staticmethod
    def _trans(dx, dy, dz):
        m = np.eye(4, dtype=np.float64); m[:3, 3] = [dx, dy, dz]; return m

    @staticmethod
    def _scale(sx, sy, sz):
        return np.diag([sx, sy, sz, 1.0]).astype(np.float64)

    def translate(self, dx, dy, dz):
        self.matrix = self._trans(dx, dy, dz) @ self.matrix

    def rotate(self, axis, degrees):
        self.matrix = self._rot(axis.lower(), degrees) @ self.matrix

    def scale_uniform(self, f):
        self.matrix = self._scale(f, f, f) @ self.matrix

    def scale_axis(self, sx, sy, sz):
        self.matrix = self._scale(sx, sy, sz) @ self.matrix

    def apply_to_gl(self):
        glMultMatrixd(self.matrix.T.flatten())


# ─────────────────────────────────────────────────────────────────────────────
# Save transformed OBJ
# ─────────────────────────────────────────────────────────────────────────────

def save_obj(out_path, raw_lines, original_vertices, transform_matrix):
    arr = np.array(original_vertices, dtype=np.float64)
    ones = np.ones((len(arr), 1))
    transformed = (transform_matrix @ np.hstack([arr, ones]).T).T[:, :3]
    v_index = 0
    out_lines = []
    for line in raw_lines:
        stripped = line.strip()
        if stripped and not stripped.startswith("#"):
            parts = stripped.split()
            if parts[0].lower() == "v" and len(parts) >= 4:
                x, y, z = transformed[v_index]
                out_lines.append(f"v {x:.6f} {y:.6f} {z:.6f}\n")
                v_index += 1
                continue
        out_lines.append(line)
    with open(out_path, "w", encoding="utf-8") as f:
        f.writelines(out_lines)
    print(f"\nSaved → {out_path}")


# ─────────────────────────────────────────────────────────────────────────────
# Bitmap text renderer  (pygame surface → OpenGL quad)
# ─────────────────────────────────────────────────────────────────────────────

class TextRenderer:
    FONT_SIZE = 17

    def __init__(self, win_w, win_h):
        self.win_w = win_w
        self.win_h = win_h
        self._font = pygame.font.SysFont("monospace", self.FONT_SIZE)
        self._cache = {}

    def _get_tex(self, text, color):
        key = (text, color)
        if key not in self._cache:
            surf = self._font.render(text, True, color)
            w, h = surf.get_size()
            data = pygame.image.tostring(surf, "RGBA", True)
            tid = glGenTextures(1)
            glBindTexture(GL_TEXTURE_2D, tid)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0,
                         GL_RGBA, GL_UNSIGNED_BYTE, data)
            self._cache[key] = (tid, w, h)
        return self._cache[key]

    def draw(self, text, x, y, color=(255, 255, 255)):
        """Draw text at pixel (x, y) measured from top-left."""
        tid, tw, th = self._get_tex(text, color)

        glMatrixMode(GL_PROJECTION)
        glPushMatrix(); glLoadIdentity()
        glOrtho(0, self.win_w, 0, self.win_h, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix(); glLoadIdentity()

        glDisable(GL_DEPTH_TEST); glDisable(GL_LIGHTING)
        glEnable(GL_TEXTURE_2D); glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glBindTexture(GL_TEXTURE_2D, tid)
        glColor4f(1, 1, 1, 1)

        yb = self.win_h - y - th          # flip to bottom-origin
        glBegin(GL_QUADS)
        glTexCoord2f(0,0); glVertex2f(x,      yb)
        glTexCoord2f(1,0); glVertex2f(x + tw, yb)
        glTexCoord2f(1,1); glVertex2f(x + tw, yb + th)
        glTexCoord2f(0,1); glVertex2f(x,      yb + th)
        glEnd()

        glDisable(GL_BLEND); glDisable(GL_TEXTURE_2D)
        glEnable(GL_LIGHTING); glEnable(GL_DEPTH_TEST)
        glPopMatrix()
        glMatrixMode(GL_PROJECTION); glPopMatrix()
        glMatrixMode(GL_MODELVIEW)


# ─────────────────────────────────────────────────────────────────────────────
# Non-blocking single-line input panel (drawn at the bottom of the window)
# ─────────────────────────────────────────────────────────────────────────────

class InputPanel:
    def __init__(self):
        self.active   = False
        self.prompt   = ""
        self.buffer   = ""
        self._cb      = None

    def start(self, prompt, callback):
        self.prompt = prompt
        self.buffer = ""
        self.active = True
        self._cb    = callback

    def feed(self, event):
        """Returns True if event was consumed."""
        if not self.active:
            return False
        if event.type != KEYDOWN:
            return True
        if event.key == K_ESCAPE:
            self.active = False
        elif event.key in (K_RETURN, K_KP_ENTER):
            val = self.buffer.strip()
            self.active = False
            if self._cb and val:
                self._cb(val)
        elif event.key == K_BACKSPACE:
            self.buffer = self.buffer[:-1]
        else:
            ch = event.unicode
            if ch and (ch.isdigit() or ch in "-+. " or ch.isalpha()):
                self.buffer += ch
        return True


# ─────────────────────────────────────────────────────────────────────────────
# Main viewer
# ─────────────────────────────────────────────────────────────────────────────

HELP = [
    "T – translate     (enter: dx dy dz)",
    "R – rotate        (enter: axis  degrees)   axis = x / y / z",
    "S – scale         (enter: factor  OR  sx sy sz)",
    "Z – zoom          (enter: factor   <1 closer  >1 farther)",
    "X – reset all transforms",
    "Click – measure   (click 2 points; measures on the X/Z plane at Y=0)",
    "M – clear measurement",
    "Ctrl+S – save transformed OBJ",
    "Q / Esc – quit",
]


class OBJViewer:
    WIN_W, WIN_H = 1200, 800

    def __init__(self, obj_path):
        self.obj_path  = os.path.abspath(obj_path)
        self.transform = Transform()
        self._status   = ""
        self._input    = InputPanel()

        # Measurement tool state  (world-space XZ points, Y fixed at 0)
        self._meas_pts   = []   # list of up to 2 np.array([x, y, z])
        self._meas_dist  = None # float or None

        print(f"Loading: {self.obj_path}")
        (self.raw_lines, self.vertices, self.texcoords, self.normals,
         self.groups, self.mtl_files, self.materials,
         self.base_dir) = parse_obj(self.obj_path)

        # Initial camera distance from mesh extent
        if self.vertices:
            arr = np.array(self.vertices)
            mn, mx = arr.min(axis=0), arr.max(axis=0)
            orig_center = (mn + mx) / 2.0
            extent = float(np.max(mx - mn)) or 1.0
        else:
            orig_center = np.zeros(3)
            extent = 1.0

        self._cam_look_at  = orig_center.copy()
        self._cam_dist     = extent * 2.0
        self._cam_dist_min = extent * 0.05
        self._cam_dist_max = extent * 50.0

        print(f"  Vertices : {len(self.vertices)}")
        print(f"  Groups   : {len(self.groups)}")
        print(f"  Materials: {list(self.materials.keys())}")

    # ── OpenGL init ───────────────────────────────────────────────────────────

    def _init_gl(self):
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glEnable(GL_NORMALIZE)
        glShadeModel(GL_SMOOTH)

        # Light coming from front-upper-right in world coords
        glLightfv(GL_LIGHT0, GL_POSITION, [1.0, -1.0, 2.0, 0.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT,  [0.35, 0.35, 0.35, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE,  [0.85, 0.85, 0.85, 1.0])
        glLightfv(GL_LIGHT0, GL_SPECULAR, [0.4,  0.4,  0.4,  1.0])

        glClearColor(0.15, 0.15, 0.18, 1.0)

        self._tex_ids = {}
        for name, mat in self.materials.items():
            tp = mat.get("map_Kd_path")
            if tp:
                tid = load_texture(tp)
                self._tex_ids[name] = tid
                if tid:
                    print(f"  Texture loaded: {os.path.basename(tp)}")

        self._dl = glGenLists(1)
        glNewList(self._dl, GL_COMPILE)
        self._draw_mesh()
        glEndList()

    def _draw_mesh(self):
        verts = self.vertices
        tcs   = self.texcoords
        norms = self.normals

        for group in self.groups:
            mat_name = group["material"]
            mat      = self.materials.get(mat_name, {})
            tex_id   = self._tex_ids.get(mat_name)

            Kd    = mat.get("Kd", (0.8, 0.8, 0.8))
            Ka    = mat.get("Ka", (0.2, 0.2, 0.2))
            Ks    = mat.get("Ks", (0.0, 0.0, 0.0))
            Ns    = mat.get("Ns", 0.0)
            alpha = mat.get("d",  1.0)

            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   list(Ka) + [alpha])
            glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   list(Kd) + [alpha])
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  list(Ks) + [alpha])
            glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, min(Ns, 128))
            glColor4f(*Kd, alpha)

            if tex_id:
                glEnable(GL_TEXTURE_2D)
                glBindTexture(GL_TEXTURE_2D, tex_id)
            else:
                glDisable(GL_TEXTURE_2D)

            for face in group["faces"]:
                tris = triangulate(face)
                glBegin(GL_TRIANGLES)
                for tri in tris:
                    v0 = verts[tri[0][0]]; v1 = verts[tri[1][0]]; v2 = verts[tri[2][0]]
                    fn = face_normal(v0, v1, v2)
                    for vi, ti, ni in tri:
                        glNormal3fv(norms[ni] if ni is not None and ni < len(norms) else fn)
                        if ti is not None and ti < len(tcs):
                            glTexCoord2fv(tcs[ti])
                        glVertex3fv(verts[vi])
                glEnd()

        glDisable(GL_TEXTURE_2D)

    # ── Fixed front camera  (Z-up, looking along +Y → −Y) ────────────────────

    def _setup_camera(self):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(40.0, self.WIN_W / self.WIN_H,
                       self._cam_dist * 0.001, self._cam_dist * 100)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # Re-set light position in world space (before camera transform)
        glLightfv(GL_LIGHT0, GL_POSITION, [0.6, -1.0, 1.5, 0.0])

        lx, ly, lz = self._cam_look_at
        gluLookAt(
            lx,  ly + self._cam_dist,  lz,   # eye   (far along +Y)
            lx,  ly,                   lz,   # look-at
            0,   0,                    1,    # up = +Z
        )

    def _recenter_camera(self):
        """Point the camera at the current transformed mesh center and adjust
        the camera distance so the whole mesh fits comfortably in view."""
        center, dims = compute_transformed_bounds(self.vertices, self.transform.matrix)
        self._cam_look_at = center.copy()
        extent = float(np.max(dims)) or 1.0
        self._cam_dist     = extent * 2.0
        self._cam_dist_min = extent * 0.05
        self._cam_dist_max = extent * 50.0

    # ── Render ────────────────────────────────────────────────────────────────

    def _screen_to_world(self, sx, sy):
        """
        Unproject a screen pixel into the world-space XZ plane (Y = look_at.Y).
        Returns np.array([x, y, z]) or None if unprojection fails.
        Camera looks along +Y so rays will almost always intersect Y = const.
        """
        viewport   = glGetIntegerv(GL_VIEWPORT)
        modelview  = glGetDoublev(GL_MODELVIEW_MATRIX)
        projection = glGetDoublev(GL_PROJECTION_MATRIX)

        # OpenGL y is flipped vs pygame
        win_y = viewport[3] - sy

        # Unproject at near (wz=0) and far (wz=1) planes
        try:
            nx, ny, nz = gluUnProject(sx, win_y, 0.0, modelview, projection, viewport)
            fx, fy, fz = gluUnProject(sx, win_y, 1.0, modelview, projection, viewport)
        except Exception:
            return None

        # Ray direction
        dx, dy, dz = fx - nx, fy - ny, fz - nz
        if abs(dy) < 1e-12:
            return None

        # Intersect with the Y = cam_look_at[1] plane
        target_y = self._cam_look_at[1]
        t = (target_y - ny) / dy
        return np.array([nx + t * dx, target_y, nz + t * dz])

    def _draw_3d_overlays(self):
        """Draw the origin marker and measurement markers/line in 3D world space."""
        glDisable(GL_LIGHTING)
        glDisable(GL_TEXTURE_2D)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        # ── Origin marker ──────────────────────────────────────────────────────
        # The mesh's local (0,0,0) transformed into world space
        origin_world = (self.transform.matrix @ np.array([0, 0, 0, 1]))[:3]
        ox, oy, oz = origin_world

        axis_len = self._cam_dist * 0.06
        glLineWidth(2.5)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.2, 0.2); glVertex3f(ox, oy, oz); glVertex3f(ox + axis_len, oy, oz)
        glColor3f(0.2, 1.0, 0.2); glVertex3f(ox, oy, oz); glVertex3f(ox, oy + axis_len, oz)
        glColor3f(0.3, 0.5, 1.0); glVertex3f(ox, oy, oz); glVertex3f(ox, oy, oz + axis_len)
        glEnd()
        glLineWidth(1.0)

        # Small white dot at the origin
        glPointSize(8.0)
        glBegin(GL_POINTS)
        glColor3f(1.0, 1.0, 1.0)
        glVertex3f(ox, oy, oz)
        glEnd()
        glPointSize(1.0)

        # ── Measurement markers & line ─────────────────────────────────────────
        if self._meas_pts:
            glPointSize(10.0)
            glBegin(GL_POINTS)
            glColor3f(1.0, 0.9, 0.0)
            for p in self._meas_pts:
                glVertex3f(*p)
            glEnd()
            glPointSize(1.0)

            # Cross-hair at each point
            ch = axis_len * 0.5
            glLineWidth(1.5)
            glBegin(GL_LINES)
            glColor3f(1.0, 0.9, 0.0)
            for p in self._meas_pts:
                px, py, pz = p
                glVertex3f(px - ch, py, pz); glVertex3f(px + ch, py, pz)
                glVertex3f(px, py, pz - ch); glVertex3f(px, py, pz + ch)
            glEnd()
            glLineWidth(1.0)

        if len(self._meas_pts) == 2:
            glLineWidth(1.5)
            glEnable(GL_LINE_STIPPLE)
            glLineStipple(1, 0xAAAA)
            glBegin(GL_LINES)
            glColor3f(1.0, 0.9, 0.0)
            glVertex3f(*self._meas_pts[0])
            glVertex3f(*self._meas_pts[1])
            glEnd()
            glDisable(GL_LINE_STIPPLE)
            glLineWidth(1.0)

        glDisable(GL_BLEND)
        glEnable(GL_LIGHTING)


    def _render(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self._setup_camera()

        glPushMatrix()
        self.transform.apply_to_gl()
        glCallList(self._dl)
        glPopMatrix()

        # Overlays are drawn in world space (no model transform)
        self._draw_3d_overlays()

        self._draw_hud()
        pygame.display.flip()

    # ── On-screen HUD ─────────────────────────────────────────────────────────

    def _draw_hud(self):
        tr  = self._text
        PAD = 12
        LH  = 21        # line height in pixels

        center, dims = compute_transformed_bounds(self.vertices, self.transform.matrix)
        cx, cy, cz   = center
        dx, dy, dz   = dims

        # Mesh local origin in world space
        origin_world = (self.transform.matrix @ np.array([0, 0, 0, 1]))[:3]
        ox, oy, oz   = origin_world

        y = PAD

        tr.draw(f"  {os.path.basename(self.obj_path)}", PAD, y, color=(220, 220, 100))
        y += LH + 2

        y += 4

        tr.draw(f"  Center :  X = {cx:+.4f}    Y = {cy:+.4f}    Z = {cz:+.4f}",
                PAD, y, color=(100, 210, 255))
        y += LH
        tr.draw(f"  Dims   :  X = {dx:.4f}    Y = {dy:.4f}    Z = {dz:.4f}",
                PAD, y, color=(100, 210, 255))
        y += LH
        tr.draw(f"  Origin :  X = {ox:+.4f}    Y = {oy:+.4f}    Z = {oz:+.4f}",
                PAD, y, color=(180, 140, 255))
        y += LH + 8

        # Transform matrix (top-left 3×4)
        tr.draw("  Current transform:", PAD, y, color=(180, 180, 180))
        y += LH
        m = self.transform.matrix
        for row in range(3):
            row_str = "    [ " + "   ".join(f"{m[row, col]:+8.4f}" for col in range(4)) + " ]"
            tr.draw(row_str, PAD, y, color=(140, 200, 140))
            y += LH

        y += 8

        # Measurement readout
        if self._meas_pts or self._meas_dist is not None:
            tr.draw("  ── Measure ──", PAD, y, color=(255, 230, 60))
            y += LH
            for i, p in enumerate(self._meas_pts):
                tr.draw(f"    P{i+1}:  X={p[0]:+.4f}  Y={p[1]:+.4f}  Z={p[2]:+.4f}",
                        PAD, y, color=(255, 220, 80))
                y += LH
            if self._meas_dist is not None:
                tr.draw(f"    Dist = {self._meas_dist:.4f}", PAD, y, color=(255, 255, 120))
                y += LH
            elif len(self._meas_pts) == 1:
                tr.draw("    Click a second point to complete measurement",
                        PAD, y, color=(200, 200, 100))
                y += LH
            y += 4

        # Last action status
        if self._status:
            tr.draw(f"  ► {self._status}", PAD, y, color=(255, 210, 80))
            y += LH + 4

        # Help
        tr.draw("  ── Keys ──", PAD, y, color=(160, 160, 160))
        y += LH
        for line in HELP:
            tr.draw(f"    {line}", PAD, y, color=(140, 140, 140))
            y += LH

        # Input bar at the bottom
        if self._input.active:
            bar_y = self.WIN_H - 36
            tr.draw(f"  ▶  {self._input.prompt}  {self._input.buffer}_",
                    0, bar_y, color=(255, 255, 60))

    # ── Save ─────────────────────────────────────────────────────────────────

    def _save(self):
        base, ext = os.path.splitext(self.obj_path)
        out_path  = base + "_transformed" + ext
        save_obj(out_path, self.raw_lines, self.vertices, self.transform.matrix)
        out_dir = os.path.dirname(out_path)
        for mtl_name in self.mtl_files:
            src = os.path.join(self.base_dir, mtl_name)
            dst = os.path.join(out_dir, mtl_name)
            if os.path.isfile(src) and src != dst:
                shutil.copy2(src, dst)
        self._status = f"Saved → {os.path.basename(out_path)}"

    # ── Input callbacks ───────────────────────────────────────────────────────

    def _start_translate(self):
        self._input.start("Translate  dx dy dz :", self._do_translate)

    def _do_translate(self, text):
        try:
            vals = [float(v) for v in text.split()]
            if len(vals) == 3:
                self.transform.translate(*vals)
                self._recenter_camera()
                self._status = (f"Translated  dx={vals[0]:+.4f}  "
                                f"dy={vals[1]:+.4f}  dz={vals[2]:+.4f}")
            else:
                self._status = "Need 3 values: dx dy dz"
        except ValueError:
            self._status = "Invalid translate input"

    def _start_rotate(self):
        self._input.start("Rotate  axis(x/y/z)  degrees :", self._do_rotate)

    def _do_rotate(self, text):
        try:
            parts = text.split()
            if len(parts) == 2 and parts[0].lower() in ("x", "y", "z"):
                axis = parts[0].lower()
                deg  = float(parts[1])
                self.transform.rotate(axis, deg)
                self._recenter_camera()
                self._status = f"Rotated {axis.upper()} by {deg:+.2f}°"
            else:
                self._status = "Need: axis degrees   e.g.  z 45"
        except ValueError:
            self._status = "Invalid rotate input"

    def _start_scale(self):
        self._input.start("Scale  factor  [or sx sy sz] :", self._do_scale)

    def _do_scale(self, text):
        try:
            vals = [float(v) for v in text.split()]
            if len(vals) == 1:
                self.transform.scale_uniform(vals[0])
                self._recenter_camera()
                self._status = f"Scaled uniformly by {vals[0]:.4f}"
            elif len(vals) == 3:
                self.transform.scale_axis(*vals)
                self._recenter_camera()
                self._status = (f"Scaled  sx={vals[0]:.4f}  "
                                f"sy={vals[1]:.4f}  sz={vals[2]:.4f}")
            else:
                self._status = "Need 1 value (uniform) or 3 (sx sy sz)"
        except ValueError:
            self._status = "Invalid scale input"

    def _start_zoom(self):
        self._input.start("Zoom  factor  (<1 closer  >1 farther) :", self._do_zoom)

    def _do_zoom(self, text):
        try:
            f = float(text.strip())
            if f <= 0:
                self._status = "Zoom factor must be > 0"
                return
            self._cam_dist = max(self._cam_dist_min,
                                 min(self._cam_dist_max, self._cam_dist * f))
            self._status = f"Camera distance → {self._cam_dist:.4f}"
        except ValueError:
            self._status = "Invalid zoom factor"

    # ── Event loop ────────────────────────────────────────────────────────────

    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == QUIT:
                return False
            if self._input.feed(event):   # panel active — swallow
                continue

            # Mouse click → measurement point
            if event.type == MOUSEBUTTONDOWN and event.button == 1:
                if not self._input.active:
                    pt = self._screen_to_world(event.pos[0], event.pos[1])
                    if pt is not None:
                        if len(self._meas_pts) >= 2:
                            # Start a new measurement
                            self._meas_pts  = [pt]
                            self._meas_dist = None
                        else:
                            self._meas_pts.append(pt)
                            if len(self._meas_pts) == 2:
                                d = np.linalg.norm(self._meas_pts[1] - self._meas_pts[0])
                                self._meas_dist = float(d)
                continue

            if event.type != KEYDOWN:
                continue
            k    = event.key
            mods = pygame.key.get_mods()

            if k in (K_ESCAPE, K_q):
                return False
            elif k == K_s and (mods & KMOD_CTRL):
                self._save()
            elif k == K_t:
                self._start_translate()
            elif k == K_r:
                self._start_rotate()
            elif k == K_s:
                self._start_scale()
            elif k == K_z:
                self._start_zoom()
            elif k == K_m:
                self._meas_pts  = []
                self._meas_dist = None
                self._status = "Measurement cleared"
            elif k == K_x:
                self.transform.reset()
                self._recenter_camera()
                self._status = "Transform reset to identity"
        return True

    # ── Run ───────────────────────────────────────────────────────────────────

    def run(self):
        pygame.init()
        pygame.display.set_mode((self.WIN_W, self.WIN_H), DOUBLEBUF | OPENGL)
        pygame.display.set_caption(f"OBJ Viewer – {os.path.basename(self.obj_path)}")
        self._text = TextRenderer(self.WIN_W, self.WIN_H)
        self._init_gl()

        print("\n" + "─" * 60)
        print("  CONTROLS  (focus the viewer window first)")
        print("─" * 60)
        for h in HELP:
            print(f"  {h}")
        print("─" * 60 + "\n")

        clock = pygame.time.Clock()
        running = True
        while running:
            running = self._handle_events()
            self._render()
            clock.tick(60)

        pygame.quit()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print("Usage: python obj_viewer.py <model.obj>")
        sys.exit(1)
    obj_path = sys.argv[1]
    if not os.path.isfile(obj_path):
        print(f"Error: file not found: {obj_path}")
        sys.exit(1)
    OBJViewer(obj_path).run()


if __name__ == "__main__":
    main()
