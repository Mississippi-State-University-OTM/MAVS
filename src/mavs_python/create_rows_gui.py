"""
create_rows_gui.py
A polished Tkinter GUI for configuring and running create_scene_with_rows.
Supports saving/loading configurations as JSON files.
"""

import json
import os
import subprocess
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
#import MAVS install info
import mavs_interface as mavs
import mavs_python_paths
mavs_data_path = mavs_python_paths.mavs_data_path
# ─────────────────────────────────────────────────────────────────────────────
# Default values mirrored from create_rows.py
# ─────────────────────────────────────────────────────────────────────────────
DEFAULTS = {
    "SCENE_FILE":           mavs_data_path+"/scenes/empty_field.json",  #"/home/kodai/mavs_latest/data/scenes/aai_field.json",
    "MESH_DIR":             mavs_data_path+"/scenes/meshes/vegetation/lemon_tree/", #"/home/kodai/mavs_latest/data/scenes/meshes/cotton/objective1_related/colmapOpenMVS",
    "MESHES_ROOT":          mavs_data_path+"/scenes/meshes",
    "OUTPUT_FILE":          mavs_data_path+"/scenes/row_scene.json",
    "FIRST_ROW_START_X":    -43.457,
    "FIRST_ROW_START_Y":    167.19,
    "FIRST_ROW_END_X":      -54.948,
    "FIRST_ROW_END_Y":      49.131,
    "PLANT_SPACING":        5.0,
    "ROW_SPACING":          8.0,
    "NUM_ROWS":             5,
    "ROW_SIDE":             1,
    "SCALE_X":              1.0,
    "SCALE_Y":              1.0,
    "SCALE_Z":              1.0,
    "Z_VALUE":              0.0,
    "POSITION_NOISE_STD":   0.25,
    "LABEL_BY_GROUP":       True,
    "SEED":                 7,
}

# ─────────────────────────────────────────────────────────────────────────────
# Colour palette
# ─────────────────────────────────────────────────────────────────────────────
C = {
    "bg":          "#1A1D23",
    "panel":       "#22262E",
    "border":      "#2E3440",
    "accent":      "#C1C6C8",      # MSU light gray
    "accent_dk":   "#3A8F68",
    "warn":        "#E07B54",
    "text":        "#E8EAF0",
    "text_dim":    "#7A8399",
    "entry_bg":    "#2A2F3A",
    "entry_fg":    "#E8EAF0",
    "btn_run_bg":  "#5D1725", #"#4CAF82",
    "btn_run_fg":  "#C1C6C8", #"#0F1115",
    "header_bg":   "#181B20",
}

FONT_HEADING  = ("Consolas", 11, "bold")
FONT_LABEL    = ("Consolas", 9)
FONT_ENTRY    = ("Consolas", 9)
FONT_BTN      = ("Consolas", 10, "bold")
FONT_STATUS   = ("Consolas", 8)
FONT_TITLE    = ("Consolas", 14, "bold")
FONT_SUBTITLE = ("Consolas", 8)


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _styled_entry(parent, textvariable, width=38):
    e = tk.Entry(
        parent,
        textvariable=textvariable,
        width=width,
        bg=C["entry_bg"],
        fg=C["entry_fg"],
        insertbackground=C["accent"],
        relief="flat",
        bd=0,
        font=FONT_ENTRY,
        highlightthickness=1,
        highlightbackground=C["border"],
        highlightcolor=C["accent"],
    )
    return e


def _section_header(parent, text):
    frm = tk.Frame(parent, bg=C["header_bg"], pady=4)
    frm.pack(fill="x", pady=(12, 4))
    tk.Label(
        frm,
        text=f"  {text}",
        bg=C["header_bg"],
        fg=C["accent"],
        font=FONT_HEADING,
        anchor="w",
    ).pack(side="left")
    return frm


def _row(parent, label_text, widget_builder, hint=""):
    frm = tk.Frame(parent, bg=C["panel"], pady=3)
    frm.pack(fill="x", padx=16)
    lbl = tk.Label(frm, text=label_text, bg=C["panel"], fg=C["text"],
                   font=FONT_LABEL, width=22, anchor="w")
    lbl.pack(side="left")
    w = widget_builder(frm)
    w.pack(side="left", padx=(4, 0))
    if hint:
        tk.Label(frm, text=hint, bg=C["panel"], fg=C["text_dim"],
                 font=FONT_STATUS).pack(side="left", padx=(6, 0))
    return frm


def _browse_row(parent, label_text, var, file_mode="open", filetypes=None):
    """A path entry with a Browse button."""
    frm = tk.Frame(parent, bg=C["panel"], pady=3)
    frm.pack(fill="x", padx=16)
    tk.Label(frm, text=label_text, bg=C["panel"], fg=C["text"],
             font=FONT_LABEL, width=22, anchor="w").pack(side="left")
    e = _styled_entry(frm, var, width=34)
    e.pack(side="left", padx=(4, 0))

    def browse():
        if file_mode == "dir":
            path = filedialog.askdirectory()
        elif file_mode == "save":
            path = filedialog.asksaveasfilename(
                filetypes=filetypes or [("JSON", "*.json"), ("All", "*.*")])
        else:
            path = filedialog.askopenfilename(
                filetypes=filetypes or [("JSON", "*.json"), ("All", "*.*")])
        if path:
            var.set(path)

    btn = tk.Button(
        frm, text="…", command=browse,
        bg=C["border"], fg=C["text"], activebackground=C["accent"],
        activeforeground=C["btn_run_fg"], relief="flat",
        font=FONT_BTN, cursor="hand2", padx=6,
    )
    btn.pack(side="left", padx=(4, 0))
    return frm


def _pair_row(parent, label_text, varA, varB, labA="X", labB="Y", hint=""):
    frm = tk.Frame(parent, bg=C["panel"], pady=3)
    frm.pack(fill="x", padx=16)
    tk.Label(frm, text=label_text, bg=C["panel"], fg=C["text"],
             font=FONT_LABEL, width=22, anchor="w").pack(side="left")
    for var, lab in ((varA, labA), (varB, labB)):
        tk.Label(frm, text=lab, bg=C["panel"], fg=C["text_dim"],
                 font=FONT_LABEL).pack(side="left", padx=(6, 2))
        _styled_entry(frm, var, width=10).pack(side="left")
    if hint:
        tk.Label(frm, text=hint, bg=C["panel"], fg=C["text_dim"],
                 font=FONT_STATUS).pack(side="left", padx=(6, 0))


# ─────────────────────────────────────────────────────────────────────────────
# Main application
# ─────────────────────────────────────────────────────────────────────────────

class CreateRowsGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("MAVS · Row Scene Builder")
        self.root.configure(bg=C["bg"])
        self.root.resizable(True, True)
        self.root.minsize(680, 740)

        self._build_vars()
        self._build_ui()
        self._load_defaults()

    # ── Variable initialisation ───────────────────────────────────────────────

    def _build_vars(self):
        self.v = {
            "SCENE_FILE":          tk.StringVar(),
            "MESH_DIR":            tk.StringVar(),
            "MESHES_ROOT":         tk.StringVar(),
            "OUTPUT_FILE":         tk.StringVar(),
            "FIRST_ROW_START_X":   tk.StringVar(),
            "FIRST_ROW_START_Y":   tk.StringVar(),
            "FIRST_ROW_END_X":     tk.StringVar(),
            "FIRST_ROW_END_Y":     tk.StringVar(),
            "PLANT_SPACING":       tk.StringVar(),
            "ROW_SPACING":         tk.StringVar(),
            "NUM_ROWS":            tk.StringVar(),
            "ROW_SIDE":            tk.StringVar(),
            "SCALE_X":             tk.StringVar(),
            "SCALE_Y":             tk.StringVar(),
            "SCALE_Z":             tk.StringVar(),
            "Z_VALUE":             tk.StringVar(),
            "POSITION_NOISE_STD":  tk.StringVar(),
            "LABEL_BY_GROUP":      tk.BooleanVar(),
            "SEED":                tk.StringVar(),
        }

    def _load_defaults(self):
        for k, val in DEFAULTS.items():
            if k in self.v:
                self.v[k].set(val)

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        # ── Title bar ────────────────────────────────────────────────────────
        title_bar = tk.Frame(self.root, bg=C["header_bg"], pady=12)
        title_bar.pack(fill="x")
        tk.Label(title_bar, text="  ⬡  ROW SCENE BUILDER",
                 bg=C["header_bg"], fg=C["accent"], font=FONT_TITLE).pack(side="left")
        tk.Label(title_bar, text="MAVS Agricultural Simulation  ",
                 bg=C["header_bg"], fg=C["text_dim"], font=FONT_SUBTITLE).pack(side="right")

        # ── Toolbar (Save / Load config) ──────────────────────────────────────
        toolbar = tk.Frame(self.root, bg=C["bg"], pady=6, padx=12)
        toolbar.pack(fill="x")
        for (txt, cmd) in [
            ("⬆  Save Config", self.save_config),
            ("⬇  Load Config", self.load_config),
            ("↺  Reset to Defaults", self._load_defaults),
        ]:
            tk.Button(
                toolbar, text=txt, command=cmd,
                bg=C["border"], fg=C["text"], activebackground=C["accent"],
                activeforeground=C["btn_run_fg"], relief="flat",
                font=FONT_BTN, cursor="hand2", padx=10, pady=4,
            ).pack(side="left", padx=(0, 6))

        # ── Scrollable main area ──────────────────────────────────────────────
        outer = tk.Frame(self.root, bg=C["bg"])
        outer.pack(fill="both", expand=True, padx=8, pady=(0, 4))

        canvas = tk.Canvas(outer, bg=C["bg"], highlightthickness=0)
        scrollbar = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        self.scroll_frame = tk.Frame(canvas, bg=C["panel"],
                                     relief="flat", bd=0)
        self.scroll_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        canvas_window = canvas.create_window(
            (0, 0), window=self.scroll_frame, anchor="nw")

        # Make canvas resize with window
        def on_canvas_resize(event):
            canvas.itemconfig(canvas_window, width=event.width)
        canvas.bind("<Configure>", on_canvas_resize)

        # Mouse-wheel scroll
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        canvas.bind_all("<MouseWheel>", _on_mousewheel)

        self._build_form(self.scroll_frame)

        # ── Run button + status bar ───────────────────────────────────────────
        bottom = tk.Frame(self.root, bg=C["bg"], pady=8, padx=12)
        bottom.pack(fill="x", side="bottom")

        self.status_var = tk.StringVar(value="Ready.")
        tk.Label(bottom, textvariable=self.status_var, bg=C["bg"],
                 fg=C["text_dim"], font=FONT_STATUS, anchor="w").pack(side="left")

        self.run_btn = tk.Button(
            bottom, text="▶  RUN SCENE BUILDER", command=self.run_main,
            bg=C["btn_run_bg"], fg=C["btn_run_fg"],
            activebackground=C["accent_dk"], activeforeground=C["btn_run_fg"],
            relief="flat", font=FONT_BTN, cursor="hand2",
            padx=18, pady=8,
        )
        self.run_btn.pack(side="right")

    def _build_form(self, parent):
        # ── File Paths ────────────────────────────────────────────────────────
        _section_header(parent, "FILE PATHS")
        _browse_row(parent, "Scene File", self.v["SCENE_FILE"],
                    filetypes=[("JSON", "*.json"), ("All", "*.*")])
        _browse_row(parent, "Mesh Directory", self.v["MESH_DIR"], file_mode="dir")
        _browse_row(parent, "Meshes Root", self.v["MESHES_ROOT"], file_mode="dir")
        _browse_row(parent, "Output File", self.v["OUTPUT_FILE"], file_mode="save",
                    filetypes=[("JSON", "*.json"), ("All", "*.*")])

        # ── Row Geometry ──────────────────────────────────────────────────────
        _section_header(parent, "ROW GEOMETRY")
        _pair_row(parent, "First Row Start", self.v["FIRST_ROW_START_X"],
                  self.v["FIRST_ROW_START_Y"], hint="metres")
        _pair_row(parent, "First Row End", self.v["FIRST_ROW_END_X"],
                  self.v["FIRST_ROW_END_Y"], hint="metres")

        _row(parent, "Plant Spacing",
             lambda p: _styled_entry(p, self.v["PLANT_SPACING"], 10),
             hint="metres between plants")
        _row(parent, "Row Spacing",
             lambda p: _styled_entry(p, self.v["ROW_SPACING"], 10),
             hint="metres between rows")
        _row(parent, "Number of Rows",
             lambda p: _styled_entry(p, self.v["NUM_ROWS"], 10))
        _row(parent, "Row Side",
             lambda p: ttk.Combobox(
                 p, textvariable=self.v["ROW_SIDE"],
                 values=["1", "-1"], width=8, state="readonly",
                 font=FONT_ENTRY),
             hint="1 or −1  (flip side of first row)")

        # ── Mesh Transform ────────────────────────────────────────────────────
        _section_header(parent, "MESH TRANSFORM")
        _pair_row(parent, "Scale (X / Y / Z)",
                  self.v["SCALE_X"], self.v["SCALE_Y"],
                  labA="X", labB="Y")
        # third scale component on same visual row
        frm_z = tk.Frame(parent, bg=C["panel"], pady=0)
        frm_z.pack(fill="x", padx=16)
        tk.Label(frm_z, text="", bg=C["panel"], width=22).pack(side="left")
        tk.Label(frm_z, text="Z", bg=C["panel"], fg=C["text_dim"],
                 font=FONT_LABEL).pack(side="left", padx=(6, 2))
        _styled_entry(frm_z, self.v["SCALE_Z"], width=10).pack(side="left")

        _row(parent, "Z Value",
             lambda p: _styled_entry(p, self.v["Z_VALUE"], 10),
             hint="global Z offset for all meshes")

        # ── Randomisation ─────────────────────────────────────────────────────
        _section_header(parent, "RANDOMISATION")
        _row(parent, "Position Noise STD",
             lambda p: _styled_entry(p, self.v["POSITION_NOISE_STD"], 10),
             hint="σ of Gaussian XY jitter")
        _row(parent, "Random Seed",
             lambda p: _styled_entry(p, self.v["SEED"], 10),
             hint="controls mesh selection & noise")

        # ── Labelling ─────────────────────────────────────────────────────────
        _section_header(parent, "LABELLING")

        def _chk(parent):
            chk = tk.Checkbutton(
                parent, variable=self.v["LABEL_BY_GROUP"],
                text="Label by group",
                bg=C["panel"], fg=C["text"], selectcolor=C["entry_bg"],
                activebackground=C["panel"], activeforeground=C["accent"],
                font=FONT_LABEL, cursor="hand2",
            )
            return chk

        _row(parent, "Label by Group", _chk)

        # bottom padding
        tk.Frame(parent, bg=C["panel"], height=16).pack()

    # ── Config serialisation ──────────────────────────────────────────────────

    def _collect(self) -> dict:
        """Read all widget vars into a plain dict (with type coercion)."""
        try:
            return {
                "SCENE_FILE":           self.v["SCENE_FILE"].get(),
                "MESH_DIR":             self.v["MESH_DIR"].get(),
                "MESHES_ROOT":          self.v["MESHES_ROOT"].get(),
                "OUTPUT_FILE":          self.v["OUTPUT_FILE"].get(),
                "FIRST_ROW_START":      [float(self.v["FIRST_ROW_START_X"].get()),
                                         float(self.v["FIRST_ROW_START_Y"].get())],
                "FIRST_ROW_END":        [float(self.v["FIRST_ROW_END_X"].get()),
                                         float(self.v["FIRST_ROW_END_Y"].get())],
                "PLANT_SPACING":        float(self.v["PLANT_SPACING"].get()),
                "ROW_SPACING":          float(self.v["ROW_SPACING"].get()),
                "NUM_ROWS":             int(self.v["NUM_ROWS"].get()),
                "ROW_SIDE":             int(self.v["ROW_SIDE"].get()),
                "SCALE":                [float(self.v["SCALE_X"].get()),
                                         float(self.v["SCALE_Y"].get()),
                                         float(self.v["SCALE_Z"].get())],
                "Z_VALUE":              float(self.v["Z_VALUE"].get()),
                "POSITION_NOISE_STD":   float(self.v["POSITION_NOISE_STD"].get()),
                "LABEL_BY_GROUP":       bool(self.v["LABEL_BY_GROUP"].get()),
                "SEED":                 int(self.v["SEED"].get()),
            }
        except ValueError as exc:
            messagebox.showerror("Validation Error", f"Invalid value: {exc}")
            return None

    def _populate(self, cfg: dict):
        """Push a config dict back into the widget vars."""
        str_keys = ["SCENE_FILE", "MESH_DIR", "MESHES_ROOT", "OUTPUT_FILE"]
        for k in str_keys:
            if k in cfg:
                self.v[k].set(cfg[k])

        if "FIRST_ROW_START" in cfg:
            self.v["FIRST_ROW_START_X"].set(cfg["FIRST_ROW_START"][0])
            self.v["FIRST_ROW_START_Y"].set(cfg["FIRST_ROW_START"][1])
        if "FIRST_ROW_END" in cfg:
            self.v["FIRST_ROW_END_X"].set(cfg["FIRST_ROW_END"][0])
            self.v["FIRST_ROW_END_Y"].set(cfg["FIRST_ROW_END"][1])
        if "SCALE" in cfg:
            self.v["SCALE_X"].set(cfg["SCALE"][0])
            self.v["SCALE_Y"].set(cfg["SCALE"][1])
            self.v["SCALE_Z"].set(cfg["SCALE"][2])

        simple_map = {
            "PLANT_SPACING": "PLANT_SPACING",
            "ROW_SPACING":   "ROW_SPACING",
            "NUM_ROWS":      "NUM_ROWS",
            "ROW_SIDE":      "ROW_SIDE",
            "Z_VALUE":       "Z_VALUE",
            "POSITION_NOISE_STD": "POSITION_NOISE_STD",
            "SEED":          "SEED",
        }
        for cfg_key, var_key in simple_map.items():
            if cfg_key in cfg:
                self.v[var_key].set(cfg[cfg_key])

        if "LABEL_BY_GROUP" in cfg:
            self.v["LABEL_BY_GROUP"].set(cfg["LABEL_BY_GROUP"])

    def save_config(self):
        cfg = self._collect()
        if cfg is None:
            return
        path = filedialog.asksaveasfilename(
            title="Save Configuration",
            defaultextension=".json",
            filetypes=[("JSON config", "*.json"), ("All files", "*.*")],
        )
        if not path:
            return
        with open(path, "w") as f:
            json.dump(cfg, f, indent=2)
        self.status_var.set(f"Config saved → {os.path.basename(path)}")

    def load_config(self):
        path = filedialog.askopenfilename(
            title="Load Configuration",
            filetypes=[("JSON config", "*.json"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            with open(path) as f:
                cfg = json.load(f)
            self._populate(cfg)
            self.status_var.set(f"Config loaded ← {os.path.basename(path)}")
        except Exception as exc:
            messagebox.showerror("Load Error", str(exc))

    # ── Runner ────────────────────────────────────────────────────────────────

    def run_main(self):
        cfg = self._collect()
        if cfg is None:
            return

        self.run_btn.config(state="disabled", text="⏳  Running…")
        self.status_var.set("Running scene builder …")
        self.root.update_idletasks()

        # Build a tiny ephemeral script that imports the real function and
        # calls it with the collected parameters, so no subprocess quoting
        # headaches and no dependency on this GUI file at runtime.
        script = f"""
import sys
sys.path.insert(0, {repr(os.path.dirname(os.path.abspath(__file__)))})
from create_rows.row_placement import create_scene_with_rows

create_scene_with_rows(
    scene_file={cfg['SCENE_FILE']!r},
    mesh_dir={cfg['MESH_DIR']!r},
    output_file={cfg['OUTPUT_FILE']!r},
    first_row_start={cfg['FIRST_ROW_START']!r},
    first_row_end={cfg['FIRST_ROW_END']!r},
    plant_spacing={cfg['PLANT_SPACING']!r},
    row_spacing={cfg['ROW_SPACING']!r},
    num_rows={cfg['NUM_ROWS']!r},
    scale={cfg['SCALE']!r},
    meshes_root={cfg['MESHES_ROOT']!r},
    row_side={cfg['ROW_SIDE']!r},
    position_noise_std={cfg['POSITION_NOISE_STD']!r},
    z_value={cfg['Z_VALUE']!r},
    label_by_group={cfg['LABEL_BY_GROUP']!r},
    seed={cfg['SEED']!r},
)
print("Scene written to: {cfg['OUTPUT_FILE']}")
"""

        try:
            result = subprocess.run(
                [sys.executable, "-c", script],
                capture_output=True, text=True, timeout=300,
            )
            if result.returncode == 0:
                msg = result.stdout.strip() or "Done."
                self.status_var.set(f"✓ {msg}")
                messagebox.showinfo("Success", f"Scene builder finished.\n\n{msg}")
            else:
                err = result.stderr.strip()
                self.status_var.set("✗ Error — see dialog.")
                messagebox.showerror("Runtime Error", err or "Unknown error.")
        except subprocess.TimeoutExpired:
            self.status_var.set("✗ Timed out after 5 min.")
            messagebox.showerror("Timeout", "The scene builder exceeded 5 minutes.")
        except Exception as exc:
            self.status_var.set(f"✗ {exc}")
            messagebox.showerror("Error", str(exc))
        finally:
            self.run_btn.config(state="normal", text="▶  RUN SCENE BUILDER")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    root = tk.Tk()

    # Apply ttk theme overrides for the scrollbar
    style = ttk.Style(root)
    try:
        style.theme_use("clam")
    except Exception:
        pass
    style.configure("Vertical.TScrollbar",
                    gripcount=0,
                    background=C["border"],
                    troughcolor=C["bg"],
                    bordercolor=C["bg"],
                    arrowcolor=C["text_dim"],
                    relief="flat")

    app = CreateRowsGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
