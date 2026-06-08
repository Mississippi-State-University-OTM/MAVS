#!/usr/bin/env python3
"""Launch a MAVS camera preview for a scene-builder scene."""

from __future__ import annotations

import ctypes as _ctypes
import json
import math
import os
import sys
import time
import traceback
from pathlib import Path

# ---------------------------------------------------------------------------
# Platform key-state polling (Windows GetAsyncKeyState; no-op elsewhere)
# ---------------------------------------------------------------------------
try:
    _user32 = _ctypes.windll.user32  # type: ignore[attr-defined]

    def _key(vk: int) -> bool:
        return bool(_user32.GetAsyncKeyState(vk) & 0x8000)
except AttributeError:
    def _key(vk: int) -> bool:  # type: ignore[misc]
        return False

_VK_Q = 0x51   # move up
_VK_E = 0x45   # move down
_VK_R = 0x52   # pitch up (look up)
_VK_F = 0x46   # pitch down (look down)


ROOT = Path(__file__).resolve().parents[2]
MAVS_PYTHON = ROOT / "src" / "mavs_python"
INSTALL_BIN = ROOT / "install" / "bin"
INSTALL_LIB = ROOT / "install" / "lib"
DATA_ROOT = ROOT / "data"

sys.path.append(str(MAVS_PYTHON))

if hasattr(os, "add_dll_directory"):
    os.add_dll_directory(str(INSTALL_BIN))
    os.add_dll_directory(str(INSTALL_LIB))

import mavs_interface as mavs  # noqa: E402


def log(message: str) -> None:
    print(message, flush=True)


def die(stage: str, exc: Exception | None = None) -> None:
    print()
    print("=" * 80)
    print("FAILED AT:", stage)
    if exc is not None:
        print("Exception:", repr(exc))
        traceback.print_exc()
    print("=" * 80)
    raise SystemExit(1)


def pose_quaternion(yaw: float, pitch: float) -> list[float]:
    """[w, x, y, z] quaternion for a ZYX yaw-then-pitch rotation."""
    hy, hp = 0.5 * yaw, 0.5 * pitch
    cy, sy = math.cos(hy), math.sin(hy)
    cp, sp = math.cos(hp), math.sin(hp)
    return [cy * cp, -sy * sp, cy * sp, sy * cp]


def get_command_values(cmd) -> tuple[float, float, float]:
    throttle = 0.0
    steering = 0.0
    braking = 0.0

    if hasattr(cmd, "throttle"):
        throttle = float(cmd.throttle)
    elif hasattr(cmd, "Throttle"):
        throttle = float(cmd.Throttle)

    if hasattr(cmd, "steering"):
        steering = float(cmd.steering)
    elif hasattr(cmd, "Steering"):
        steering = float(cmd.Steering)

    if hasattr(cmd, "braking"):
        braking = float(cmd.braking)
    elif hasattr(cmd, "Braking"):
        braking = float(cmd.Braking)

    if isinstance(cmd, (list, tuple)) and len(cmd) >= 3:
        throttle = float(cmd[0])
        steering = float(cmd[1])
        braking = float(cmd[2])

    return throttle, steering, braking


def scene_instance_positions(scene_path: Path) -> list[list[float]]:
    scene = json.loads(scene_path.read_text(encoding="utf-8"))
    positions: list[list[float]] = []
    for scene_object in scene.get("Objects", []):
        mesh = str(scene_object.get("Mesh", ""))
        if mesh == "surfaces/scene_builder_plane.obj":
            continue
        for instance in scene_object.get("Instances", []):
            position = instance.get("Position", [0.0, 0.0, 0.0])
            if len(position) == 3:
                positions.append([float(position[0]), float(position[1]), float(position[2])])
        random_spec = scene_object.get("Random", {})
        offset = random_spec.get("Offset", [0.0, 0.0, 0.0])
        offset_z = float(offset[2]) if len(offset) == 3 else 0.0
        for point in random_spec.get("Polygon", []):
            if len(point) == 2:
                positions.append([float(point[0]), float(point[1]), offset_z])
    return positions


def auto_camera_pose(scene_path: Path) -> tuple[list[float], float]:
    positions = scene_instance_positions(scene_path)
    if not positions:
        return [-35.0, 0.0, 6.0], 0.0

    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    zs = [p[2] for p in positions]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    max_z = max(zs)
    center_x = 0.5 * (min_x + max_x)
    center_y = 0.5 * (min_y + max_y)
    span_x = max(1.0, max_x - min_x)
    span_y = max(1.0, max_y - min_y)
    distance = max(18.0, 1.15 * math.hypot(span_x, span_y))
    height = max(3.0, max_z + 0.25 * distance)
    return [center_x - distance, center_y, height], 0.0


def main() -> None:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("scene", help="Path to scene JSON file")
    ap.add_argument("--x",       type=float, default=None)
    ap.add_argument("--y",       type=float, default=None)
    ap.add_argument("--z",       type=float, default=None)
    ap.add_argument("--heading", type=float, default=None)
    ap.add_argument("--pitch",   type=float, default=0.0)
    args = ap.parse_args()

    scene_path = Path(args.scene).resolve()
    preview_image_out = ROOT / "data" / "scenes" / "scene_builder_preview.bmp"
    image_width = 960
    image_height = 540
    dt = 1.0 / 30.0
    move_speed = 10.0
    turn_speed = 1.2
    pitch_speed = 0.9

    os.chdir(DATA_ROOT)
    log("MAVS scene-builder preview")
    log("--------------------------")
    log(f"Scene: {scene_path}")
    log(f"Working directory: {Path.cwd()}")

    try:
        scene = mavs.MavsEmbreeScene()
        env = mavs.MavsEnvironment()
        scene.Load(str(scene_path))
        env.SetScene(scene)
        env.SetTime(8)
        env.SetFog(0.0)
        env.SetSnow(0.0)
        env.SetTurbidity(7.0)
        env.SetAlbedo(0.1)
        env.SetCloudCover(0.1)
        env.SetRainRate(0.0)
        env.SetWind([0.0, 0.0])
    except Exception as exc:
        die("scene/environment setup", exc)

    if all(v is not None for v in [args.x, args.y, args.z, args.heading]):
        x, y, z = args.x, args.y, args.z
        heading = args.heading
        pitch = args.pitch
        log(f"Camera: pos=({x:.2f}, {y:.2f}, {z:.2f})  heading={math.degrees(heading):.1f}°  pitch={math.degrees(pitch):.1f}°")
    else:
        position, heading = auto_camera_pose(scene_path)
        x, y, z = position
        pitch = 0.0
        log(f"Camera: auto  pos=({x:.2f}, {y:.2f}, {z:.2f})  heading={math.degrees(heading):.1f}°")

    try:
        cam = mavs.MavsCamera()
        cam.Initialize(image_width, image_height, 0.0062222, 0.0035, 0.0035)
        cam.SetAntiAliasingFactor(1)
        cam.SetSaturationAndTemp(1.05, 7500.0)
        cam.SetGammaAndGain(0.75, 2.0)
        cam.RenderShadows(True)
        cam.SetOffset([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
        cam.SetPose([x, y, z], pose_quaternion(heading, pitch))
        cam.Update(env, dt)
        cam.Display()
    except Exception as exc:
        die("camera setup", exc)

    log("")
    log("Controls:")
    log("  Click the MAVS camera window first.")
    log("  Up/W          move forward (along camera direction)")
    log("  Down/S        move backward")
    log("  Left/A        turn left")
    log("  Right/D       turn right")
    log("  Q             fly up")
    log("  E             fly down")
    log("  R             pitch up (look up)")
    log("  F             pitch down (look down)")
    log("  Close window  save preview image and quit")
    log("")

    try:
        while cam.DisplayOpen():
            loop_start = time.time()
            cmd = cam.GetDrivingCommand()
            throttle, steering, braking = get_command_values(cmd)

            heading += steering * turn_speed * dt

            if _key(_VK_R):
                pitch = min(1.4, pitch + pitch_speed * dt)
            if _key(_VK_F):
                pitch = max(-1.4, pitch - pitch_speed * dt)

            fwd = throttle - braking
            cos_p = math.cos(pitch)
            x += fwd * move_speed * cos_p * math.cos(heading) * dt
            y += fwd * move_speed * cos_p * math.sin(heading) * dt
            z += fwd * move_speed * math.sin(pitch) * dt

            if _key(_VK_Q):
                z += move_speed * dt
            if _key(_VK_E):
                z -= move_speed * dt

            cam.SetPose([x, y, z], pose_quaternion(heading, pitch))
            cam.Update(env, dt)
            cam.Display()
            time.sleep(max(0.0, dt - (time.time() - loop_start)))
    finally:
        try:
            cam.SaveCameraImage(str(preview_image_out))
            log(f"Saved preview image: {preview_image_out}")
        except Exception as exc:
            log(f"Could not save preview image: {exc}")


if __name__ == "__main__":
    main()
