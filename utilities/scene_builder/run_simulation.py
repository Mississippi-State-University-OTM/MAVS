#!/usr/bin/env python3
"""Launch a MAVS simulation with RP3D vehicles following waypoint paths."""

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
RUNTIME_VEHICLE_ROOT = DATA_ROOT / "vehicles" / "rp3d_vehicles" / "_scene_builder_runtime"

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
    if isinstance(cmd, (list, tuple)) and len(cmd) >= 3:
        return float(cmd[0]), float(cmd[1]), float(cmd[2])
    throttle = float(getattr(cmd, "throttle", 0.0) or getattr(cmd, "Throttle", 0.0))
    steering = float(getattr(cmd, "steering", 0.0) or getattr(cmd, "Steering", 0.0))
    braking = float(getattr(cmd, "braking", 0.0) or getattr(cmd, "Braking", 0.0))
    return throttle, steering, braking


def controller_path(
    initial_position: list[float],
    waypoints: list[list[float]],
    spacing: float = 1.0,
) -> list[list[float]]:
    """Build a dense path whose first point is the vehicle's current position."""
    points = [[float(initial_position[0]), float(initial_position[1])]]
    for waypoint in waypoints:
        point = [float(waypoint[0]), float(waypoint[1])]
        start = points[-1]
        distance = math.hypot(point[0] - start[0], point[1] - start[1])
        if distance < 0.001:
            continue
        steps = max(1, math.ceil(distance / spacing))
        for step in range(1, steps + 1):
            fraction = step / steps
            points.append([
                start[0] + (point[0] - start[0]) * fraction,
                start[1] + (point[1] - start[1]) * fraction,
            ])
    return points


def auto_camera_pose(scene_path: Path) -> tuple[list[float], float]:
    try:
        scene = json.loads(scene_path.read_text(encoding="utf-8"))
    except Exception:
        return [-35.0, 0.0, 6.0], 0.0
    positions: list[list[float]] = []
    for obj in scene.get("Objects", []):
        if str(obj.get("Mesh", "")).endswith("scene_builder_plane.obj"):
            continue
        for inst in obj.get("Instances", []):
            p = inst.get("Position", [])
            if len(p) == 3:
                positions.append([float(p[0]), float(p[1]), float(p[2])])
    if not positions:
        return [-35.0, 0.0, 6.0], 0.0
    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    zs = [p[2] for p in positions]
    cx = 0.5 * (min(xs) + max(xs))
    cy = 0.5 * (min(ys) + max(ys))
    span = max(1.0, math.hypot(max(xs) - min(xs), max(ys) - min(ys)))
    dist = max(18.0, 1.15 * span)
    height = max(3.0, max(zs) + 0.25 * dist)
    return [cx - dist, cy, height], 0.0


def main() -> None:
    if len(sys.argv) < 3:
        raise SystemExit("Usage: run_simulation.py <scene.json> <vehicles.json>")

    scene_path = Path(sys.argv[1]).resolve()
    vehicles_path = Path(sys.argv[2]).resolve()

    image_width = 960
    image_height = 540
    dt = 1.0 / 30.0
    move_speed = 10.0
    turn_speed = 1.2
    pitch_speed = 0.9

    os.chdir(DATA_ROOT)
    log("MAVS scene-builder simulation")
    log("------------------------------")
    log(f"Scene:  {scene_path}")
    log(f"Vehicles: {vehicles_path}")
    log(f"CWD:    {Path.cwd()}")

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

    try:
        scene_data = json.loads(scene_path.read_text(encoding="utf-8"))
        RUNTIME_VEHICLE_ROOT.mkdir(parents=True, exist_ok=True)
        vehicle_inputs = json.loads(vehicles_path.read_text(encoding="utf-8")).get("vehicles", [])
        if not vehicle_inputs:
            raise ValueError("No vehicle presets were provided")
        vehicles = []
        for entry in vehicle_inputs:
            controller_mode = str(entry.get("controller_mode", "path")).lower()
            if controller_mode not in {"path", "human", "none"}:
                controller_mode = "none"
            waypoints = entry.get("waypoints", [])
            if controller_mode == "path" and len(waypoints) < 2:
                continue
            definition_path = DATA_ROOT / "vehicles" / "rp3d_vehicles" / entry["definition_file"]
            definition = json.loads(definition_path.read_text(encoding="utf-8"))
            if definition.get("Using Veg Forces") and not scene_data.get("Vegetation File"):
                definition["Using Veg Forces"] = False
                definition_path = RUNTIME_VEHICLE_ROOT / entry["definition_file"]
                definition_path.write_text(json.dumps(definition), encoding="utf-8")
            vehicle = mavs.MavsRp3d()
            vehicle.Load(str(definition_path))
            fallback_initial = [waypoints[0][0], waypoints[0][1], 0.0] if waypoints else [0.0, 0.0, 0.0]
            initial = entry.get("initial_position", fallback_initial)
            vehicle.SetInitialPosition(float(initial[0]), float(initial[1]), float(initial[2]))
            heading = math.radians(float(entry.get("initial_heading_degrees", 0.0)))
            vehicle.SetInitialHeading(heading)
            vehicle.Update(env, 0.0, 0.0, 1.0, 0.000001)

            controller = None
            if controller_mode == "path":
                controller = mavs.MavsVehicleController()
                path_spacing = max(0.1, float(entry.get("pathSpacing", 1.0)))
                desired_path = controller_path(initial, waypoints, path_spacing)
                controller.SetDesiredPath(desired_path)
                controller.SetDesiredSpeed(float(entry.get("speed", 5.0)))
                controller.SetSteeringScale(float(entry.get("steeringScale", 3.0)))
                controller.SetWheelbase(max(0.5, float(entry.get("wheelbase", 1.25))))
                controller.SetMaxSteerAngle(max(0.01, float(entry.get("maxSteerAngle", 0.6))))
                min_lookahead = max(0.1, float(entry.get("minLookAhead", 5.0)))
                max_lookahead = max(min_lookahead, float(entry.get("maxLookAhead", 25.0)))
                controller.SetMinLookAhead(min_lookahead)
                controller.SetMaxLookAhead(max_lookahead)
                log(
                    f"Vehicle path: {len(waypoints)} editor waypoints expanded "
                    f"to {len(desired_path)} controller points at {path_spacing:.2f} m spacing"
                )
                log(
                    "Controller params: "
                    f"speed={float(entry.get('speed', 5.0)):.2f} m/s, "
                    f"steering scale={float(entry.get('steeringScale', 3.0)):.2f}, "
                    f"max steer={float(entry.get('maxSteerAngle', 0.6)):.2f} rad, "
                    f"lookahead={min_lookahead:.2f}-{max_lookahead:.2f} m"
                )
            log(f"Vehicle controller mode: {controller_mode}")
            vehicles.append((vehicle, controller_mode, controller, entry))
        if not vehicles:
            raise ValueError("No valid vehicle controller configurations were provided")
        human_vehicle = next((v for v, mode, _, _e in vehicles if mode == "human"), None)
        human_camera_offset = next(
            (_e.get("camera_offset", [-8.0, 0.0, 3.5]) for v, mode, _, _e in vehicles if mode == "human"),
            [-8.0, 0.0, 3.5],
        )
    except Exception as exc:
        die("vehicle/controller loading", exc)

    position, heading = auto_camera_pose(scene_path)
    x, y, z = position
    pitch = 0.0

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
    log("  Click the MAVS window first.")
    if human_vehicle is not None:
        log("  W / Up        throttle (drive forward)")
        log("  S / Down      brake / reverse")
        log("  A / Left      steer left")
        log("  D / Right     steer right")
        log("  Camera follows vehicle in 3rd-person view.")
    else:
        log("  Up/W          move forward (along camera direction)")
        log("  Down/S        move backward")
        log("  Left/A        turn left")
        log("  Right/D       turn right")
        log("  Q             fly up")
        log("  E             fly down")
        log("  R             pitch up (look up)")
        log("  F             pitch down (look down)")
    log("  Close window  quit")
    log("")

    try:
        while cam.DisplayOpen():
            loop_start = time.time()
            cmd = cam.GetDrivingCommand()
            throttle, steering, braking = get_command_values(cmd)
            for vehicle, controller_mode, controller, _entry in vehicles:
                if controller_mode == "path" and controller is not None:
                    position = vehicle.GetPosition()
                    controller.SetCurrentState(position[0], position[1], vehicle.GetSpeed(), vehicle.GetHeading())
                    command = controller.GetDrivingCommand(dt)
                    vehicle.Update(env, command.throttle, command.steering, command.braking, dt)
                elif controller_mode == "human":
                    vehicle.Update(env, throttle, steering, braking, dt)
                else:
                    vehicle.Update(env, 0.0, 0.0, 1.0, dt)
            env.AdvanceTime(dt)

            if human_vehicle is not None:
                veh_pos = human_vehicle.GetPosition()
                veh_heading = human_vehicle.GetHeading()
                veh_z = float(veh_pos[2]) if len(veh_pos) > 2 else 0.0
                bx, by, bz = human_camera_offset
                cam_x = veh_pos[0] + bx * math.cos(veh_heading) - by * math.sin(veh_heading)
                cam_y = veh_pos[1] + bx * math.sin(veh_heading) + by * math.cos(veh_heading)
                cam_z = veh_z + bz
                horiz = math.hypot(bx, by)
                cam_pitch = math.atan2(bz, horiz) if horiz > 0.01 else 0.0
                cam.SetPose([cam_x, cam_y, cam_z], pose_quaternion(veh_heading, cam_pitch))
            else:
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
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
