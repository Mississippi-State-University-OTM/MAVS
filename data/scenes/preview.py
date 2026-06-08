import os
import sys
import time
import math
import traceback

# --------------------------------------------------
# MAVS setup
# --------------------------------------------------

sys.path.append(r"C:\mavs_work\mavs\src\mavs_python")

os.add_dll_directory(r"C:\mavs_work\mavs\install\bin")
os.add_dll_directory(r"C:\mavs_work\mavs\install\lib")

import mavs_interface as mavs
import mavs_python_paths

mavs_data_path = mavs_python_paths.mavs_data_path

# Important for relative mesh/texture paths in scene files.
os.chdir(mavs_data_path)
print("Working directory:", os.getcwd(), flush=True)


# --------------------------------------------------
# User settings
# --------------------------------------------------

mavs_scenefile = "/scenes/straight_line_obstacle_mrzr.json"

preview_image_out = r"C:\mavs_work\frames\scene_preview.bmp"

# Low-res fast preview
image_width = 640
image_height = 360
aa_factor = 1

# Start camera near the route, looking forward.
camera_x = -125.0
camera_y = 0.0
camera_z = 2.0
camera_heading = 0.0

# Movement controls
move_speed = 8.0          # m/s
strafe_speed = 6.0        # m/s
turn_speed = 1.2          # rad/s
vertical_speed = 3.0      # m/s

dt = 1.0 / 30.0


# --------------------------------------------------
# Helpers
# --------------------------------------------------

def log(msg):
    print(msg, flush=True)


def die(stage, exc=None):
    print()
    print("=" * 80)
    print("FAILED AT:", stage)
    if exc is not None:
        print("Exception:", repr(exc))
        traceback.print_exc()
    print("=" * 80)
    raise SystemExit(1)


def safe_mkdir(path):
    os.makedirs(path, exist_ok=True)


def yaw_to_quaternion(heading):
    half = 0.5 * heading
    return [math.cos(half), 0.0, 0.0, math.sin(half)]


def get_command_values(cmd):
    """
    MAVS GetDrivingCommand generally maps arrow keys:
      up/down = throttle/braking
      left/right = steering
    We reuse it for camera preview movement.
    """
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


# --------------------------------------------------
# Setup
# --------------------------------------------------

scene_path = mavs_data_path + mavs_scenefile

log("MAVS low-res scene preview")
log("--------------------------")
log(f"MAVS data path: {mavs_data_path}")
log(f"Scene: {scene_path}")
log(f"Preview image out: {preview_image_out}")

safe_mkdir(os.path.dirname(preview_image_out))


# --------------------------------------------------
# Load scene/environment
# --------------------------------------------------

try:
    log("Creating scene/env")
    scene = mavs.MavsEmbreeScene()
    env = mavs.MavsEnvironment()

    log("Loading scene")
    scene.Load(scene_path)

    log("Setting scene")
    env.SetScene(scene)

    log("Setting environment")
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


# --------------------------------------------------
# Camera
# --------------------------------------------------

try:
    log("Creating camera")
    cam = mavs.MavsCamera()
    cam.Initialize(image_width, image_height, 0.0062222, 0.0035, 0.0035)

    cam.SetAntiAliasingFactor(aa_factor)
    cam.SetSaturationAndTemp(1.05, 7500.0)
    cam.SetGammaAndGain(0.75, 2.0)
    cam.RenderShadows(True)

    # No vehicle-relative offset. This camera pose is absolute.
    cam.SetOffset([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])

    log("Initial camera update")
    cam.SetPose([camera_x, camera_y, camera_z], yaw_to_quaternion(camera_heading))
    cam.Update(env, dt)
    cam.Display()

except Exception as exc:
    die("camera setup", exc)


# --------------------------------------------------
# Preview loop
# --------------------------------------------------

log("")
log("Controls:")
log("  Click the MAVS camera window first.")
log("  Up Arrow      move forward")
log("  Down Arrow    move backward")
log("  Left Arrow    turn left")
log("  Right Arrow   turn right")
log("  Close window  save preview image and quit")
log("")
log("Preview started.")

x = camera_x
y = camera_y
z = camera_z
heading = camera_heading

last_status = time.time()
nsteps = 0

try:
    while cam.DisplayOpen() or nsteps == 0:
        loop_start = time.time()

        cmd = cam.GetDrivingCommand()
        throttle, steering, braking = get_command_values(cmd)

        # Use MAVS driving command as camera navigation.
        heading += steering * turn_speed * dt

        forward = throttle - braking
        x += forward * move_speed * math.cos(heading) * dt
        y += forward * move_speed * math.sin(heading) * dt

        orientation = yaw_to_quaternion(heading)
        position = [x, y, z]

        cam.SetPose(position, orientation)
        cam.Update(env, dt)
        cam.Display()

        now = time.time()
        if now - last_status > 2.0:
            log(
                f"camera pos=({x:.2f}, {y:.2f}, {z:.2f}) "
                f"heading={heading:.3f} "
                f"cmd=({throttle:.1f}, {steering:.1f}, {braking:.1f})"
            )
            last_status = now

        nsteps += 1

        elapsed = time.time() - loop_start
        time.sleep(max(0.0, dt - elapsed))

finally:
    try:
        log("Saving preview image")
        cam.SaveCameraImage(preview_image_out)
        log(f"Saved: {preview_image_out}")
    except Exception as exc:
        log(f"Could not save preview image: {exc}")

    log("Preview stopped.")