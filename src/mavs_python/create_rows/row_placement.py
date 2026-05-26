import json
import math
import random
from pathlib import Path


def load_scene(scene_path):
    with open(scene_path, "r", encoding="utf-8") as file:
        return json.load(file)


def write_scene(scene, output_path):
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as file:
        json.dump(scene, file, indent=4)


def discover_obj_meshes(mesh_dir, meshes_root=None):
    mesh_dir = Path(mesh_dir)
    meshes_root = Path(meshes_root) if meshes_root is not None else mesh_dir.parent
    return sorted(
        path.relative_to(meshes_root).as_posix()
        for path in mesh_dir.rglob("*.obj")
    )


def _as_xy(point, name):
    if len(point) != 2:
        raise ValueError(f"{name} must contain exactly two values: [x, y].")
    return (float(point[0]), float(point[1]))


def _unit_vector(vector):
    length = math.hypot(vector[0], vector[1])
    if length <= 0:
        raise ValueError("The first row start and end points must be different.")
    return (vector[0] / length, vector[1] / length), length


def _add_xy(left, right):
    return (left[0] + right[0], left[1] + right[1])


def _scale_xy(vector, scalar):
    return (vector[0] * scalar, vector[1] * scalar)


def generate_row_points(first_row_start, first_row_end, plant_spacing, row_spacing, num_rows, row_side=1):
    if plant_spacing <= 0:
        raise ValueError("plant_spacing must be greater than zero.")
    if row_spacing < 0:
        raise ValueError("row_spacing must be zero or greater.")
    if num_rows <= 0:
        raise ValueError("num_rows must be greater than zero.")
    if row_side not in (-1, 1):
        raise ValueError("row_side must be either 1 or -1.")

    start = _as_xy(first_row_start, "first_row_start")
    end = _as_xy(first_row_end, "first_row_end")
    row_vector = (end[0] - start[0], end[1] - start[1])
    row_direction, row_length = _unit_vector(row_vector)
    side_direction = (-row_direction[1] * row_side, row_direction[0] * row_side)

    interval_count = int(row_length // plant_spacing)
    point_count = interval_count + 1

    rows = []
    for row_index in range(num_rows):
        row_offset = _scale_xy(side_direction, row_spacing * row_index)
        row_start = _add_xy(start, row_offset)
        row_points = []

        for point_index in range(point_count):
            distance = min(plant_spacing * point_index, row_length)
            point = _add_xy(row_start, _scale_xy(row_direction, distance))
            row_points.append(point)

        rows.append(row_points)

    return rows


def make_instance(yaw_pitch_roll, position, scale):
    return {
        "YawPitchRoll": yaw_pitch_roll,
        "Position": position,
        "Scale": scale,
    }


def make_object(mesh_path, yaw_pitch_roll, position, scale, label_by_group=False):
    return {
        "Mesh": mesh_path,
        "Smooth Normals": False,
        "Label By Group": label_by_group,
        "Rotate Y to Z": False,
        "Instances": [make_instance(yaw_pitch_roll, position, scale)],
    }


def place_object(scene, mesh_path, yaw_pitch_roll, position, scale, label_by_group=False):
    if "Objects" not in scene:
        raise KeyError("The input scene must contain an 'Objects' list.")

    for scene_object in scene["Objects"]:
        if scene_object["Mesh"] == mesh_path:
            scene_object["Instances"].append(make_instance(yaw_pitch_roll, position, scale))
            return scene

    scene["Objects"].append(
        make_object(
            mesh_path=mesh_path,
            yaw_pitch_roll=yaw_pitch_roll,
            position=position,
            scale=scale,
            label_by_group=label_by_group,
        )
    )
    return scene


def random_yaw_pitch_roll(rng, tilt_degrees=0.5):
    return [
        rng.uniform(-tilt_degrees, tilt_degrees),
        rng.uniform(-180.0, 180.0),
        rng.uniform(-tilt_degrees, tilt_degrees),
    ]


def place_objects_in_rows(
    scene,
    mesh_paths,
    first_row_start,
    first_row_end,
    plant_spacing,
    row_spacing,
    num_rows,
    scale,
    row_side=1,
    position_noise_std=0.01,
    z_value=0.0,
    label_by_group=False,
    seed=None,
):
    if len(mesh_paths) == 0:
        raise ValueError("mesh_paths must contain at least one OBJ mesh path.")

    rng = random.Random(seed)
    rows = generate_row_points(
        first_row_start=first_row_start,
        first_row_end=first_row_end,
        plant_spacing=plant_spacing,
        row_spacing=row_spacing,
        num_rows=num_rows,
        row_side=row_side,
    )

    for row_points in rows:
        for x_value, y_value in row_points:
            mesh_path = rng.choice(mesh_paths)
            position = [
                x_value + rng.gauss(0.0, position_noise_std),
                y_value + rng.gauss(0.0, position_noise_std),
                float(z_value),
            ]
            yaw_pitch_roll = random_yaw_pitch_roll(rng)
            place_object(
                scene=scene,
                mesh_path=mesh_path,
                yaw_pitch_roll=yaw_pitch_roll,
                position=position,
                scale=scale,
                label_by_group=label_by_group,
            )

    return scene


def create_scene_with_rows(
    scene_file,
    mesh_dir,
    output_file,
    first_row_start,
    first_row_end,
    plant_spacing,
    row_spacing,
    num_rows,
    scale,
    meshes_root=None,
    row_side=1,
    position_noise_std=0.01,
    z_value=0.0,
    label_by_group=False,
    seed=None,
):
    scene = load_scene(scene_file)
    mesh_paths = discover_obj_meshes(mesh_dir=mesh_dir, meshes_root=meshes_root)
    scene = place_objects_in_rows(
        scene=scene,
        mesh_paths=mesh_paths,
        first_row_start=first_row_start,
        first_row_end=first_row_end,
        plant_spacing=plant_spacing,
        row_spacing=row_spacing,
        num_rows=num_rows,
        scale=scale,
        row_side=row_side,
        position_noise_std=position_noise_std,
        z_value=z_value,
        label_by_group=label_by_group,
        seed=seed,
    )
    write_scene(scene, output_file)
    return scene
