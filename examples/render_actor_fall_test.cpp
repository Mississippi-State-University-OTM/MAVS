/*
  Simple MAVS falling actor test.

  No vehicle.
  No path.
  Fixed camera.
  Actor starts falling after 1 second.
  Saves frames to output folder.

  Usage:
    render_actor_fall_test.exe scene.json output_folder

  Example:
    render_actor_fall_test.exe ^
      C:\mavs_work\mavs\data\scenes\surface_only.json ^
      C:\mavs_work\frames\actor_fall_test
*/

#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <cmath>
#include <algorithm>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include <mavs_core/math/utils.h>
#include <mavs_core/environment/environment.h>

#include <raytracers/embree_tracer/embree_tracer.h>
#include <sensors/mavs_sensors.h>

namespace fs = std::filesystem;

static glm::vec3 LerpVec3(const glm::vec3& a, const glm::vec3& b, float u) {
    return (1.0f - u) * a + u * b;
}

static glm::quat YawQuat(float heading) {
    return glm::quat(
        std::cos(0.5f * heading),
        0.0f,
        0.0f,
        std::sin(0.5f * heading)
    );
}

int main(int argc, char* argv[]) {
#ifdef USE_EMBREE

    if (argc < 3) {
        std::cerr << "Usage:\n";
        std::cerr << "  render_actor_fall_test.exe scene.json output_folder\n";
        return 1;
    }

    std::string scene_file = argv[1];
    std::string output_folder = argv[2];

    fs::create_directories(output_folder);

    std::cout << "Scene file   : " << scene_file << std::endl;
    std::cout << "Output folder: " << output_folder << std::endl;

    // --------------------------------------------------
    // Render settings
    // --------------------------------------------------

    const float fps = 30.0f;
    const float dt = 1.0f / fps;
    const int num_frames = 180;       // 6 seconds at 30 FPS

    const float trigger_time = 1.0f;  // actor starts falling after 1 sec
    const float transition_time = 3.0f;

    // --------------------------------------------------
    // Actor settings
    // --------------------------------------------------

    const std::string actor_mesh = "vegetation/pine_tree/pine_tree.obj";

    const bool actor_y_to_z = false;
    const bool actor_x_to_y = false;
    const bool actor_y_to_x = false;

    // Actor pivot point in world.
    // Put it off the side of the road / view.
    const glm::vec3 actor_initial_pos(0.0f, 10.0f, 0.0f);
    const glm::vec3 actor_final_pos(0.0f, 10.0f, 0.25f);

    // Mesh transform relative to actor pivot.
    // Start simple: no side offset. If the trunk is not rooted where expected,
    // change this after verifying rotation works.
    const glm::vec3 actor_mesh_offset(0.0f, 0.0f, 0.0f);
    const glm::vec3 actor_mesh_scale(1.0f, 1.0f, 2.0f);

    const float pi = 3.14159265358979323846f;

    const glm::quat actor_initial_q(1.0f, 0.0f, 0.0f, 0.0f);

    // This matches your ROS launch:
    // final_orientation: [cos(0.25*pi), sin(0.25*pi), 0, 0]
    // 90 degrees around X.
    const glm::quat actor_final_q(
        std::cos(0.25f * pi),
        std::sin(0.25f * pi),
        0.0f,
        0.0f
    );

    // If the tree does not visibly rotate with X-axis rotation, try this instead:
    //
    // const glm::quat actor_final_q(
    //     std::cos(0.25f * pi),
    //     0.0f,
    //     std::sin(0.25f * pi),
    //     0.0f
    // );

    // --------------------------------------------------
    // Load scene
    // --------------------------------------------------

    mavs::raytracer::embree::EmbreeTracer scene;
    scene.Load(scene_file);

    std::cout << "Loaded scene with "
              << scene.GetNumberTrianglesLoaded()
              << " triangles."
              << std::endl;

    // --------------------------------------------------
    // Environment
    // --------------------------------------------------

    mavs::environment::Environment env;
    env.SetRaytracer(&scene);

    env.SetTurbidity(2.0f);
    env.SetFog(0.0f);
    env.SetDateTime(2023, 9, 1, 12, 0, 0, 6);

    // --------------------------------------------------
    // Add actor
    // --------------------------------------------------

    std::cout << "Adding actor mesh: " << actor_mesh << std::endl;

    std::vector<int> actor_ids = env.AddActor(
        actor_mesh,
        actor_y_to_z,
        actor_x_to_y,
        actor_y_to_x,
        actor_mesh_offset,
        actor_mesh_scale
    );

    if (actor_ids.empty()) {
        std::cerr << "ERROR: AddActor returned no actor ids." << std::endl;
        return 2;
    }

    int actor_id = actor_ids[0];

    std::cout << "Actor id: " << actor_id << std::endl;
    std::cout << "Actor initial world pos: "
              << actor_initial_pos.x << ", "
              << actor_initial_pos.y << ", "
              << actor_initial_pos.z << std::endl;

    env.SetActorPosition(actor_id, actor_initial_pos, actor_initial_q, true);

    // --------------------------------------------------
    // Camera
    // --------------------------------------------------

    mavs::sensor::camera::RgbCamera camera;
    camera.SetEnvironmentProperties(&env);

    camera.Initialize(1280, 720, 0.0062222f, 0.0035f, 0.0035f);
    camera.SetName("camera");
    camera.SetAntiAliasing("oversampled");
    camera.SetPixelSampleFactor(2);
    camera.SetElectronics(0.75f, 2.0f);

    // Fixed camera looking toward the actor.
    // Camera default looks along local +X, so yaw toward actor.
    glm::vec3 camera_pos(-28.0f, -4.0f, 4.0f);

    float dx = actor_initial_pos.x - camera_pos.x;
    float dy = actor_initial_pos.y - camera_pos.y;
    float heading = std::atan2(dy, dx);

    glm::quat camera_q = YawQuat(heading);

    camera.SetPose(camera_pos, camera_q);

    std::cout << "Camera position: "
              << camera_pos.x << ", "
              << camera_pos.y << ", "
              << camera_pos.z << std::endl;
    std::cout << "Camera heading: " << heading << std::endl;

    // --------------------------------------------------
    // Render loop
    // --------------------------------------------------

    std::cout << "Rendering actor fall test..." << std::endl;

    for (int frame_id = 0; frame_id < num_frames; frame_id++) {
        float sim_time = frame_id * dt;

        float u = 0.0f;
        if (sim_time >= trigger_time) {
            u = (sim_time - trigger_time) / transition_time;
            u = std::clamp(u, 0.0f, 1.0f);
        }

        glm::vec3 actor_pos = LerpVec3(actor_initial_pos, actor_final_pos, u);
        glm::quat actor_q = glm::normalize(glm::slerp(actor_initial_q, actor_final_q, u));

        // Commit every frame.
        env.SetActorPosition(actor_id, actor_pos, actor_q, true);

        camera.Update(&env, dt);

        std::string image_name =
            output_folder + "/image_" + mavs::utils::ToString(frame_id, 6) + ".bmp";

        camera.SaveImage(image_name);

        if (frame_id % 10 == 0 || frame_id == num_frames - 1) {
            std::cout << "frame " << frame_id
                      << "  t=" << sim_time
                      << "  actor_u=" << u
                      << "  saved=" << image_name
                      << std::endl;
        }
    }

    std::cout << "Done." << std::endl;
    return 0;

#else
    std::cerr << "ERROR: This program requires MAVS built with Embree." << std::endl;
    return 1;
#endif
}