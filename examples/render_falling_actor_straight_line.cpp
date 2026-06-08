/*
  MAVS kinematic render with falling tree + MRZR vehicle in city scene.

  Vehicle is composed of 5 actors because mrzr_no_wheel_rot.obj does not
  include the wheels; mrzr_single_wheel.obj is loaded 4 times and the
  wheels are pinned to the body's axle positions every frame. No physics.

  Actor IDs, assuming falling_tree_scene_actors.json load order:
    0: tree
    1: chassis
    2: front-left  wheel
    3: front-right wheel
    4: rear-left   wheel
    5: rear-right  wheel

  Usage:
    render_falling_actor_straight_line.exe scene.json actors.json output_folder

  Example:
    render_falling_actor_straight_line.exe ^
      C:\mavs_work\mavs\data\scenes\spa_city.json ^
      C:\mavs_work\mavs\data\actors\actors\falling_tree_scene_actors.json ^
      C:\mavs_work\frames\falling_tree_cpp
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

static glm::quat HeadingQuat(float heading) {
    return glm::quat(
        std::cos(0.5f * heading),
        0.0f,
        0.0f,
        std::sin(0.5f * heading)
    );
}

int main(int argc, char* argv[]) {
#ifdef USE_EMBREE

    if (argc < 4) {
        std::cerr << "Usage:\n";
        std::cerr << "  render_falling_actor_straight_line.exe "
                  << "scene.json actors.json output_folder\n";
        return 1;
    }

    std::string scene_file    = argv[1];
    std::string actor_file    = argv[2];
    std::string output_folder = argv[3];

    fs::create_directories(output_folder);

    std::cout << "Scene file   : " << scene_file    << std::endl;
    std::cout << "Actors file  : " << actor_file    << std::endl;
    std::cout << "Output folder: " << output_folder << std::endl;

    // --------------------------------------------------
    // Render / motion settings
    // --------------------------------------------------

    const float fps = 30.0f;
    const float dt  = 1.0f / fps;


     // forest road test location.
    // Start on the road lane at x=-43.7, y=4.42 and drive along +X.
    /*const float start_x        = -60.0f;
    const float start_y        =   0.0f;
    const float end_x          =  -12.0f;
    const float start_heading  =   0.0f;
    const float start_z_offset =   1.0f;   // chassis mesh ride height
*/
    // City road test location.
    // Start on the road lane at x=-43.7, y=4.42 and drive along +X.
    /**/const float start_x        = -45.7f;
    const float start_y        =   5.75f;
    const float end_x          =  -12.0f;
    const float start_heading  =   0.0f;
    const float start_z_offset =   1.0f;   // chassis mesh ride height

    const float vehicle_speed  =   5.0f;

    const float route_length = std::max(0.0f, end_x - start_x);
    const float route_time   = route_length / vehicle_speed;
    const int num_frames     = static_cast<int>(std::ceil(route_time * fps)) + 1;

    // Tree begins to fall shortly before the vehicle reaches it.
    const float fall_start_time = 3.5f;
    const float transition_time = 2.0f;

    // --------------------------------------------------
    // Vehicle geometry, matching mrzr4_tires_low_gear.json axles
    // --------------------------------------------------

    const float front_axle_x = 1.34625f;
    const float rear_axle_x  = -1.37375f;
    const float front_half_track = 0.651f;
    const float rear_half_track  = 0.6695f;
    const float wheel_radius = 0.33f;

    // Body-frame wheel offsets.
    // Heading is 0 for this city-road test, so body frame == world axes.
    const glm::vec3 wheel_fl_offset( front_axle_x,  front_half_track, 0.0f);
    const glm::vec3 wheel_fr_offset( front_axle_x, -front_half_track, 0.0f);
    const glm::vec3 wheel_rl_offset( rear_axle_x,   rear_half_track,  0.0f);
    const glm::vec3 wheel_rr_offset( rear_axle_x,  -rear_half_track,  0.0f);

    // --------------------------------------------------
    // Tree pose targets
    // --------------------------------------------------
    //
    // The vehicle drives along y=4.42.
    // Tree starts on the side of the road at y=9.5 and falls into the lane.
    //
    // At 5 m/s:
    //   vehicle x at t=3.5 sec = -26.2
    //   tree x = -20.0
    // So the tree starts falling while the vehicle is approaching.
    /*
    const glm::vec3 tree_initial_pos(-15.0f, 8.0f, 0.0f);
    const glm::vec3 tree_final_pos  (-15.0f, 8.0f, 0.2f);*/
    /**/
    const glm::vec3 tree_initial_pos(-10.0f, 16.0f, 0.0f);
    const glm::vec3 tree_final_pos  (-10.0f, 16.0f, 0.2f);

    const float pi = 3.14159265358979323846f;

    //const glm::quat tree_initial_q(1.0f, 0.0f, 0.0f, 0.0f);
    const glm::quat tree_initial_q(
        std::cos(-0.25f * pi),
        0.0f,
        0.0f,
        std::sin(-0.25f * pi)
    );
    // Same falling rotation that worked in the earlier actor-only test.
    const glm::quat tree_final_q(
        std::cos(0.25f * pi),
        std::sin(0.25f * pi),
        0.0f,
        0.0f
    );

    // --------------------------------------------------
    // Scene & environment
    // --------------------------------------------------

    mavs::raytracer::embree::EmbreeTracer scene;
    scene.Load(scene_file);

    std::cout << "Loaded scene with "
              << scene.GetNumberTrianglesLoaded()
              << " triangles." << std::endl;

    mavs::environment::Environment env;
    env.SetRaytracer(&scene);
    env.SetTurbidity(2.0f);
    env.SetFog(0.0f);
    env.SetDateTime(2023, 9, 1, 19, 0, 0, 6);

    // --------------------------------------------------
    // Load actors
    // --------------------------------------------------

    std::cout << "Calling env.LoadActors(" << actor_file << ")..." << std::endl;
    env.LoadActors(actor_file);

    const int tree_id     = 0;
    const int chassis_id  = 1;
    const int wheel_fl_id = 2;
    const int wheel_fr_id = 3;
    const int wheel_rl_id = 4;
    const int wheel_rr_id = 5;

    // --------------------------------------------------
    // Initial surface height
    // --------------------------------------------------

    float zstart = 0.0f;

    try {
        zstart = scene.GetSurfaceHeight(start_x, start_y);
    }
    catch (...) {
        zstart = 0.0f;
    }

    if (!std::isfinite(zstart) || zstart < -1000.0f || zstart > 1000.0f) {
        std::cout << "WARNING: invalid surface height at start. Forcing zstart = 0.0" << std::endl;
        zstart = 0.0f;
    }

    glm::quat body_q = HeadingQuat(start_heading);
    glm::vec3 body_pos_initial(start_x, start_y, zstart + start_z_offset);

    // --------------------------------------------------
    // Pin initial actor poses
    // --------------------------------------------------

    env.SetActorPosition(tree_id,    tree_initial_pos, tree_initial_q, true);
    env.SetActorPosition(chassis_id, body_pos_initial, body_q,         true);

    glm::quat wheel_q = body_q;

    env.SetActorPosition(
        wheel_fl_id,
        glm::vec3(start_x + wheel_fl_offset.x, start_y + wheel_fl_offset.y, wheel_radius),
        wheel_q,
        true
    );

    env.SetActorPosition(
        wheel_fr_id,
        glm::vec3(start_x + wheel_fr_offset.x, start_y + wheel_fr_offset.y, wheel_radius),
        wheel_q,
        true
    );

    env.SetActorPosition(
        wheel_rl_id,
        glm::vec3(start_x + wheel_rl_offset.x, start_y + wheel_rl_offset.y, wheel_radius),
        wheel_q,
        true
    );

    env.SetActorPosition(
        wheel_rr_id,
        glm::vec3(start_x + wheel_rr_offset.x, start_y + wheel_rr_offset.y, wheel_radius),
        wheel_q,
        true
    );

    std::cout << "Initial test setup:" << std::endl;
    std::cout << "  vehicle start = (" << start_x << ", " << start_y << ", " << zstart + start_z_offset << ")" << std::endl;
    std::cout << "  vehicle end x = " << end_x << std::endl;
    std::cout << "  vehicle speed = " << vehicle_speed << " m/s" << std::endl;
    std::cout << "  frames        = " << num_frames << std::endl;
    std::cout << "  tree initial  = (" << tree_initial_pos.x << ", " << tree_initial_pos.y << ", " << tree_initial_pos.z << ")" << std::endl;
    std::cout << "  tree final    = (" << tree_final_pos.x << ", " << tree_final_pos.y << ", " << tree_final_pos.z << ")" << std::endl;

    // Optional actor probe.
    auto dump = [&](const char* name, int id, glm::vec3 expect) {
        glm::vec3 g = env.GetActorPosition(id);
        std::cout << "  " << name << " id=" << id
                  << " got=(" << g.x << "," << g.y << "," << g.z << ")"
                  << " expect=(" << expect.x << "," << expect.y << "," << expect.z << ")"
                  << std::endl;
    };

    std::cout << "After init:" << std::endl;
    dump("tree   ", tree_id,     tree_initial_pos);
    dump("chassis", chassis_id,  body_pos_initial);
    dump("wh FL  ", wheel_fl_id, glm::vec3(start_x + wheel_fl_offset.x, start_y + wheel_fl_offset.y, wheel_radius));
    dump("wh FR  ", wheel_fr_id, glm::vec3(start_x + wheel_fr_offset.x, start_y + wheel_fr_offset.y, wheel_radius));
    dump("wh RL  ", wheel_rl_id, glm::vec3(start_x + wheel_rl_offset.x, start_y + wheel_rl_offset.y, wheel_radius));
    dump("wh RR  ", wheel_rr_id, glm::vec3(start_x + wheel_rr_offset.x, start_y + wheel_rr_offset.y, wheel_radius));

    // --------------------------------------------------
    // Camera
    // --------------------------------------------------

    mavs::sensor::camera::RgbCamera camera;
    camera.SetEnvironmentProperties(&env);

    camera.Initialize(1920, 1080, 0.0062222f, 0.0035f, 0.0035f);
    camera.SetName("camera");
    camera.SetAntiAliasing("oversampled");
    camera.SetPixelSampleFactor(3);
    camera.SetElectronics(0.75f, 2.0f);

    // Chase camera.
    glm::vec3 sensor_offset(-8.0f, 0.0f, 2.4f);
    glm::quat sensor_orient(1.0f, 0.0f, 0.0f, 0.0f);
    camera.SetRelativePose(sensor_offset, sensor_orient);

    // --------------------------------------------------
    // Render loop
    // --------------------------------------------------

    std::cout << "Starting render loop..." << std::endl;

    for (int frame_id = 0; frame_id < num_frames; frame_id++) {
        float sim_time = frame_id * dt;

        // Body pose along city road.
        float body_x = start_x + vehicle_speed * sim_time;

        if (body_x > end_x) {
            body_x = end_x;
        }

        float body_y = start_y;
        float body_z = zstart + start_z_offset;

        glm::vec3 body_pos(body_x, body_y, body_z);

        // Tree fall interpolation.
        float u = 0.0f;

        if (sim_time >= fall_start_time) {
            u = (sim_time - fall_start_time) / transition_time;
            u = std::clamp(u, 0.0f, 1.0f);
        }

        glm::vec3 tree_pos = LerpVec3(tree_initial_pos, tree_final_pos, u);
        glm::quat tree_q   = glm::normalize(glm::slerp(tree_initial_q, tree_final_q, u));

        env.SetActorPosition(tree_id, tree_pos, tree_q, true);

        // Chassis.
        env.SetActorPosition(chassis_id, body_pos, body_q, true);

        // Wheels.
        // Heading is 0. If we later use nonzero heading, these offsets should be rotated by body_q.
        env.SetActorPosition(
            wheel_fl_id,
            glm::vec3(body_x + wheel_fl_offset.x, body_y + wheel_fl_offset.y, wheel_radius),
            body_q,
            true
        );

        env.SetActorPosition(
            wheel_fr_id,
            glm::vec3(body_x + wheel_fr_offset.x, body_y + wheel_fr_offset.y, wheel_radius),
            body_q,
            true
        );

        env.SetActorPosition(
            wheel_rl_id,
            glm::vec3(body_x + wheel_rl_offset.x, body_y + wheel_rl_offset.y, wheel_radius),
            body_q,
            true
        );

        env.SetActorPosition(
            wheel_rr_id,
            glm::vec3(body_x + wheel_rr_offset.x, body_y + wheel_rr_offset.y, wheel_radius),
            body_q,
            true
        );

        // Camera follows vehicle body.
        camera.SetPose(body_pos, body_q);
        camera.Update(&env, dt);

        std::string image_name =
            output_folder + "/image_" + mavs::utils::ToString(frame_id, 6) + ".bmp";

        camera.SaveImage(image_name);

        if (frame_id % 10 == 0 || frame_id == num_frames - 1) {
            std::cout << "frame " << frame_id
                      << "  t=" << sim_time
                      << "  body=(" << body_x << "," << body_y << "," << body_z << ")"
                      << "  tree_u=" << u
                      << "  tree=(" << tree_pos.x << "," << tree_pos.y << "," << tree_pos.z << ")"
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