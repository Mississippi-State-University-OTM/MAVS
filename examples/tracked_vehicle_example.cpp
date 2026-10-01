// c++ includes
#include <iostream>
#include <algorithm>
// project includes
#include "vehicles/tracked/tracked_vehicle.h"
#include "vehicles/tracked/tracked_render.h"
#include <sensors/mavs_sensors.h>
#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif

static float throttle = 0.0f;
static float steering = 0.0f;
static float braking = 0.0f;
static float cstep = 0.001f;

static void UpdateDrivingCommand(std::vector<bool> keyboard_commands) {
	
	if (keyboard_commands[0]) {
		throttle += cstep;
		braking = 0.0f;
	}
	else if (keyboard_commands[1]) {
		braking += cstep;
		throttle = 0.0f;
	}
	else {
		braking = 0.0f;
		throttle = 0.0f;
	}
	if (keyboard_commands[2]) {
		steering += cstep;
	}
	else if (keyboard_commands[3]) {
		steering -= cstep;
	}
	else {
		steering = 0.0f;
	}
	throttle = std::max(0.0f, std::min(1.0f, throttle));
	braking = std::max(0.0f, std::min(1.0f, braking));
	steering = std::max(-1.0f, std::min(1.0f, steering));
}


int main(int argc, char** argv) {

    std::string scene_file(argv[1]);
    std::string vehic_file(argv[2]);

	mavs::vehicle::tracked::TrackedVehicle tracked_veh(vehic_file);

	mavs::vehicle::tracked::TrackedRender tracked_debug_render(&tracked_veh);

    mavs::raytracer::embree::EmbreeTracer scene;
    scene.Load(scene_file);
    mavs::environment::Environment env;
	env.SetRaytracer(&scene);
	//float theta = -1.570796f/4.0f;
	//glm::vec3 sensor_offset(-8.0f, 3.0f, 2.0f);
	//glm::quat sensor_orient(cosf(0.5f * theta), 0.0f, 0.0f, sinf(0.5f * theta));
	float theta = -1.570796f;
	glm::vec3 sensor_offset(0.0f, 6.5f, 1.25f);
	glm::quat sensor_orient(cosf(0.5f * theta), 0.0f, 0.0f, sinf(0.5f * theta));
	//glm::vec3 sensor_offset(-10.0f, 0.0f, 1.5f);
	//glm::quat sensor_orient(1.0f, 0.0f, 0.0f, 0.0f);
	glm::vec3 position(0.0f, 0.0f, 1.0f);
	glm::quat orient(1.0f, 0.0f, 0.0f, 0.0f);
	mavs::sensor::camera::RgbCamera camera;
	camera.SetEnvironmentProperties(&env);
	camera.Initialize(640, 480, 0.00466667f, 0.0035f, 0.0035f);
	camera.SetRelativePose(sensor_offset, sensor_orient);
	camera.SetName("camera");
	camera.SetPose(position, orient);
	camera.SetElectronics(0.95f, 1.0f);

	int nsteps = 0;
	while (camera.DisplayOpen() || nsteps == 0) {

		UpdateDrivingCommand(camera.GetKeyCommands());

		tracked_veh.Update(&env, throttle, steering, braking, (float)tracked_veh.GetSimulationDt());
		tracked_debug_render.Update();

		if (nsteps % 50 == 0) {
			glm::dquat ori = tracked_veh.GetOrientation();
			camera.SetPose(tracked_veh.GetPosition(), tracked_veh.GetOrientation());
			camera.Update(&env, 0.03);
			camera.Display();
		}
		
		nsteps++;

    }

}
