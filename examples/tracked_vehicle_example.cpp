// c++ includes
#include <iostream>
#include <algorithm>
#include <omp.h>
// project includes
#include "vehicles/tracked/tracked_vehicle.h"
#include "vehicles/tracked/tracked_render.h"
#include <sensors/mavs_sensors.h>
#include <raytracers/embree_tracer/embree_tracer.h>

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
	tracked_veh.SetInitialPose(0.0, 0.0, 0.0);


	//mavs::vehicle::tracked::TrackedRender tracked_debug_render(&tracked_veh);

    mavs::raytracer::embree::EmbreeTracer scene;
    scene.Load(scene_file);
    mavs::environment::Environment env;
	env.SetRaytracer(&scene);
	float theta = -1.570796f/4.0f;
	glm::vec3 sensor_offset(-8.0f, 3.0f, 2.0f);
	glm::quat sensor_orient(cosf(0.5f * theta), 0.0f, 0.0f, sinf(0.5f * theta));
	glm::vec3 position(0.0f, 0.0f, 1.0f);
	glm::quat orient(1.0f, 0.0f, 0.0f, 0.0f);
	mavs::sensor::camera::RgbCamera camera;
	camera.SetEnvironmentProperties(&env);
	camera.Initialize(960, 540, 0.0062222222f, 0.0035f, 0.0035f);
	camera.SetRelativePose(sensor_offset, sensor_orient);
	camera.SetName("camera");
	camera.SetPose(position, orient);
	camera.SetElectronics(0.95f, 1.0f);

	// do an initial step to load all the MAVS stuff
	tracked_veh.Update(&env, throttle, steering, braking, 0.0000001);

	// simulation setup 
	float dt = 0.01f; // 100 Hz
	int nsteps = 0;
	double t_total = 0.0;
	while (camera.DisplayOpen() || nsteps == 0) {

		UpdateDrivingCommand(camera.GetKeyCommands());
		double t0 = omp_get_wtime();
		tracked_veh.Update(&env, throttle, steering, braking, dt);
		
		t_total += omp_get_wtime() - t0;
		if (nsteps % 4 == 0) { // 25 Hz
			glm::dquat ori = tracked_veh.GetOrientation();
			camera.SetPose(tracked_veh.GetPosition(), tracked_veh.GetOrientation());
			camera.Update(&env, 0.03);
			camera.Display();
			
		}
		//if (nsteps % 10 == 0) tracked_debug_render.Update(); // 10 Hz
		
		nsteps++;
		
    }
	std::cout << "Simulated " << tracked_veh.GetElapsedTime() <<" seconds in " << t_total << " seconds of wall time " << std::endl;

}
