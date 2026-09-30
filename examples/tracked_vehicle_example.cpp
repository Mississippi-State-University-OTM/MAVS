// c++ includes
#include <iostream>
// project includes
#include "vehicles/tracked/tracked_vehicle.h"
#include "vehicles/tracked/tracked_render.h"
#include <sensors/mavs_sensors.h>
#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif


int main(int argc, char** argv) {

    std::string scene_file(argv[1]);
    std::string vehic_file(argv[2]);

	mavs::vehicle::tracked::TrackedVehicle tracked_veh(vehic_file);

	mavs::vehicle::tracked::TrackedRender render(&tracked_veh);

    mavs::raytracer::embree::EmbreeTracer scene;
    scene.Load(scene_file);
    mavs::environment::Environment env;
	env.SetRaytracer(&scene);

	glm::vec3 sensor_offset(-10.0f, 0.0f, 1.5f);
	glm::quat sensor_orient(1.0f, 0.0f, 0.0f, 0.0f);
	glm::vec3 position(0.0f, 0.0f, 1.0f);
	glm::quat orient(1.0f, 0.0f, 0.0f, 0.0f);
	mavs::sensor::camera::RgbCamera camera;
	camera.SetEnvironmentProperties(&env);
	camera.Initialize(480, 320, 0.00525f, 0.0035f, 0.0035f);
	camera.SetRelativePose(sensor_offset, sensor_orient);
	camera.SetName("camera");
	//camera.SetAntiAliasing("oversampled");
	//camera.SetPixelSampleFactor(3);
	camera.SetPose(position, orient);
	camera.SetElectronics(0.95f, 1.0f);

	int nsteps = 0;
	//while (camera.DisplayOpen() || nsteps == 0) {
	while (render.DisplayOpen() || nsteps == 0) {

		std::vector<bool> driving_commands = camera.GetKeyCommands();
		float throttle = 0.0f;
		float steering = 0.0f;
		float braking = 0.0f;
		driving_commands = camera.GetKeyCommands();
		if (driving_commands[0]) {
			throttle = 1.0f;
		}
		else if (driving_commands[1]) {
			braking = 1.0f;
		}
		if (driving_commands[2]) {
			steering = 1.0f;
		}
		else if (driving_commands[3]) {
			steering = -1.0f;
		}

        //mavs::vehicle::tracked::TrackSpeeds cmd = render.GetKeyboardDrivingCommand();

		//tracked_veh.Step(tracked_veh.GetSimulationDt(), cmd);
		tracked_veh.Update(&env, throttle, steering, braking, (float)tracked_veh.GetSimulationDt());
        render.Update();

		if (nsteps % 50 == 0) {
			glm::dquat ori = tracked_veh.GetOrientation();
			camera.SetPose(tracked_veh.GetPosition(), tracked_veh.GetOrientation());
			camera.Update(&env, 0.03);
			camera.Display();
		}
		//

		nsteps++;

    }

}
