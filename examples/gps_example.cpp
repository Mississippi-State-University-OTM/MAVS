/*
MIT License

Copyright (c) 2024 Mississippi State University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
/**
* \file gps_example.cpp
*
* Example of the MAVS GPS simulation
*
* Usage: ./gps_example mavs_scene_file.json rp3d_vehicle_file.json surface_type cone_index pose_log_freq
*
* mavs_scene_file.json is a MAVS scene file, examples of which can
* be found in mavs/data/scenes.
*
* rp3d_vehicle_file.json examples can be found in mavs/data/vehicles/rp3d_vehicles
*
* Creates a differential and dual-band GPS and runs
* two iterations of each sensor. In the first simulation
* the GPS sensors are run in completely "open-field"
* conditions with no satellite occlusion or multipath
* errors. In the second, a simple scene with buildings
* surrounding the GPS units is created, causing
* multipath and dilution of precision errors.
* Correct errors should be on the order of 10s of centimeters
* for the dual band GPS and a few centimeters for the differential gps.
*
* \author Chris Goodin
*
* \date 7/30/2025
*/
#include <vehicles/rp3d_veh/mavs_rp3d_veh.h>
#include <raytracers/embree_tracer/embree_tracer.h>
#include <sensors/mavs_sensors.h>


mavs::sensor::camera::RgbCamera camera;
mavs::vehicle::Rp3dVehicle veh;
mavs::environment::Environment env;
mavs::raytracer::embree::EmbreeTracer scene;

static mavs::VehicleState UpdateVehicle(float dt);
static float LateralError(glm::vec3 a, glm::vec3 b);

int main(int argc, char* argv[]) {

	if (argc < 2) {
		std::cerr << "ERROR, must provide scene file and vehicle file as arguments" << std::endl;
		return 1;
	}
	std::string scene_file(argv[1]);
	std::string vehic_file(argv[2]);

	scene.Load(scene_file);

	env.SetTurbidity(9.0);
	env.SetFog(0.0f);
	env.SetDateTime(2023, 9, 1, 8, 0, 0, 6);
	env.SetRaytracer(&scene);

	camera.Initialize(480, 320, 0.00525f, 0.0035f, 0.0035f);
	camera.SetRelativePose(glm::vec3(-10.0f, 0.0f, 1.5f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	camera.SetPose(glm::vec3(0.0f, 0.0f, 1.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	camera.SetElectronics(0.65f, 1.0f);

	veh.Load(vehic_file);
	float zstart = scene.GetSurfaceHeight(0.0f, 0.0f);
	veh.SetPosition(0.0f, 0.0f, zstart + 0.25f);
	veh.SetOrientation(1.0f, 0.0f, 0.0f, 0.0f);

	mavs::sensor::gps::Gps gps, dual_band_gps, differential_gps;
	gps.SetType("normal");
	differential_gps.SetType("differential");
	dual_band_gps.SetType("dual band");
	differential_gps.SetRelativePose(glm::vec3(1.0f, 0.0f, 2.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	dual_band_gps.SetRelativePose(glm::vec3(1.0f, 0.0f, 2.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	gps.SetRelativePose(glm::vec3(1.0f, 0.0f, 2.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));

	int nsteps = 0;
	float elapsed_time = 0.0f;
	float dt = 0.01f;
	while (camera.DisplayOpen() || nsteps == 0) {
		// update vehicle
		mavs::VehicleState veh_state = UpdateVehicle(dt);

		if (nsteps % 100 == 0.0) {
			// update gps
			gps.SetPose(veh_state);
			gps.Update(&env, 1.0);
			dual_band_gps.SetPose(veh_state);
			dual_band_gps.Update(&env, 1.0);
			differential_gps.SetPose(veh_state);
			differential_gps.Update(&env, 1.0);

			glm::dvec3 p_diff = differential_gps.GetRecieverPositionENU();
			glm::dvec3 p_dual = dual_band_gps.GetRecieverPositionENU();
			glm::dvec3 p_std = gps.GetRecieverPositionENU();
			mavs::NavSatFix nav_sat_fix = gps.GetRosNavSatFix();
			mavs::NavSatStatus nav_sat_status = gps.GetRosNavSatStatus();
			std::cout << "Standard GPS measured position: (" << p_std.x << ", " << p_std.y << ", " << p_std.z << "), Error = " << LateralError(p_std, veh_state.pose.position) << ", Num Sats = " << dual_band_gps.GetNumSignals() << std::endl;
			std::cout << "GPS NavSatFix: (" << nav_sat_fix.longitude << ", " << nav_sat_fix.latitude << ", " << nav_sat_fix.altitude << ")" << std::endl;
			int gps_status = static_cast<int>(nav_sat_status.status);
			std::cout << "GPS NavSatStatuss: " << nav_sat_status.service << " " << gps_status << std::endl;
			std::cout << "Dual band measured position: (" << p_dual.x << ", " << p_dual.y << ", " << p_dual.z << "), Error = " << LateralError(p_dual, veh_state.pose.position) << ", Num Sats = " << dual_band_gps.GetNumSignals() << std::endl;
			std::cout << "Differential measured position: (" << p_diff.x << ", " << p_diff.y << ", " << p_diff.z << "), Error = " << LateralError(p_diff, veh_state.pose.position) << ", Num Sats = " << dual_band_gps.GetNumSignals() << std::endl << std::endl;
		}
		// update the camera
		if (nsteps % 4 == 0) {
			glm::vec3 look_to = veh.GetLookTo();
			float heading = 0.5f * (atan2f(look_to.y, look_to.x));
			camera.SetPose(veh.GetPosition(), glm::quat(cosf(heading), 0.0f, 0.0f, sinf(heading)));
			camera.Update(&env, 4 * dt);
			camera.Display();

		}

		nsteps++;
		elapsed_time += dt;
	}

	return 0;
}

static mavs::VehicleState UpdateVehicle(float dt) {
	double throttle = 0.0;
	double steering = 0.0;
	double braking = 0.0;
	std::vector<bool> driving_commands = camera.GetKeyCommands();
	if (driving_commands[0]) {
		throttle = 1.0;
	}
	else if (driving_commands[1]) {
		braking = 1.0;
	}
	if (driving_commands[2]) {
		steering = 1.0;
	}
	else if (driving_commands[3]) {
		steering = -1.0;
	}

	veh.Update(&env, (float)throttle, (float)steering, (float)braking, dt);
	mavs::VehicleState veh_state = veh.GetState();
	return veh_state;
}

static float LateralError(glm::vec3 a, glm::vec3 b) {
	float dx = a.x - b.x;
	float dy = a.y - b.y;
	float err = sqrtf(dx * dx + dy * dy);
	return err;
}