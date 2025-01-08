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
* \file drop_test_example.cpp
*
* Implentation of a MAVS drop test for measuring a vehicles spring rate
*
* Usage: >./drop_test_example rp3d_vehicle_file.json
*
* rp3d_vehicle_file.json examples can be found in mavs/data/vehicles/rp3d_vehicles
*
* \author Chris Goodin
*
* \date 10/4/2024
*/
#include <mavs_core/data_path.h>
#include "mavs_core/math/curve_fitter.h"
#include <vehicles/rp3d_veh/mavs_rp3d_veh.h>
#include <sensors/mavs_sensors.h>
#include <raytracers/embree_tracer/embree_tracer.h>

static double DampedOscillator(double x, std::vector<double> p);

static std::vector<double> DampedOscillatorJacob(double x, std::vector<double> param);

int main(int argc, char *argv[]) {
	// check that vehicle file is specified
	if (argc < 2) {
		std::cerr << "ERROR, must provide vehicle file as argument" << std::endl;
		return 1;
	}

	// set the files to load
	mavs::MavsDataPath mdp;
	std::string data_path =  mdp.GetPath();
	std::string scene_file = data_path + "/scenes/" + "surface_only.json";
	std::string vehic_file(argv[1]);
	std::vector<double> front_time, front_disp, rear_time, rear_disp;
	double front_load_newtons = 0.0;
	double rear_load_newtons = 0.0;
	// do the front axle drop
	{
		// create the scene and load it
		mavs::raytracer::embree::EmbreeTracer scene;
		scene.Load(scene_file);

		// create the environment and add the scene
		mavs::environment::Environment env;
		env.SetRaytracer(&scene);

		// create the vehicle and load it
		mavs::vehicle::Rp3dVehicle veh;
		veh.Load(vehic_file);
		veh.SetPosition(0.0f, 0.0f, 1.0f);
		veh.SetOrientation(1.0f, 0.0f, 0.0f, 0.0f);

		// get some properties of the vehicle
		veh.Update(&env, 0.0f, 0.0f, 0.0f, 0.01f);
		float wheelbase = veh.GetTirePosition(0).y - veh.GetTirePosition(1).y;
		float tracklength = veh.GetTirePosition(0).x - veh.GetTirePosition(2).x;
		// now place the vehicle and let it settle
		for (int i = 0; i < 500; i++) veh.Update(&env, 0.0f, 0.0f, 0.0f, 0.01f);
		float front_axle_height = veh.GetTirePosition(0).z;
		// now compress the front axle
		float compress_height = front_axle_height - 0.25f * veh.GetTire(0)->GetRadius();
		int niter = 0;
		while (veh.GetTirePosition(0).z > compress_height && niter<1000) {
			glm::vec3 point = 0.5f * (veh.GetTirePosition(0) + veh.GetTirePosition(1));
			glm::vec3 force(0.0f, 0.0f, -1.5f*veh.GetChassis()->GetBody()->getMass() * 9.8f);
			veh.SetExternalForceAtPoint(force, point);
			veh.Update(&env, 0.0f, 0.0f, 0.0f, 0.01f);
			niter++;
		}

		// zero out the external force
		veh.SetExternalForceAtPoint(glm::vec3(0.0f, 0.0f, 0.0f), veh.GetPosition());

		// create the camera
		glm::vec3 cam_pos(veh.GetTirePosition(0).x, 2.0f * wheelbase, veh.GetTire(0)->GetRadius());
		glm::quat cam_ori(cosf(-0.25f * mavs::kPi), 0.0f, 0.0f, sinf(-0.25f * mavs::kPi));
		mavs::sensor::camera::RgbCamera camera;
		camera.Initialize(480, 320, 0.00525f, 0.0035f, 0.0035f);
		camera.SetElectronics(0.85f, 1.0f);
		camera.SetPose(cam_pos, cam_ori);
		camera.Update(&env, 0.02f);
		camera.Display();

		// do the drop
		float dt = 0.01f;
		float elapsed_time = 0.0f;
		float dh = 1000.0f;
		float last_z = veh.GetTirePosition(0).z;
		float last_defl = 0.0f;
		bool logging = false;
		while (camera.DisplayOpen() && dh > 1.0E-6f) {

			last_defl = veh.GetTire(0)->GetCurrentDeflection();
			
			front_load_newtons = veh.GetTireForces(0).z;
			front_time.push_back((double)(elapsed_time));
			front_disp.push_back((double)(veh.GetTirePosition(0).z - front_axle_height));

			veh.Update(&env, 0.0f, 0.0f, 0.0f, dt);
			camera.Update(&env, 0.2f);
			camera.Display();
			dh = std::fabs(veh.GetTirePosition(0).z - last_z);
			last_z = veh.GetTirePosition(0).z;

			elapsed_time += dt;
		}
	} // finished front axle
	
	// do the rear axle drop
	{
		// create the scene and load it
		mavs::raytracer::embree::EmbreeTracer scene;
		scene.Load(scene_file);

		// create the environment and add the scene
		mavs::environment::Environment env;
		env.SetRaytracer(&scene);

		// create the vehicle and load it
		mavs::vehicle::Rp3dVehicle veh;
		veh.Load(vehic_file);
		veh.SetPosition(0.0f, 0.0f, 1.0f);
		veh.SetOrientation(1.0f, 0.0f, 0.0f, 0.0f);

		// get some properties of the vehicle
		veh.Update(&env, 0.0f, 0.0f, 0.0f, 0.01f);
		float wheelbase = veh.GetTirePosition(0).y - veh.GetTirePosition(1).y;
		float tracklength = veh.GetTirePosition(0).x - veh.GetTirePosition(2).x;
		// now place the vehicle and let it settle
		for (int i = 0; i < 500; i++) veh.Update(&env, 0.0f, 0.0f, 0.0f, 0.01f);
		float rear_axle_height = veh.GetTirePosition(2).z;
		// compress the front axle
		float compress_height = rear_axle_height - 0.25f * veh.GetTire(2)->GetRadius();
		int niter = 0;
		while (veh.GetTirePosition(2).z > compress_height && niter < 1000) {
			glm::vec3 point = 0.5f * (veh.GetTirePosition(2) + veh.GetTirePosition(3));
			glm::vec3 force(0.0f, 0.0f, -1.5f * veh.GetChassis()->GetBody()->getMass() * 9.8f);
			veh.SetExternalForceAtPoint(force, point);
			veh.Update(&env, 0.0f, 0.0f, 0.0f, 0.01f);
			niter++;
		}
		// zero out the external force
		veh.SetExternalForceAtPoint(glm::vec3(0.0f, 0.0f, 0.0f), veh.GetPosition());

		// create the camera
		glm::vec3 cam_pos(veh.GetTirePosition(2).x, 2.0f * wheelbase, veh.GetTire(2)->GetRadius());
		glm::quat cam_ori(cosf(-0.25f * mavs::kPi), 0.0f, 0.0f, sinf(-0.25f * mavs::kPi));
		mavs::sensor::camera::RgbCamera camera;
		camera.Initialize(480, 320, 0.00525f, 0.0035f, 0.0035f);
		camera.SetElectronics(0.85f, 1.0f);
		camera.SetPose(cam_pos, cam_ori);
		camera.Update(&env, 0.02f);
		camera.Display();

		// do the drop
		float dt = 0.01f;
		float elapsed_time = 0.0f;
		float dh = 1000.0f;
		float last_z = veh.GetTirePosition(2).z;
		float last_defl = 0.0f;
		bool logging = false;
		while (camera.DisplayOpen() && dh > 1.0E-6f) {

			last_defl = veh.GetTire(2)->GetCurrentDeflection();

			rear_load_newtons = veh.GetTireForces(2).z;
			rear_time.push_back((double)(elapsed_time));
			rear_disp.push_back((double)(veh.GetTirePosition(2).z - rear_axle_height));

			veh.Update(&env, 0.0f, 0.0f, 0.0f, dt);
			camera.Update(&env, 0.2f);
			camera.Display();
			dh = std::fabs(veh.GetTirePosition(2).z - last_z);
			last_z = veh.GetTirePosition(2).z;

			elapsed_time += dt;
		}
	}

	// now fit the results
	double front_mass = front_load_newtons / 9.806;
	double rear_mass = rear_load_newtons / 9.806;
	
	std::cout << "Fitting model of form: a*cos(b*x + pi)*exp(-c*x)" << std::endl;
	mavs::math::curve_fit::CurveFitter solver;
	solver.SetRmseThreshold(0.0005);
	solver.SetMaxIterations(15000);
	std::vector<double> p_guess{ 0.01, 30.0, 8.0 };
	std::vector<double> front_params = solver.Solve(&DampedOscillator, &DampedOscillatorJacob, front_time, front_disp, p_guess);
	std::cout << "Front Axle: " << std::endl;
	solver.PrintResults();
	solver.PrintFit();

	double omega = front_params[1];
	double gamma = front_params[2];
	double k_front = front_mass * (omega * omega - gamma * gamma);
	double c_front = 2.0 * front_mass * sqrt(k_front / front_mass) * gamma;
	std::cout << "Front Spring Constant: " << k_front << std::endl;
	std::cout << "Front Damping Constant: " << c_front << std::endl;
	std::cout << std::endl;

	solver.Reset();
	solver.SetRmseThreshold(0.0005);
	solver.SetMaxIterations(15000);
	std::vector<double> rear_params = solver.Solve(&DampedOscillator, &DampedOscillatorJacob, rear_time, rear_disp, p_guess);
	std::cout << "Rear Axle: " << std::endl;
	solver.PrintResults();
	solver.PrintFit();
	omega = rear_params[1];
	gamma = rear_params[2];
	double k_rear = rear_mass * (omega * omega - gamma * gamma);
	double c_rear = 2.0 * rear_mass * sqrt(k_rear / rear_mass) * gamma;
	std::cout << "Rear Spring Constant: " << k_rear << std::endl;
	std::cout << "Rear Damping Constant: " << c_rear << std::endl;
	std::cout << std::endl;

	return 0;
}

static double DampedOscillator(double x, std::vector<double> p) {
	double a = p[0];
	double b = p[1];
	double c = p[2];
	double y = a * exp(-c * x) * cos(b * x + mavs::kPi);
	return y;
}

static std::vector<double> DampedOscillatorJacob(double x, std::vector<double> param) {
	double a = param[0]; // amplitude
	double b = param[1]; // frequency
	double c = param[2]; // damping 
	std::vector<double> j = param;
	j[0] = -exp(-c * x) * cos(b * x);
	j[1] = a * x * exp(-c * x) * sin(b * x);
	j[2] = a * x * exp(-c * x) * cos(b * x);
	return j;
}