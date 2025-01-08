/*
Non-Commercial License - Mississippi State University Autonomous Vehicle Software (MAVS)

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

*NOTICE*
Thank you for your interest in MAVS, we hope it serves you well!  We have a few small requests to ask of you that will help us continue to create innovative platforms:
-To share MAVS with a friend, please ask them to visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to register and download MAVS for free.
-Feel free to create derivative works for non-commercial purposes, i.e. academics, U.S. government, and industry research.  If commercial uses are intended, please contact us for a commercial license at otm@msstate.edu or jonathan.rudd@msstate.edu.
-Finally, please use the ACKNOWLEDGEMENT above in your derivative works.  This helps us both!

Please visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to view the full license, or email us if you have any questions.

Copyright 2018 (C) Mississippi State University
*/
/**
* \file utest_mavs_springtire.cpp
*
* Unit test to evaluate the load - deflection relationship of the spring tire.
* 
* Compare to curves from 
* "Multi-Wheeled Combat Vehicle Tire Modeling on Rigid and Soft Terrain"
* H. Ragheb, M. El-Gindy, H. A. Kishawy (2015)
*
* Usage: >./utest_mavs_springtire
*
* \author Chris Goodin
*
* \date 1/7/2021
*/
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <mavs_core/data_path.h>
#include <raytracers/embree_tracer/embree_tracer.h>
#include "vehicles/rp3d_veh/radial_spring_tire.h"

int main (int argc, char *argv[]){

	// Load the scene that the tires will be tested in
	mavs::environment::Environment env;
	mavs::MavsDataPath mavs_data_path;
	std::string scene_file(mavs_data_path.GetPath() + "/scenes/surface_only.json");
	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(scene_file);
	env.SetRaytracer(&scene);
	float surface_height = env.GetGroundHeight(0.0f, 0.0f);

	// Create the tires to test
	float tire_radius = 0.565f;
	float tire_width = 0.309f;
	float sidewall_constant = 150000.0f;
	float angular_res_deg = 2.5f;
	int nslices = 3;
	mavs::vehicle::radial_spring::Tire tire_4bar, tire_6bar, tire_8bar;
	tire_4bar.Initialize(tire_width, tire_radius, 400000.0f + sidewall_constant, angular_res_deg, nslices);
	tire_6bar.Initialize(tire_width, tire_radius, 600000.0f + sidewall_constant, angular_res_deg, nslices);
	tire_8bar.Initialize(tire_width, tire_radius, 800000.0f + sidewall_constant, angular_res_deg, nslices);

	float defl_step = 0.01f;
	float current_deflection = 0.0f; 
	std::cout << "Defl(mm) 4_bar(kN) 6_bar(kN) 8_bar(kN)" << std::endl;
	while (current_deflection <= 0.12f) {
		tire_4bar.SetPosition(glm::vec3(0.0f, 0.0f, surface_height + tire_radius - current_deflection));
		tire_6bar.SetPosition(glm::vec3(0.0f, 0.0f, surface_height + tire_radius - current_deflection));
		tire_8bar.SetPosition(glm::vec3(0.0f, 0.0f, surface_height + tire_radius - current_deflection));
		float fn_4 = tire_4bar.GetNormalForce(&env);
		float fn_6 = tire_6bar.GetNormalForce(&env);
		float fn_8 = tire_8bar.GetNormalForce(&env);
		std::cout << 1000.0f*current_deflection << " " << fn_4 / 1000.0f << " " << fn_6 / 1000.0f << " " << fn_8 / 1000.0f << std::endl;
		current_deflection += defl_step;
	}

  return 0;
}
