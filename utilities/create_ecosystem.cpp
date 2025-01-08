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
* \file create_ecosystem.cpp
*
* Create a random ecosystem
*
* Usage: >./create_ecosystem random_scene_inputs.json
*
* where eco_file.json is random scene definition file.
* An example is in mavs/data/ecosystem_files/random_scene_inputs.json
*
* Will create a surface file mesh and associated scene file.
* 
*
* \author Chris Goodin
*
* \date 10/4/2018
*/
#include <iostream>
#include <string>
#include <mavs_core/terrain_generator/random_scene.h>

int main(int argc, char *argv[]) {

	if (argc < 2) {
		std::cerr << "ERROR: must list random scene input file as command line argument" << std::endl;
		return 1;
	}
	std::string infile(argv[1]);

	mavs::terraingen::RandomScene scene;
	scene.Load(infile);
	scene.Create();

	return 0;
}

