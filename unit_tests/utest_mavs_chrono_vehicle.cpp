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
* \file utest_mavs_chrono_vehicle.cpp
*
* Unit test to evaluate the mavs chrono vehicle interface
*
* Usage: >./utest_mavs_chrono_vehicle chrono_vehicle_input_file.json
* 
* where chrono_vehicle_input_file.json gives the path to the chrono
* data directory and the different vehicle configurations to use.
* An example can be found in mavs/data/vehicles/chrono_inputs/hmmwv_windows.json
*
* The simulation will run for 15 seconds of simulated time, printing a state update
* every second. Vehicle drives in a straight line and should reach a maximum speed of around
* 38.9 m/s
*
* \author Chris Goodin
*
* \date 10/4/2018
*/
#include <iostream>
#include <vehicles/chrono/chrono_wheeled_json.h>
#include <mavs_core/environment/environment.h>

int main (int argc, char *argv[]){

	if (argc < 2) {
		std::cerr << "Usage: $./utest_mavs_chrono_vehicle chrono_vehicle_input_file.json " << std::endl;
		return 1;
	}
	
	std::string chrono_infile(argv[1]);

	mavs::vehicle::Vehicle *vehicle = new mavs::vehicle::ChronoWheeledJson;
	vehicle->SetPosition(0.0, 0.0, 2.0);
	vehicle->SetOrientation(1.0, 0.0, 0.0, 0.0);
	vehicle->Load(chrono_infile);

	mavs::environment::Environment env;

	float time = 0.0f;
	float dt = 1.0E-3f;
	float throttle = 0.0f;
	float steering = 0.0f;
	int nsteps = 0;
	while (time < 15.0f) {
		if (time > 5.0f) {
		  throttle = time - 5.0f;
			if (throttle > 1.0f)throttle = 1.0f;
		}

		vehicle->Update(&env, throttle, steering, 0.0f, dt);

		mavs::VehicleState state = vehicle->GetState();
		float velocity = (float)sqrt(state.twist.linear.x*state.twist.linear.x + state.twist.linear.y*state.twist.linear.y);
		if (nsteps % 1000 == 0) {
			std::cout << "Time = " << time << ", vehicle at position (" << state.pose.position.x << ", " << state.pose.position.y <<
				"), velocity = " << velocity << std::endl;
		}
		time += dt;
		nsteps++;
	}
	delete vehicle;
  return 0;
}

