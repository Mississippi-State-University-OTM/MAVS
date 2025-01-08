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
* \class UavController
*
* Class that implements UAV control using PID for altitude and speed.
*
* \author Chris Goodin
*
* \date 11/18/2024
*/

#ifndef MAVS_UAV_CONTROLLER_H
#define MAVS_UAV_CONTROLLER_H
// mavs includes
#include "vehicles/uav/flight_control.h"
#include "vehicles/uav/uav.h"
#include "vehicles/controllers/pid_controller.h"
#include "vehicles/controllers/pure_pursuit_controller.h"

namespace mavs {
namespace vehicle {
class UavController {
public:
	UavController();

	FlightControl UpdateControl(Uav* uav, float dt);

	void SetDesiredSpeed(float speed_ms) { speed_pid_.SetSetpoint(speed_ms); }

	void SetDesiredAltitude(float alt_m) { alt_pid_.SetSetpoint(alt_m); }

	void SetSpeedControllerParams(float p, float i, float d);

	void SetAltitudeControllerParams(float p, float i, float d);

	void SetWaypoints(std::vector<glm::vec2> waypoints) {
		roll_control_.SetDesiredPath(waypoints);
		waypoints_ = waypoints;
	}

	std::vector<glm::vec2> GetWaypoints() { return waypoints_; }

	void SetMaxRollRadians(float mr) { max_roll_ = mr; }

	void SetMaxLookahead(float mla) { roll_control_.SetMaxLookAhead(mla); }
	void SetMinLookahead(float mla) { roll_control_.SetMinLookAhead(mla); }
	void SetSteeringCoeff(float k) { roll_control_.SetSteeringParam(k); }
	void SetGoalThresh(float gt) { roll_control_.SetGoalThreshhold(gt); }

private:
	mavs::vehicle::PidController speed_pid_;
	mavs::vehicle::PidController alt_pid_;
	mavs::vehicle::PurePursuitController roll_control_;
	std::vector<glm::vec2> waypoints_;
	float max_roll_;
}; // class UavController
} // namespace uav
} //namespace uav

#endif // include gaurd