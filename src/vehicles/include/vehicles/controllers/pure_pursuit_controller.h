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
* \class PurePursuitController
*
* A pure-pursuit vehicle control that follows an input vehicle
* trajectory in 2D space. 
* 
* See "Implementation of the Pure Pursuit Path Tracking Algorithm"
* by Craig Coulter, CMU-RI-TR-92-01
* 
* and
* 
* "Automatic Steering Methods for Autonomous Automobile Path Tracking"
* by Jarrod M. Snider, CMU-RI-TR-09-08
*
* \author Chris Goodin
*
* \date 12/3/2018
*/
#ifndef PURE_PURSUIT_CONTROLLER_H
#define PURE_PURSUIT_CONTROLLER_H

//mavs includes
#include <vehicles/controllers/pid_controller.h>
#include <glm/glm.hpp>
#include <mavs_core/messages.h>

namespace mavs {
namespace vehicle{
class PurePursuitController {
public:
	/// Create a controller
	PurePursuitController();

	/**
	* Calculate a driving command based on a trajectory
	* The first point on the trajectory must be the current
	* vehicle state.
	* \param traj The desired trajectory
	*/
	//Twist GetDcFromTraj(Path traj);
	void GetDrivingCommand(float &throttle, float &steering, float &braking, float dt);

	/**
	* Set the desired path as a sequence or 2D waypoints
	* \param path The path in local ENU coordinates
	*/
	void SetDesiredPath(std::vector<glm::vec2> path) { path_ = path; }

	/**
	* Set the wheelbase of the vehicle in meters
	* \param wb Wheelbase to set
	*/
	void SetWheelbase(float wb) { wheelbase_ = wb; }

	/**
	* Set the max steering angle of the vehicle in radians
	* \param st Max steering angle
	*/
	void SetMaxSteering(float st) { max_steering_angle_ = st; }

	/** 
	* Set the minimum look-ahead distance of the planner, in meters
	* \param min_la The minimum look-ahead distance
	*/
	void SetMinLookAhead(float min_la) { min_lookahead_ = min_la; }

	/**
	* Set the maximum look-ahead distance of the planner, in meters
	* \param max_la The maximum look-ahead distance
	*/
	void SetMaxLookAhead(float max_la) { max_lookahead_ = max_la; }

	/** 
	* Set the gain factor on the steering controller
	* \param k Desired gain factor
	*/
	void SetSteeringParam(float k) { k_ = k; }

	/**
	* Set the maximum allowed speed of the vehicle
	* Contoller will limit throttle to stay below this speed
	* \param speed Maximum desired speed in m/s
	*/
	void SetMaxStableSpeed(float speed) { max_stable_speed_ = speed; }

	/**
	* Set the desired speed of the vehicle in m/s
	* \param speed The desired speed
	*/
	void SetDesiredSpeed(float speed) {
		desired_speed_ = speed;
		speed_controller_.SetSetpoint(speed);
		max_stable_speed_ = 1.5f*speed;
	}

	/**
	* Set the coefficients of the PID speed controller
	* \param kp Proportional coefficient
	* \param ki Integral coefficient
	* \param kd Derivative coefficient
	*/
	void SetSpeedControllerParams(float kp, float ki, float kd) {
		speed_controller_.SetKp(kp);
		speed_controller_.SetKi(ki);
		speed_controller_.SetKd(kd);
	}

	/**
	* Set the current vehicle position in local ENU
	* \param x Current x-coordinate in ENU
	* \param y Current y-coordinate in ENU
	*/
	void SetVehiclePosition(float x, float y) {
		veh_x_ = x;
		veh_y_ = y;
	}

	/**
	* Set the current vehicle speed in m/s
	* \param speed The current vehicle speed 
	*/
	void SetVehicleSpeed(float speed) {
		veh_speed_ = speed;
	}

	/**
	* Set the current vehicle heading in radians
	* \param heading The current vehicle heading
	*/
	void SetVehicleOrientation(float heading) {
		veh_heading_ = heading;
	}

	/**
	 *  Set the vehicle position, orientation and speed
	 * \param state The vehicle state
	 */
	void SetVehicleState(Odometry state);

	/**
	*  Set the current position, orientation and speed
	* \param x_pos The x-position in local ENU meters
	* \param y_pos The y-position in local ENU meters
	* \param speed The vehicle speed in the horizontal plane
	* \param heading The vehicle heading in radians from +x
	*/
	void SetVehicleState(float x_pos, float y_pos, float speed, float heading);

	/**
	* Set the distance for which vehicle must approach the final waypoint
	* \param thresh Distance in meters
	*/
	void SetGoalThreshhold(float thresh) { goal_thresh_ = thresh; }

	/**
	* Call this to have the vehicle loop over the path
	*/
	void TurnOnLooping() { looping_ = true; }

	/// Return if the controller is finished with the path
	bool GetIsComplete() { return complete_; }

	/// Set the complete status
	void SetIsComplete(bool complete) { complete_ = complete; }

private:
	std::vector<glm::vec2> path_;
	float wheelbase_; //meters
	float max_steering_angle_; //radians
	float min_lookahead_; //meters
	float max_lookahead_; //meters
	float k_; //unitless
	float desired_speed_; // m/s
	float max_stable_speed_;
	PidController speed_controller_;
	float goal_thresh_;
	bool complete_;
	bool looping_;
	float last_throttle_;
	float max_throttle_rate_;

	//current vehicle state info
	float veh_x_;
	float veh_y_;
	float veh_heading_;
	float veh_speed_;
};
} //namespace vehicle
} //namespace mavs

#endif
