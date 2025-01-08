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
* \class Rtk
*
* Class for a MAVS Rtk sensor
* 
* Based on the following papers
*
* Skoglund, Martin, et al. 
* "Static and dynamic performance evaluation of low-cost RTK GPS receivers." 
* 2016 IEEE Intelligent Vehicles Symposium (IV). IEEE, 2016.
*
* Mahmoud, Tareg, and Bambang Riyanto Trilaksono. 
* "Integrated INS/GPS Navigation System." 
* International Journal on Electrical Engineering and Informatics 10.3 (2018): 491-512.
*
* Xiang, Y. I. N., et al. 
*"Development of an automatically guided rice transplanter using RTK-GNSS and IMU." 
* IFAC-PapersOnLine 51.17 (2018): 374-378.
*
* Scherzinger, Bruno M. 
* "Precise robust positioning with inertial/GPS RTK." 
* Proceedings of the 13th International Technical Meeting fo the Satellite Division of the Institute of Navigation (ION GPS). 2000.
*
* \author Chris Goodin
*
* \date 4/26/2019
*/
#ifndef MAVS_RTK_H
#define MAVS_RTK_H

#include <string>
#include <sensors/sensor.h>
#include <mavs_core/environment/environment.h>
#include <mavs_core/messages.h>
#include <FastNoise.h>

namespace mavs {
namespace sensor {
namespace rtk {
class Rtk : public Sensor {
public:
	/// Create Rtk sensor
	Rtk();

	/**
	 * Update one step of the RTK system
	 * \param env The mavs environment
	 * \param dt The time step in seconds 
	 */
	void Update(environment::Environment *env, double dt);

	/** 
	 * Return the odometry message of the
	 * piksi. In ENU relative to the base station
	 */
	mavs::Odometry GetOdometryMessage();

	/**
	 * Print current odemetry information to stdout
	 * \param err_to_print Can be "all" or "position" 
	 */
	void PrintCurrentOdometry(std::string err_to_print);

	/**
	 * Set the maximum (pre-warmup) positioning error of the system in meters.
	 * The RMS error will be about 20% of this value
	 * \param err The desired max error in meters.
	 */
	void SetError(float err){
		error_ = err;
	}

	/**
	 * Set the dropout rate in num/hour
	 * \param rate The number of gps dropouts per hour 
	 */
	void SetDropoutRate(float rate);

	/**
	 * Start the sim certain number of seconds into the RTK turn on.
	 * RTK takes about 10 minutes to get best localization. Can start
	 * system at t=wut_seconds to get good solutions
	 * \param wut_seconds The seconds of warmup time to elapse 
	 */
	void SetWarmupTime(float wut_seconds){
		rtk_elapsed_time_ = wut_seconds;
	}

	/// Return true if the GPS dropped out, false if not
	bool DroppedOut() { return dropped_out_; }

protected:
	mavs::Odometry odom_;
	float error_;
	float rtk_elapsed_time_;
	glm::vec3 offset_direction_;
	FastNoise random_noise_;
	FastNoise pop_noise_;
	bool dropped_out_;
	float drop_time_elapsed_;
}; // class Rtk

} //namespace Rtk
} //namespace halo
} //namespace mavs


#endif