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
* \file radar_models.h
*
* Declares several default radar models
*
* \author Chris Goodin
*
* \date 6/27/2018
*/
#ifndef RADAR_MODELS_H
#define RADAR_MODELS_H

#include <sensors/radar/radar.h>

namespace mavs {
namespace sensor {
namespace radar {

/**
* \class DelphiMidRange
*
* Delphi ESR, mid-range mode
* https://www.autonomoustuff.com/wp-content/uploads/2016/08/delphi-esr.pdf
*/
class DelphiMidRange : public Radar {
public:
	/// Create Delphi mid-range radar
	DelphiMidRange() {
		// FOV, divergence, sample step 
		//Initialize(90.0,3.0,0.25);
		Initialize(90.0f, 1.0f, 0.25f);
		max_range_ = 60.0f;
	}
};

/**
* \class DelphiLongRange
*
* Delphi ESR, long-range mode
* https://www.autonomoustuff.com/wp-content/uploads/2016/08/delphi-esr.pdf
*/
class DelphiLongRange : public Radar {
public:
	/// Create Delphi long-range radar
	DelphiLongRange() {
		// FOV, divergence, sample step 
		//Initialize(20.0, 3.0, 0.25);
		Initialize(20.0f, 1.0f, 0.25f);
		max_range_ = 174.0f;
	}
};



} //namespace radar
} //namespace sensor
} //namespace mavs

#endif