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
#ifndef ANVEL_VPRP_READER_H
#define ANVEL_VPRP_READER_H
/**
* \class AnvelVprpReader
*
* Reads in poses from an ANVEL .vprp file in text format
*
* \author Chris Goodin
*
* \date 8/1/2018
*/
#include <vector>

#include <mavs_core/messages.h>

namespace mavs{

class AnvelVprpReader{
 public:
	 /**
	 * Create a reader
	 */
  AnvelVprpReader(){};

	/**
	* Load poses from the input .vprp file
	* The .vprp file must be in text format
	* Returns a vector of Poses
	* \param fname The file name to be loaded
	* \param pf The pose frequency, will only use every pf^th pose
	* Set pf to 1 to use every pose
	*/
  std::vector<mavs::Pose> Load(std::string fname, int pf);
  
 private:
  int num_vehicle_objects_;
  std::vector<mavs::Pose> poses_;
};

} //namespace mavs

#endif