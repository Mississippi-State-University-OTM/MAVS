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
 * \file constants.h
 * 
 * Defines some useful math constants. 
 *
 * \author Chris Goodin
 *
 * \date 12/13/2017
 */

#ifndef MAVS_CONSTANTS_H
#define MAVS_CONSTANTS_H

#include <glm/glm.hpp>

namespace mavs{

const double kPi = 3.141592653589793238462643383279;
const double kPi_2 = 1.570796326794896619231321691639;
const double kPi_4 = 0.785398163397448309615660845819;
const double k1_Pi = 0.318309886183790671537767526745;
const double k2Pi = 6.283185307179586476925286766559;
const double kRadToDeg = 180.0 / 3.1415926535897932384626433832795;
const double kDegToRad = 3.1415926535897932384626433832795 / 180.0;
const double kSqrt2 = 1.41421356237309504880;
const double kGravity = 9.80665;
const glm::vec3 zero_vec3(0.0f,0.0f,0.0f);
const glm::vec3 one_vec3(1.0f,1.0f,1.0f);
} //namespace mavs

#endif
