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
 * \file add_rain_to_existing_image.h
 *
 * Function to add rain to an esistin image
 *
 * \author Chris Goodin
 *
 * \date 5/31/2019
 */

#ifndef ADD_RAIN_TO_EXISTING_H
#define ADD_RAIN_TO_EXISTING_H

#include <CImg.h>
namespace mavs {
namespace sensor {
namespace camera {

void AddRainToImage(cimg_library::CImg<float> &image_, float rate, bool raindrops_on_lens);

void AddRainToImage(cimg_library::CImg<float> &image_, float rate, float rho, bool raindrops_on_lens);

} //namespace camera
} //namespace sensor
} //namespace mavs

#endif
