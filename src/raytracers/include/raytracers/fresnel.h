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
 * \file fresnel.h
 *
 * Interfaces to Fresnel equations
 *
 * \author Chris Goodin
 *
 * \date 2/3/2020
 */
#ifndef FRESNEL_H
#define FRESNEL_H

#include <glm/glm.hpp>

namespace mavs{
namespace raytracer{

float GetFresnelTransmissionAngle(float n1, float n2, float theta_i);

glm::vec4 GetFresnelCoeffs(float n1, float n2, float theta_i, float theta_t);

float GetFresnelReflectance(float n1, float n2, float theta_i, float theta_t);

glm::vec2 GetPolarizedFresnelReflectance(float n1, float n2, float theta_i, float theta_t);

} //namespace raytracer
} //namespace mavs

#endif
