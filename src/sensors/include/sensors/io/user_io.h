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
* \file user_io.h
*
* Definition for functions that get user input
* through dialogs
*
* \author Chris Goodin
*
* \date 2/27/2019
*/
#ifndef MAVS_USER_IO
#define MAVS_USER_IO

#include <string>

namespace mavs {
namespace io {

std::string GetInputFileName(std::string message, std::string ftype);

std::string GetSaveFileName(std::string message, std::string default_type, std::string ftype);

float GetUserNumericInput(std::string title, std::string message);

bool GetUserBool(std::string question, std::string message);

void DisplayUserInfo(std::string title, std::string message);

} //namespace mavs
} //namespace io
#endif