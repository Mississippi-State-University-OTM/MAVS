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
* \file json_io.h
*
* Definition for functions that load / write
* json blocks that are re-used in several types
* of mavs input files.
*
* \author Chris Goodin
*
* \date 8/2/2018
*/
#ifndef MAVS_JSON_IO
#define MAVS_JSON_IO
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>

#include <sensors/sensor.h>

#include <vector>

namespace mavs {
namespace io {

/**
* Load a sensor entry block and returna a pointer to the sensor
* \param sensor Rapidjson value of the sensor object
*/
sensor::Sensor *LoadSensor(const rapidjson::Value &sensor);

/**
* Load a full list of sensors and modify the input array
* \param d The rapidjson document to parse
* \param sensors The output list of sensors
*/
void LoadSensorBlock(const rapidjson::Document& d, 
	std::vector<sensor::Sensor*> &sensors);

} //namespace mavs
} //namespace io
#endif