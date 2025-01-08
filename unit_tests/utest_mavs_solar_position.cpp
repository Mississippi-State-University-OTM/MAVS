/*
MIT License

Copyright (c) 2024 Mississippi State University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
/**
* \file utest_mavs_solar_position.cpp
*
* Unit test to evaluate the mavs solar position calculation
*
* Usage: >./utest_mavs_solar_position
*
* Check the solar position calculator, comparing results to values from
* the NOAA solar calculator.
* https://www.esrl.noaa.gov/gmd/grad/solcalc/azel.html
* for 1 Jan 2000 at 1800 UT (1200 CST) , 32 Lat, 90 Long, time zone = 6
*
* The values from the web page are "solar elevation = 35, solar azimuth = 179.03"
* The algorithm in MAVS for solar position is slightly less precise than the NOAA page,
* so there will be some differences.
*
* \author Chris Goodin
*
* \date 10/4/2018
*/

#include "mavs_core/environment/solar_position.h"
#include <iostream>

int main(int argc, char *argv[]){
	mavs::environment::SolarPosition sunpos;

  std::cout.precision(10);
  mavs::environment::DateTime check_day;
  check_day.year = 2000;
  check_day.month = 1;
  check_day.day = 1;
  check_day.hour = 12;
  check_day.time_zone = 6;
  check_day.minute = 0;
  check_day.second = 0;
	sunpos.SetDateTime(check_day);
  //Location check_position;
  float lat = 32.0f;
  float lon = 90.0f;
  float deg2rad = 0.01745329252f;
	sunpos.SetLocalLatLong(lat, lon);
  mavs::environment::HorizontalCoordinate checkpos = sunpos.GetSolarPosition();
  std::cout<<"Solar Elevation (error) = "<<90-checkpos.zenith/deg2rad<<" ("<<
    (90-checkpos.zenith/deg2rad)-35<<")"<<std::endl;
  std::cout<<"Solar Azimuth (error) = "<<checkpos.azimuth/deg2rad<<" ("<<
    checkpos.azimuth/deg2rad-179.03<<")"<<std::endl;
  
  return 0;
}
