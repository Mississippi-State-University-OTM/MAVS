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
* \file utest_mavs_rtk.cpp
*
* Unit test to evaluate the mavs rtk sensor
*
* Usage: >./utest_mavs_rtk (output_type) (max_error) (droput_rate) 
* 
* where 
* output_type can be either "all" or "position"
* max_error is the maximum error in meters and
* droput_rate is the # of gps dropouts/hour
*
* Creates an RTK sensor that uses an empirical error model
* and runs it for four hours. Prints the position error to stdout.
*
* \author Chris Goodin
*
* \date 4/29/2019
*/
#include <stdlib.h>
#include <sensors/rtk/rtk.h>

int main (int argc, char *argv[]){
  
  mavs::environment::Environment env;
  env.SetLocalOrigin(32.6526,-90.8779,73.152); //Vicksburg, MS
  
  float err = 0.35f;
  float drop_rate = 3.0f;
  std::string output_type = "position";
  if (argc>1){
    std::string out_type(argv[1]);
    if (out_type=="all" || out_type=="position"){
      output_type = out_type;
    }
  }
  if (argc>2){
    err = (float)atof(argv[2]);
  }
  if (argc>3){
    drop_rate = (float)atof(argv[3]);
  }


  mavs::VehicleState veh_state;
  veh_state.pose.position.x = 5.0;
  veh_state.pose.position.y = 10.0;
  veh_state.pose.position.z = 2.0;
  
  mavs::sensor::rtk::Rtk rtk;
  rtk.SetError(err);
  rtk.SetDropoutRate(drop_rate);
	rtk.SetWarmupTime(600.0f);

  float t = 0.0f;
  float dt = 0.05f;
  float path_radius = 15.0f;
  while (t<14400.0f){
    veh_state.pose.position.x = path_radius*cos(t/100.0);
    veh_state.pose.position.y = path_radius*sin(2.0*t/100.0);
    rtk.SetPose(veh_state);
	  rtk.Update(&env, dt);
    rtk.PrintCurrentOdometry(output_type);
    t += dt;
  }
  
  return 0;
}
