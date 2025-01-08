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
#include <sensors/lidar/os1.h>
#include <mavs_core/math/utils.h>

namespace mavs{
namespace sensor{
namespace lidar{

OusterOS1::OusterOS1(){
	SetBeamSpotCircular((float)(0.18*mavs::kDegToRad));
  SetRotationRate(10.0,1024);
  max_range_ = 125.0;
  min_range_ = 0.5;
}

void OusterOS1::SetRotationRate(float rot_hz, int mode){
	if (rot_hz < 10.0)rot_hz = 10.0;
	if (rot_hz > 20.0)rot_hz = 20.0;
	if (rot_hz != 10.0 && rot_hz != 20.0) {
		if (rot_hz < 15.0) {
			rot_hz = 10.0;
		}
		else {
			rot_hz = 20.0;
		}
	}
	if (mode == 2048) rot_hz = 10.0;
	if (mode != 512 && mode != 1024 && mode != 2048) {
		std::cerr << "ERROR in setting mode of OS1 lidar." << std::endl;;
		std::cerr << "Mode must be 512,1024, or 2048" << std::endl;
		exit(10);
	}
	rotation_rate_ = rot_hz;
	float res = 360.0f / (1.0f*mode);
	//float vres = 33.2f / 64.0f;
	float vres = 31.6f/63.0f;
  //SetScanPattern(-180.0f,180.0f-res,res,-16.6f,16.6f,vres);
  SetScanPattern(-180.0f,180.0f-res,res,-15.8f,15.8f,vres);
  SetTimeStep(1.0/rot_hz);
}

} //namespace lidar
} //namespace sensor
} //namespace mavs
