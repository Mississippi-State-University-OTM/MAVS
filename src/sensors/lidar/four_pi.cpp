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
#include <sensors/lidar/four_pi.h>
#include <limits>
#include <mavs_core/math/utils.h>

namespace mavs{
namespace sensor{
namespace lidar{

FourPiLidar::FourPiLidar(){
	angle_res_ = 0.25f; //degrees
	InitFourPi(angle_res_);
}

FourPiLidar::FourPiLidar(float res) {
	angle_res_ = res;
	InitFourPi(angle_res_);
}
void FourPiLidar::InitFourPi(float res){
	SetBeamSpotRectangular(0.0f, 0.0f);
	max_range_ = 1000.0f; // std::numeric_limits<float>::max();
	min_range_ = 0.0f;

  SetScanPattern(-180.0f,180.0f-res,res,-90.0f,90.0f,res);
  SetTimeStep(0.1);
}

} //namespace lidar
} //namespace sensor
} //namespace mavs
