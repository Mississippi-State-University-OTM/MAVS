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
#include <sensors/lidar/vlp16.h>

#include <mavs_core/math/utils.h>

namespace mavs{
namespace sensor{
namespace lidar{

Vlp16::Vlp16(){
  SetBeamSpotRectangular(0.0033f, 0.0007f);
  SetRotationRate(5.0);
  max_range_ = 100.0;
  min_range_ = 1.0;
}

void Vlp16::SetRotationRate(float rot_hz){
  rot_hz = math::clamp(rot_hz,5.0f,20.0f);
	rotation_rate_ = rot_hz;
  //float res = 0.2f*rot_hz;
  float res = 0.02f*rot_hz;
  SetScanPattern(-180.0f,180.0f-res,res,-15.0f,15.0f,2.0f);
  SetTimeStep(1.0/rot_hz);
}

} //namespace lidar
} //namespace sensor
} //namespace mavs
