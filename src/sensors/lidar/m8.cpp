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
#include <sensors/lidar/m8.h>

#include <mavs_core/math/utils.h>
#include <mavs_core/math/constants.h>

namespace mavs{
namespace sensor{
namespace lidar{

MEight::MEight(){
  //M8 spins at a constant rate of 10Hz
  // Lasers fire at 53,828 Hz, 5,383 fires/spin
  // 420,000 points per second in single return mode
  // 42,000 points per rotation at 10 Hz
  // 5,250 * 8 = 42,000
  // 360/5250 = 0.06857....

  // no info about the laser divergence in spec sheet
  // set it to the same as velodyne
  SetBeamSpotRectangular(0.0033f, 0.0007f);
  SetRotationRate(10.0f);
  max_range_ = 150.0f;
  min_range_ = 1.0f;
  SetMode(1);
}

void MEight::SetRotationRate(float rot_hz){
  rot_hz = math::clamp(rot_hz,5.0f,20.0f);
  float res = 0.006857f*rot_hz;
  SetScanPattern(-180.0f,180.0f-res,res,-18.22f,3.2f,3.06f);
  SetTimeStep(1.0f/rot_hz);
	rotation_rate_ = rot_hz;
}

} //namespace lidar
} //namespace sensor
} //namespace mavs
