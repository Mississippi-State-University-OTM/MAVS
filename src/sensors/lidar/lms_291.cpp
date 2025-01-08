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
#include <sensors/lidar/lms_291.h>

namespace mavs{
namespace sensor{
namespace lidar{

static const double spin_time = 1.0/75.0;

Lms291_S05::Lms291_S05(){
  SetMode(1);
  SetBeamSpotCircular(0.012913f);
  OneDegreeResolution();
  SetTimeStep(spin_time);
  min_range_ = 1.0;
  max_range_ = 80.0;
  rotation_rate_ = 75.0; //hz
  repitition_rate_ = 75.0;
  is_planar_ = true;
}

void Lms291_S05::OneDegreeResolution(){
  angle_min_ = (float)(-90*kDegToRad);
  angle_max_ = (float)(90*kDegToRad);
  angle_increment_ = (float)(1.0*kDegToRad);
  SetScanPattern(-90,90,1.0);
}

void Lms291_S05::HalfDegreeResolution(){
  angle_min_ = (float)(-90*kDegToRad);
  angle_max_ = (float)(90*kDegToRad);
  angle_increment_ = (float)(0.5*kDegToRad);
  repitition_rate_ = (float)(0.5*repitition_rate_);
  SetScanPattern(-90,90,0.5);
}

void Lms291_S05::QuarterDegreeResolution(){
  angle_min_ = (float)(-50*kDegToRad);
  angle_max_ = (float)(50*kDegToRad);
  angle_increment_ = (float)(0.25*kDegToRad);
  repitition_rate_ = 0.25f*repitition_rate_;
  SetScanPattern(-50,50,0.25);
}

} //namespace lidar
} //namespace sensor
} //namespace mavs
