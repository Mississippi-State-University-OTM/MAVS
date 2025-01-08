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
#include <sensors/lidar/planar_lidar.h>

namespace mavs{
namespace sensor{
namespace lidar{

LaserScan PlanarLidar::GetLaserScan(){
  LaserScan scan;
  scan.angle_min = angle_min_;
  scan.angle_max = angle_max_;
  scan.angle_increment = angle_increment_;
  scan.time_increment = 1.0f/rotation_rate_; //time_increment_;
  scan.scan_time = 1.0f/repitition_rate_; //scan_time_;
  scan.range_min = min_range_;
  scan.range_max = max_range_;
  scan.ranges = distances_;
  scan.intensities = intensities_;
  return scan;
}

void PlanarLidar::SetScanProperties(float ang_min, float ang_max, float ang_res) {
	angle_min_ = (float)(mavs::kDegToRad*ang_min);
	angle_max_ = (float)(mavs::kDegToRad*ang_max);
	angle_increment_ = (float)(mavs::kDegToRad*ang_res);
	SetScanPattern(ang_min, ang_max, ang_res);
}

} //namespace lidar
} //namespace sensor
} //namespace mavs
