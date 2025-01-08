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
#include <sensors/lidar/hdl64e_simple.h>

#include <mavs_core/math/utils.h>

namespace mavs{
namespace sensor{
namespace lidar{

Hdl64ESimple::Hdl64ESimple() {
	Hdl64ESimple(10.0);
}

Hdl64ESimple::Hdl64ESimple(float rep_rate){
	float delta = 0.0187f*rep_rate - 0.015f;
  SetBeamSpotRectangular(0.0033f, 0.0007f);
  SetScanPattern(-180.0f,180.0f-delta,delta,-24.8f,2.0f,0.425396825f);
  SetTimeStep(0.1f);
  rotation_rate_ = 10.0f;
  max_range_ = 120.0f;
  min_range_ = 1.0f;
}

} //namespace lidar
} //namespace sensor
} //namespace mavs
