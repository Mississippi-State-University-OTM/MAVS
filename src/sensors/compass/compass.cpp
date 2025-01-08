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
#include <sensors/compass/compass.h>

#include <iostream>
#include <fstream>
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <stdlib.h>

#include <glm/glm.hpp>

namespace mavs{
namespace sensor{
namespace compass{

Compass::Compass(){
  rms_error_degrees_ = 1.0;
  rms_error_radians_ = kDegToRad*rms_error_degrees_;
  type_ = "compass";
}

Compass::~Compass(){
}

void Compass::Update(environment::Environment *env, double dt){
  
  current_heading_ = atan2(look_to_.y, look_to_.x) + 
    rms_error_radians_*((double)rand()/(RAND_MAX));

  local_sim_time_ += local_time_step_;
  updated_ = true;
}

double Compass::GetHeading(){
  return current_heading_;
}
#ifdef USE_MPI
void Compass::PublishData(int root,MPI_Comm broadcast_to){
  MPI_Bcast(&current_heading_,1,MPI_DOUBLE,root,broadcast_to);
}
#endif
} //namespace compass
} //namespace sensor
} //namespace mavs

