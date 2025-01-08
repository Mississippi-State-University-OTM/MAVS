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
* \file utest_mavs_gps.cpp
*
* Unit test to evaluate the mavs gps sensor
*
* Usage: >./utest_mavs_gps
*
* Creates a differential and dual-band GPS and runs
* two iterations of each sensor. In the first simulation
* the GPS sensors are run in completely "open-field" 
* conditions with no satellite occlusion or multipath
* errors. In the second, a simple scene with buildings
* surrounding the GPS units is created, causing
* multipath and dilution of precision errors.
* Correct errors should be on the order of 10s of centimeters
* for the dual band GPS and a few centimeters for the differential gps.
*
* \author Chris Goodin
*
* \date 10/4/2018
*/

#include <raytracers/simple_tracer/simple_tracer.h>
#include <sensors/gps/gps.h>

int main (int argc, char *argv[]){
  
  int myid = 0;
  int numprocs = 1;
#ifdef USE_MPI
  int ierr = MPI_Init(&argc, &argv);
  MPI_Comm_size(MPI_COMM_WORLD,&numprocs);
  MPI_Comm_rank(MPI_COMM_WORLD, &myid);
#endif  
  mavs::environment::Environment env;
  env.SetLocalOrigin(32.6526,-90.8779,73.152); //Vicksburg, MS
  
  mavs::raytracer::SimpleTracer scene;
  mavs::raytracer::Aabb ground(0.0f, 0.0f, 0.0f, 1.0E6f, 1.0E6f, 0.01f);
  ground.SetColor(0.24f, 0.48f, 0.2f);
  scene.AddPrimitive(ground);

  env.SetRaytracer(&scene);
  
  mavs::VehicleState veh_state;

	mavs::sensor::gps::Gps dual_band_gps, differential_gps;
	differential_gps.SetType("differential");
	dual_band_gps.SetType("dual band");

  veh_state.pose.position.x = 0.0;
  veh_state.pose.position.y = 0.0;
  veh_state.pose.position.z = 2.0;
  dual_band_gps.SetPose(veh_state);
  dual_band_gps.Update(&env,1.0);
	differential_gps.SetPose(veh_state);
	differential_gps.Update(&env, 1.0);

  glm::dvec3 p_diff = differential_gps.GetRecieverPositionENU();
	glm::dvec3 p_dual = dual_band_gps.GetRecieverPositionENU();
  std::cout<<"No buildings: "<<std::endl;
  std::cout<<"Dual band measured position: "<<p_dual.x<<" "<<p_dual.y<<" "<<p_dual.z<<std::endl;
  std::cout<<"Dual band lateral error: "<<sqrt(p_dual.x*p_dual.x+p_dual.y*p_dual.y)<<" meters."<<std::endl;
	std::cout << "Differential measured position: " << p_diff.x << " " << p_diff.y << " " << p_diff.z << std::endl;
	std::cout << "Differential lateral error: " << sqrt(p_diff.x*p_diff.x + p_diff.y*p_diff.y) << " meters." << std::endl;
  std::cout<<"Num satellite signals: "<<dual_band_gps.GetNumSignals()<<std::endl;

  //add some "buildings" around to block signals
  float bsize = 40.0f;
  float bheight = 10.0f;
  float bloc = 0.5f*bsize;
  float bh2 = 0.5f*bheight;
  mavs::raytracer::Aabb b1(bloc, 0.0f, bh2, 0.1f, bsize, bheight);
  b1.SetColor(0.5f, 0.15f, 0.15f);
  mavs::raytracer::Aabb b2(0.0f, bloc, bh2, bsize, 0.1f, bheight);
  b2.SetColor(0.5f, 0.15f, 0.15f);
  mavs::raytracer::Aabb b3(-bloc, 0.0f, bh2, 0.1f, bsize, bheight);
  b3.SetColor(0.5f, 0.15f, 0.15f);
  mavs::raytracer::Aabb b4(0.0f, -bloc, bh2, bsize, 0.1f, bheight);
  b4.SetColor(0.5f, 0.15f, 0.15f);
  scene.AddPrimitive(b1);
  scene.AddPrimitive(b2);
  scene.AddPrimitive(b3);
  scene.AddPrimitive(b4);
  
	dual_band_gps.Update(&env, 1.0);
	differential_gps.Update(&env, 1.0);
  
	p_diff = differential_gps.GetRecieverPositionENU();
	p_dual = dual_band_gps.GetRecieverPositionENU();
  std::cout<<std::endl<<"With buildings: "<<std::endl;
	std::cout << "Dual band measured position: " << p_dual.x << " " << p_dual.y << " " << p_dual.z << std::endl;
	std::cout << "Dual band lateral error: " << sqrt(p_dual.x*p_dual.x + p_dual.y*p_dual.y) << " meters." << std::endl;
	std::cout << "Differential measured position: " << p_diff.x << " " << p_diff.y << " " << p_diff.z << std::endl;
	std::cout << "Differential lateral error: " << sqrt(p_diff.x*p_diff.x + p_diff.y*p_diff.y) << " meters." << std::endl;
	std::cout << "Num satellite signals: " << dual_band_gps.GetNumSignals() << std::endl;
  
  std::cout<<"GPS Simulation completed"<<std::endl;
#ifdef USE_MPI  
  MPI_Finalize();
#endif
  return 0;
}
