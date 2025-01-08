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
/** \file utest_mavs_imu.cpp
* Unit test to evaluate the mavs imu sensor
*
* Usage: >./utest_mavs_imu
*
* Saves a file, imu_log.txt, that has seven space delimited
* columns for 
* time, measured_acceleration(x,y,z) and measured angular velocity (x,y,z)
*
* The true acceleration is (1,0,9.806) and the true angular
* velocity is (0,0,0.5)
* 
* \author Chris Goodin
*
* \date 10/11/2018
*/
// c++ includes
#include <fstream>
// mavs includes
#include <raytracers/simple_tracer/simple_tracer.h>
#include <sensors/imu/imu.h>
#include <sensors/imu/imu_simple.h>
#include <mavs_core/data_path.h>

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

	//mavs::sensor::imu::Imu imu;
	mavs::sensor::imu::ImuSimple imu;
	mavs::MavsDataPath mdp;
	imu.Load(mdp.GetPath() + "/sensors/imu/sample_imu_simple.json");
	//imu.SetNoiseModel("acc", "x", -0.1f, 0.1f);

  veh_state.pose.position.x = 0.0;
  veh_state.pose.position.y = 0.0;
  veh_state.pose.position.z = 2.0;
	veh_state.accel.linear.x = 1.0;
	veh_state.accel.linear.y = 0.0;
	veh_state.accel.linear.z = 9.806;
	veh_state.twist.angular.x = 0.0;
	veh_state.twist.angular.y = 0.0;
	veh_state.twist.angular.z = 0.5;

	std::ofstream fout;
	fout.open("imu_log.txt");
	double dt = 0.01;
	double time = 0.0;
	while (time<1000.0){
		imu.SetPose(veh_state);
		imu.Update(&env, dt);
		glm::vec3 accel_out = imu.GetAcceleration();
		glm::vec3 rot_out = imu.GetAngularVelocity();
		fout << time<<" "<<accel_out.x << " "<<accel_out.y<<" "<<accel_out.z<<" "<<rot_out.x<<" "<<rot_out.y<<" "<<rot_out.z<<std::endl;
		time += dt;
	}
	fout.close();
	
#ifdef USE_MPI  
  MPI_Finalize();
#endif
  return 0;
}
