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
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>

#include <simulation/simulation.h>

//mavs vehicles
//#include "vehicles/turtle/turtle.h"
//#include "vehicles/bicycle/bicycle.h"
//#include "vehicles/car/full_car.h"
#include <vehicles/rp3d_veh/mavs_rp3d_veh.h>
#ifdef USE_CHRONO
#include <vehicles/chrono/chrono_wheeled_json.h>
#endif

// mavs driver
#include <drivers/simple_path_planner/simple_path_planner.h>
#include <drivers/simple_waypoint_follower/simple_waypoint_follower.h>

//mavs sensors
//#include <sensors/mavs_sensors.h>
#include <sensors/io/json_io.h>

//mavs raytracers
#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif
#include <raytracers/simple_tracer/simple_tracer.h>

// mavs file_exists utility
#include <mavs_core/math/utils.h>

namespace mavs{

void Simulation::Load(std::string input_file){
  FILE* fp = fopen(input_file.c_str(),"rb");
  if (fp == NULL){
    std::cout<<"There was an error loading the simulation input json file. "<<
      "The simulation will not run properly."<<std::endl;
    return;
  }
  char readBuffer[65536];
  rapidjson::FileReadStream is(fp,readBuffer,sizeof(readBuffer));
  rapidjson::Document d;
  d.ParseStream(is);
  fclose(fp);
  
  int timezone,month,year,date,hour,minute,second;
  environment_ = new environment::Environment;
  if (d.HasMember("Environment")){
    month = d["Environment"]["Month"].GetInt();
    year = d["Environment"]["Year"].GetInt();
    date = d["Environment"]["Day"].GetInt();
    hour = d["Environment"]["Hour"].GetInt();
    minute = d["Environment"]["Minute"].GetInt();
    second = d["Environment"]["Second"].GetInt();
    double turbidity = d["Environment"]["Turbidity"].GetDouble();
    environment_->SetTurbidity(turbidity);
    double red_alb = d["Environment"]["Local Albedo"][0].GetDouble();
    double grn_alb = d["Environment"]["Local Albedo"][1].GetDouble();
    double blu_alb = d["Environment"]["Local Albedo"][2].GetDouble();
    environment_->SetAlbedoRgb(red_alb,grn_alb,blu_alb);
		if (d["Environment"].HasMember("Rain Rate")) {
			float rain_rate = d["Environment"]["Rain Rate"].GetFloat();
			environment_->SetRainRate(rain_rate);
		}
		if (d["Environment"].HasMember("Wind")) {
			float wx = d["Environment"]["Wind"][0].GetFloat();
			float wy = d["Environment"]["Wind"][1].GetFloat();
			environment_->SetWind(wx, wy);
		}
  }
  else{
    std::cerr<<"ERROR: No environment block was listed"<<std::endl;
  }

  if (d.HasMember("Driver")){ 
    std::string driver_type = d["Driver"]["Type"].GetString();
    if (driver_type == "A* Planner"){
      driver_ = new mavs::driver::SimplePathPlanner;
    }
    else if (driver_type == "Waypoint Follower"){
      driver_ = new mavs::driver::SimpleWaypointFollower;
    }
    std::string infile = d["Driver"]["Input File"].GetString();
    if (infile.length()>0){
      if (mavs::utils::file_exists(infile)){
	driver_->Load(infile);
      }
      else{
	std::cerr<<"ERROR: The driver file \""<<infile<<"\" does not exist."
		 <<std::endl;
	return;
      }
    }
    else {
      std::cerr<<"ERROR: No driver input file listed, exiting."<<std::endl;
      exit(1);
    }
    int nprocs = d["Driver"]["Number Processors"].GetInt();
    double rate = d["Driver"]["Update Rate"].GetDouble();
    driver_->SetTimeStep(1.0/rate);
    SetDriver(nprocs);
  }
  else{
    std::cerr<<"ERROR: No Driver block was listed "<<std::endl;
  }

  if (d.HasMember("Vehicle")){
    std::string veh_type = d["Vehicle"]["Type"].GetString();
		if (veh_type == "Rp3d") {
			vehicle_ = new vehicle::Rp3dVehicle;
		}
    /*if (veh_type == "Turtle"){
      vehicle_ = new vehicle::Turtle;
    }
    else if (veh_type == "Bicycle"){
      vehicle_ = new vehicle::Bicycle;
    }
    else if (veh_type == "Car"){
      vehicle_ = new vehicle::FullCar;
    }*/
#ifdef USE_CHRONO    
    else if (veh_type == "Chrono") {
      vehicle_ = new vehicle::ChronoWheeledJson;
    }
#endif    
    else {
      //type not recognized;
    }
    double px = d["Vehicle"]["Initial Position"][0].GetDouble();
    double py = d["Vehicle"]["Initial Position"][1].GetDouble();
    double pz = d["Vehicle"]["Initial Position"][2].GetDouble();
    double qw = d["Vehicle"]["Initial Orientation"][0].GetDouble();
    double qx = d["Vehicle"]["Initial Orientation"][1].GetDouble();
    double qy = d["Vehicle"]["Initial Orientation"][2].GetDouble();
    double qz = d["Vehicle"]["Initial Orientation"][3].GetDouble();
    double cg_height = d["Vehicle"]["CG Height"].GetDouble();
    double rate = d["Vehicle"]["Update Rate"].GetDouble();
    vehicle_->SetPosition(px,py,pz);
    vehicle_->SetOrientation(qw,qx,qy,qz);
    vehicle_->SetTimeStep(1.0/rate);
    vehicle_->SetCgHeight(cg_height);

		std::string veh_file = d["Vehicle"]["Input File"].GetString();
		int nprocs = d["Vehicle"]["Number Processors"].GetInt();
		if (mavs::utils::file_exists(veh_file)) {
			vehicle_->Load(veh_file);
		}
		else {
			std::cerr << "ERROR: the vehicle file \"" << veh_file << "\" does not exist"
				<< std::endl;
			return;
		}

    SetVehicle(nprocs);
  }
  else{
    std::cerr<<"ERROR: No Vehicle info was listed"<<std::endl;
  }

  if (d.HasMember("Time Step")){
    sim_time_step_ = d["Time Step"].GetDouble();
  }

  if (d.HasMember("Scene")){
    scene_file_ = d["Scene"]["Input File"].GetString();
    if (!mavs::utils::file_exists(scene_file_)){
      std::cerr<<"ERROR: the scene file \""<<scene_file_<<"\" does not exist."
	       <<" Simulation will not run properly."<<std::endl;
      return;
    }
#ifdef USE_EMBREE
    scene_ = new raytracer::embree::EmbreeTracer;
#else
	scene_ = new raytracer::SimpleTracer;
#endif
    //let the individual communicators load the scene if they need it
    scene_->Load(scene_file_);
    double lat = d["Scene"]["Origin"][0].GetDouble();
    double lon = d["Scene"]["Origin"][1].GetDouble();
    double alt = d["Scene"]["Origin"][2].GetDouble();
    timezone = d["Scene"]["Time Zone"].GetInt();
    environment_->SetLocalOrigin(lat,lon,alt);
  }
  else{
    std::cerr<<"ERROR: No scene block was listed."<<std::endl;
  }

  if (d.HasMember("Display Sensors")){
    display_sensors_ = d["Display Sensors"].GetBool();
  }

  if (d.HasMember("Save Data")){
    save_data_ = d["Save Data"].GetBool();
  }

	io::LoadSensorBlock(d, sensors_);
	for (int ns = 0; ns < (int)sensors_.size(); ns++) {
		CommDef def;
		def.num_procs = sensors_[ns]->GetNumProcs();
		def.type = "sensor";
		def.id_num = ns; 
		processes_.push_back(def);
	}

  if (d.HasMember("Max Sim Time")){
    max_sim_time_ = d["Max Sim Time"].GetDouble();
  }

	if (d.HasMember("Output Directory")) {
		std::string out_dir = d["Output Directory"].GetString();
		for (int i = 0; i < (int)sensors_.size(); i++) {
			sensors_[i]->SetSaveDirectory(out_dir);
		}
	}

  environment_->SetDateTime(year,month,date,hour,minute,second,timezone);
  environment_->SetRaytracer(scene_);
#ifdef USE_MPI
  CreateGroups(MPI_COMM_WORLD);
#endif
  was_loaded_ = true;
}

} //namespace mavs
