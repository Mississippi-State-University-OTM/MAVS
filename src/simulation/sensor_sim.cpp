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

#include <sensors/mavs_sensors.h>
#include <simulation/sensor_sim.h>

//mavs sensors
#include <sensors/io/json_io.h>

//mavs raytracers
#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif
#include <raytracers/simple_tracer/simple_tracer.h>

//pose readers
#include <mavs_core/pose_readers/anvel_vprp_reader.h>

#include <mavs_core/math/utils.h>

#ifdef USE_OMP
#include <omp.h>
#endif

namespace mavs {

SensorSimulation::SensorSimulation() {
	was_loaded_ = false;
	display_sensors_ = false;
	save_sensors_ = false;
}

SensorSimulation::~SensorSimulation() {
	if (was_loaded_) {
		delete environment_;
		delete scene_;
		for (sensor::Sensor* sensor_ : sensors_) {
			delete sensor_;
		}
	}
}

void SensorSimulation::Run() {
	int myid = 0;
#ifdef USE_MPI
	MPI_Comm_rank(MPI_COMM_WORLD, &myid);
#endif

	int np = (int)poses_.size();
	for (int i = 0; i < (int)poses_.size(); i++) {
#ifdef USE_OMP
		double t0 = omp_get_wtime();
#endif
		float h = environment_->GetGroundHeight((float)poses_[i].position.x, (float)poses_[i].position.y);
		glm::vec3 position = poses_[i].position;
		position.z = 1.5f + h;
		for (sensor::Sensor* sensor_ : sensors_) {
			sensor_->SetPose(position, poses_[i].quaternion);
			sensor_->Update(environment_, 0.0);
		}
		if (myid == 0) {
			for (sensor::Sensor* sensor_ : sensors_) {
				if (display_sensors_)sensor_->Display();
				if (save_sensors_)sensor_->SaveRaw();
				if (sensor_->GetDrawAnnotations()) {
					if (!save_sensors_)sensor_->SaveRaw();
					sensor_->AnnotateFrame(environment_, true);
					sensor_->SaveAnnotation();
			}
		}
#ifdef USE_OMP
			std::cout << "Frame time = " << omp_get_wtime() - t0 <<
				" seconds." << std::endl;
#endif
	}
}
}

void SensorSimulation::LoadTextPoses(std::string pose_file) {
	std::ifstream fin(pose_file.c_str());
	while (!fin.eof()) {
		mavs::Pose p;
		fin >> p.position.x >> p.position.y >> p.position.z >> p.quaternion.w >>
			p.quaternion.x >> p.quaternion.y >> p.quaternion.z;
		poses_.push_back(p);
	}
	poses_.pop_back();
	fin.close();
}

void SensorSimulation::Load(std::string input_file) {
	FILE* fp = fopen(input_file.c_str(), "rb");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	if (d.HasMember("Poses")) {
		std::string pose_file = d["Poses"]["Pose File"].GetString();
		std::string pose_type = d["Poses"]["Pose Type"].GetString();
		int pose_freq = 1;
		if (d["Poses"].HasMember("Pose Frequency"))
			pose_freq = d["Poses"]["Pose Frequency"].GetInt();
		if (pose_type == "anvel") {
			AnvelVprpReader reader;
			poses_ = reader.Load(pose_file, pose_freq);
		}
		else if (pose_type == "text") {
			LoadTextPoses(pose_file);
		}
	}
	else {

	}

	environment_ = new environment::Environment;
	int month, year, date, hour, minute, second, timezone;
	if (d.HasMember("Environment")) {
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
		environment_->SetAlbedoRgb(red_alb, grn_alb, blu_alb);
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

	if (d.HasMember("Scene")) {
		std::string scene_file = d["Scene"]["Input File"].GetString();
#ifdef USE_EMBREE
		scene_ = new raytracer::embree::EmbreeTracer;
#else
		scene_ = new raytracer::SimpleTracer;
#endif
		scene_->Load(scene_file);
		double lat = d["Scene"]["Origin"][0].GetDouble();
		double lon = d["Scene"]["Origin"][1].GetDouble();
		double alt = d["Scene"]["Origin"][2].GetDouble();
		timezone = d["Scene"]["Time Zone"].GetInt();
		environment_->SetLocalOrigin(lat, lon, alt);
	}

	if (d.HasMember("Display Sensors")) {
		display_sensors_ = d["Display Sensors"].GetBool();
	}

	if (d.HasMember("Save Data")) {
		save_sensors_ = d["Save Data"].GetBool();
	}

	io::LoadSensorBlock(d, sensors_);
	if (sensors_.size() <= 0) {
		std::cerr << "ERROR: No sensor block was loaded" << std::endl;
	}
	if (d.HasMember("Output Directory")) {
		std::string out_dir = d["Output Directory"].GetString();
		for (sensor::Sensor* sensor_ : sensors_) {
			sensor_->SetSaveDirectory(out_dir);
		}
	}

	environment_->SetDateTime(year, month, date, hour, minute, second, timezone);
	environment_->SetRaytracer(scene_);
	was_loaded_ = true;
}

bool SensorSimulation::IsValid() {
	bool valid = true;
	if (environment_ == NULL) {
		valid = false;
		std::cerr << "Simulation environment_ pointer not valid." << std::endl;
	}
	if (scene_ == NULL) {
		valid = false;
		std::cerr << "Simulation scene_ pointer not valid." << std::endl;
	}

	if (sensors_.empty() == true) {
		valid = false;
		std::cerr << "Sensor pointer is not valid." << std::endl;
	}

	return valid;
}

} //namespace mavs
