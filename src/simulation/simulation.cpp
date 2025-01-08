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
#include <simulation/simulation.h>

#include <stdlib.h>
#include <iostream>
#include <numeric>
#ifdef USE_OMP
#include <omp.h>
#endif

#include <sensors/camera/rgb_camera.h>

namespace mavs {

Simulation::Simulation() {
	group_number_ = 0;
	sim_time_ = 0.0;
	sim_time_step_ = 0.01;
	max_sim_time_ = 3.0*sim_time_step_;
	display_sensors_ = false;
	log_.open("sim_log.txt");
	total_wall_ = 0.1;
	total_sensor_ = 0.0;
	total_reduce_ = 0.0;
	total_vehicle_ = 0.0;
	total_driver_ = 0.0;
	was_loaded_ = false;
	log_ << "SimTime WallTime Ratio Reduce Sensor Vehicle Driver Check " << std::endl;
}

Simulation::~Simulation() {
	if (was_loaded_) {
		delete environment_;
		delete scene_;
		delete vehicle_;
		delete driver_;
		if (sensors_.size() > 0) {
			for (int i = 0; i < (int)sensors_.size(); i++) {
				delete sensors_[i];
			}
			sensors_.clear();
		}
	}
	log_.close();
}

void Simulation::AddVehicle(vehicle::Vehicle &vehicle, const int nprocs) {
	vehicle_ = &vehicle;
	SetVehicle(nprocs);
}

void Simulation::SetVehicle(int nprocs) {
	CommDef def;
	def.num_procs = nprocs;
	def.type = "vehicle";
	def.id_num = -1;
	processes_.push_back(def);
}

void Simulation::AddSensor(sensor::Sensor &sensor, const int nprocs) {
	sensor::Sensor *sensor_g = &sensor;
	sensors_.push_back(sensor_g);
	SetSensor(nprocs);
}

void Simulation::SetSensor(int nprocs) {
	CommDef def;
	def.num_procs = nprocs;
	def.type = "sensor";
	def.id_num = (int)sensors_.size() - 1;
	processes_.push_back(def);
}

void Simulation::AddDriver(driver::Driver &driver, const int nprocs) {
	driver_ = &driver;
	SetDriver(nprocs);
}

void Simulation::SetDriver(int nprocs) {
	CommDef def;
	def.num_procs = nprocs;
	def.type = "driver";
	def.id_num = -2;
	processes_.push_back(def);
}

void Simulation::SetEnvironment(environment::Environment &env) {
	environment_ = &env;
}

#ifdef USE_MPI
void Simulation::CreateGroups(MPI_Comm parent_comm) {
	int total_procs;
	MPI_Comm_dup(parent_comm, &broadcast_to_);
	MPI_Comm_size(broadcast_to_, &total_procs);
	MPI_Comm_rank(broadcast_to_, &proc_id_);

	num_processes_ = (int)processes_.size();
	process_root_.resize(num_processes_);
	int total_requested = 0;
	for (int i = 0; i < num_processes_; i++) {
		total_requested = total_requested + processes_[i].num_procs;
	}

	if (total_requested > total_procs) {
		if (proc_id_ == 0)
			std::cerr << "Error: Requested more processors than the job has available."
			<< std::endl;
		exit(EXIT_FAILURE);
	}

	total_requested = 0;
	for (int i = 0; i < num_processes_; i++) {
		if (processes_[i].num_procs <= 0) {
			if (proc_id_ == 0)
				std::cerr << "Error: Must request at least one processor per group." <<
				std::endl;
			exit(EXIT_FAILURE);
		}
		int old_total = total_requested;
		process_root_[i] = old_total;
		total_requested += processes_[i].num_procs;
		if (proc_id_ >= old_total && proc_id_ < total_requested) {
			group_number_ = i;
		}
	}

	MPI_Comm_split(broadcast_to_, group_number_, proc_id_, &group_comm_);
	SetCommunicators();
}
#endif

#ifdef USE_MPI
void Simulation::SetCommunicators() {
	driver_id_num_ = 0;
	vehicle_id_num_ = 0;
	sensor_id_num_.resize(sensors_.size(), 0);
	for (int i = 0; i < num_processes_; i++) {
		if (i == group_number_) {
			if (processes_[i].type == "vehicle") {
				vehicle_->SetComm(group_comm_);
				vehicle_id_num_ = i;
			}
			else if (processes_[i].type == "sensor") {
				sensors_[processes_[i].id_num]->SetComm(group_comm_);
				sensor_id_num_[processes_[i].id_num] = i;
			}
			else if (processes_[i].type == "driver") {
				driver_->SetComm(group_comm_);
				driver_id_num_ = i;
			}
			else {
				std::cerr << "ERROR, process type " << processes_[i].type <<
					" not recognized" << std::endl;
			}
		}
	}
	MPI_Allreduce(MPI_IN_PLACE, &driver_id_num_, 1, MPI_INT, MPI_MAX, broadcast_to_);
	MPI_Allreduce(MPI_IN_PLACE, &vehicle_id_num_, 1, MPI_INT, MPI_MAX, broadcast_to_);
	if (sensor_id_num_.size() > 0) {
		int sid_size = (int)sensor_id_num_.size();
		MPI_Allreduce(MPI_IN_PLACE, &sensor_id_num_[0], sid_size,
			MPI_INT, MPI_MAX, broadcast_to_);
	}
	MPI_Barrier(broadcast_to_);
}
#endif

bool Simulation::IsValid() {
	bool valid = true;
	if (vehicle_ == NULL) {
		valid = false;
		std::cerr << "Simulation vehicle_ pointer not valid." << std::endl;
	}
	if (driver_ == NULL) {
		valid = false;
		std::cerr << "Simulation driver_ pointer not valid." << std::endl;
	}
	if (sensors_.size() <= 0) {
		valid = false;
		std::cerr << "Simulation not valid - no sensors added." << std::endl;
	}
	else {
		for (int i = 0; i < (int)sensors_.size(); i++) {
			if (sensors_[i] == NULL) {
				valid = false;
				std::cerr << "Sensor " << i << " pointer is not valid." << std::endl;
			}
		}
	}
	return valid;
}

VehicleState Simulation::GetVehicleState() {
	VehicleState veh_state = vehicle_->GetState();
	return veh_state;
}

bool Simulation::Complete() {
	bool complete = false;
	if (sim_time_ > max_sim_time_ || driver_->Complete()) complete = true;
	return complete;
}

void Simulation::UpdateSerial(double dt) {
#ifdef USE_OMP  
	double t0 = omp_get_wtime();
#endif  
	if (sim_time_ >= vehicle_->GetLocalSimTime()) {
		vehicle_->Update(environment_, (float)driver_->GetThrottle(), (float)driver_->GetSteering(), 0.0f, (float)dt);
	}
#ifdef USE_OMP  
	double t1 = omp_get_wtime();
#endif
	VehicleState veh_state = vehicle_->GetState();
	for (int i = 0; i < (int)sensors_.size(); i++) {
		if (sim_time_ >= sensors_[i]->GetLocalSimTime()) {
			sensors_[i]->SetPose(veh_state);
			sensors_[i]->Update(environment_, dt);
			if (display_sensors_)sensors_[i]->Display();
			if (save_data_)sensors_[i]->SaveRaw();
		}
	}
#ifdef USE_OMP  
	double t2 = omp_get_wtime();
#endif
	if (sim_time_ >= driver_->GetLocalSimTime()) {
		//VehicleState veh_state = vehicle_->GetState();
		driver_->Update(sensors_, dt);
		if (display_sensors_)driver_->Display();
	}
#ifdef USE_OMP  
	double t3 = omp_get_wtime();
#endif
	sim_time_ += dt;
#ifdef USE_OMP  
	double wall = t3 - t0;
	total_wall_ += wall;
	total_sensor_ += t2 - t1;
	total_vehicle_ += t1 - t0;
	total_driver_ += t3 - t2;
	log_ << sim_time_ << " " << total_wall_ << " " << total_wall_ / sim_time_ << " 0 " <<
		(int)(100 * (total_sensor_) / total_wall_) << " " <<
		(int)(100 * (total_vehicle_) / total_wall_) << " " <<
		(int)(100 * (total_driver_) / total_wall_) << " 100" << std::endl;
#endif
}

void Simulation::Update(double dt) {
#ifdef USE_MPI  
	std::vector<double> timing;
	timing.resize(9, 0.0);
	timing[1] = MPI_Wtime();
#endif
	for (int i = 0; i < num_processes_; i++) {
		if (i == group_number_) {
			if (processes_[i].type == "vehicle") {
#ifdef USE_MPI
				timing[2] = MPI_Wtime();
#endif
				if (sim_time_ >= vehicle_->GetLocalSimTime()) {
					vehicle_->Update(environment_, (float)driver_->GetThrottle(), (float)driver_->GetSteering(), 0.0f, (float)dt);
				}
#ifdef USE_MPI
				timing[3] = MPI_Wtime();
#endif
			}
			else if (processes_[i].type == "sensor") {
#ifdef USE_MPI
				timing[4] = MPI_Wtime();
#endif
				if (sim_time_ >= sensors_[processes_[i].id_num]->GetLocalSimTime()) {
					VehicleState veh_state = vehicle_->GetState();
					sensors_[processes_[i].id_num]->SetPose(veh_state);
					sensors_[processes_[i].id_num]->Update(environment_, dt);
					if (display_sensors_)sensors_[processes_[i].id_num]->Display();
					if (save_data_)sensors_[processes_[i].id_num]->SaveRaw();
				}
#ifdef USE_MPI
				timing[5] = MPI_Wtime();
#endif
			}
			else if (processes_[i].type == "driver") {
#ifdef USE_MPI
				timing[6] = MPI_Wtime();
#endif
				if (sim_time_ >= driver_->GetLocalSimTime()) {
					VehicleState veh_state = vehicle_->GetState();
					driver_->Update(sensors_, dt);
					if (display_sensors_)driver_->Display();
				}
#ifdef USE_MPI
				timing[7] = MPI_Wtime();
#endif
			}
		}
	}

#ifdef USE_MPI
	timing[0] = MPI_Wtime();
	//publish data
	vehicle_->PublishData(process_root_[vehicle_id_num_], broadcast_to_);
	driver_->PublishData(process_root_[driver_id_num_], broadcast_to_);
	for (int i = 0; i < sensors_.size(); i++) {
		int id = sensor_id_num_[i];
		sensors_[i]->PublishData(process_root_[id], broadcast_to_);
	}
	timing[8] = MPI_Wtime();
	MPI_Allreduce(MPI_IN_PLACE, &timing[0], 9, MPI_DOUBLE, MPI_MAX, broadcast_to_);
	if (proc_id_ == 0) {
		double wall = timing[8] - timing[1];
		double sensor = timing[5] - timing[4];
		double vehicle = timing[3] - timing[2];
		double driver = timing[7] - timing[6];
		double reduce = timing[8] - timing[1];
		total_wall_ += wall;
		total_sensor_ += sensor;
		total_reduce_ += reduce;
		total_vehicle_ += vehicle;
		total_driver_ += driver;
		int reduce_frac = (int)(100 * total_reduce_ / total_wall_);
		int sensor_frac = (int)(100 * total_sensor_ / total_wall_);
		int vehicle_frac = (int)(100 * total_vehicle_ / total_wall_);
		int driver_frac = (int)(100 * total_driver_ / total_wall_);
		int checksum = reduce_frac + sensor_frac + vehicle_frac + driver_frac;
		log_ << sim_time_ << " " << total_wall_ << " " << total_wall_ / sim_time_ << " "
			<< reduce_frac << " " << sensor_frac << " " << vehicle_frac << " "
			<< driver_frac << " " << checksum << std::endl;
	}
#endif  
	sim_time_ += dt;
}

void Simulation::Run() {
	while (!Complete()) {
#ifdef USE_MPI    
		Update(sim_time_step_);
#else
		UpdateSerial(sim_time_step_);
#endif
	}
	return;
} //void Run



void Simulation::RunInteractive() {
	sensor::camera::RgbCamera driver_camera;
	cimg_library::CImgDisplay display;
	float throttle = 0.0f;
	float steering = 0.0f;
	float dt = 0.01f;
	int ns = 0;
	while (!Complete()) {
		if (display.is_keyW()) {
			throttle += 0.05f;
		}
		if (display.is_keyS()) {
			throttle -= 0.05f;
		}
		if (display.is_keyA()) {
			steering += 0.5f;
		}
		else if (display.is_keyD()) {
			steering -= 0.05f;
		}
		else {
			if (steering > 0.0f) {
				steering -= 0.05f;
			}
			else if (steering < 0.05f) {
				steering += 0.05f;
			}
		}
		if (sim_time_ >= vehicle_->GetLocalSimTime()) {
			vehicle_->Update(environment_, throttle, steering, 0.0f, dt);
		}
		VehicleState veh_state = vehicle_->GetState();
		for (int i = 0; i < (int)sensors_.size(); i++) {
			if (sim_time_ >= sensors_[i]->GetLocalSimTime()) {
				sensors_[i]->SetPose(veh_state);
				sensors_[i]->Update(environment_, dt);
				if (display_sensors_)sensors_[i]->Display();
				if (save_data_)sensors_[i]->SaveRaw();
			}
		}

		if (ns % 3 == 0) {
			driver_camera.SetPose(veh_state);
			driver_camera.Update(environment_, dt);
			display.assign(driver_camera.GetCurrentImage());
		}

		ns++;
	} // While !Complete
	return;
} //void Run

} //namespace mavs
