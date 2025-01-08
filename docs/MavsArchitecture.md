# MAVS Architecture
A MAVS simulation consists of a vehicle, a driver, and any number of sensors. Each simulation component keeps track of its own simulation time and is updated when its local time lags the global simulation time. An example update loop for the serial / shared memory version of the code is given below.

``` c++
void Simulation::UpdateSerial(double dt) {
  // Update the vehicle model
  if (sim_time_ >= vehicle_->GetLocalSimTime()) {
    vehicle_->Update(environment_, driver_->GetThrottle(), driver_->GetSteering(), dt);
  }
  // Get the updated vehicle state
  VehicleState veh_state = vehicle_->GetState();
  // Loop through the sensors
  for (int i = 0; i<sensors_.size(); i++) {
    // Update the sensor if its local time is lagging
    if (sim_time_ >= sensors_[i]->GetLocalSimTime()) {
      sensors_[i]->SetPose(veh_state);
      sensors_[i]->Update(environment_, dt);
	}
  }
  // Update the driver
  if (sim_time_ >= driver_->GetLocalSimTime()) {
    driver_->Update(sensors_, dt);
  }
  // Update the global simulation time
  sim_time_ += dt;
}
```

For simulations using MPI, the update step is more complex. Each component is assigned an MPI subgroup which itself consists of some number of parallel processes. The subgroup is identified by the *group_number_* variable in the simulation. The update step for a distributed simulation is shown below.
``` c++
void Simulation::Update(double dt) {
  // Loop over all the MPI subgroups
  // Each process runs this loop independently
  for (int i = 0; i<num_processes_; i++) {
    // Update only components that share my group number
    if (i == group_number_) {
      if (processes_[i].type == "vehicle") {
        if (sim_time_ >= vehicle_->GetLocalSimTime()) {
          vehicle_->Update(environment_, driver_->GetThrottle(), driver_->GetSteering(), dt);
        }
      }
      else if (processes_[i].type == "sensor") {
        if (sim_time_ >= sensors_[processes_[i].id_num]->GetLocalSimTime()) {
          VehicleState veh_state = vehicle_->GetState();
          sensors_[processes_[i].id_num]->SetPose(veh_state);
          sensors_[processes_[i].id_num]->Update(environment_, dt);
        }
      }
      else if (processes_[i].type == "driver") {
        if (sim_time_ >= driver_->GetLocalSimTime()) {
          VehicleState veh_state = vehicle_->GetState();
          driver_->Update(sensors_, dt);
        }
      }
    }
  }
  // The publish data method shares the interface data across subgroups
  vehicle_->PublishData(process_root_[vehicle_id_num_], broadcast_to_);
  driver_->PublishData(process_root_[driver_id_num_], broadcast_to_);
  for (int i = 0; i<sensors_.size(); i++) {
    int id = sensor_id_num_[i];
    sensors_[i]->PublishData(process_root_[id], broadcast_to_);
  }
  sim_time_ += dt;
}
```

