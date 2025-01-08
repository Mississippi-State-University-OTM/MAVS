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
// project includes
#include "vehicles/uav/uav_controller.h"
// c++ includes
#include <iostream>
#include <algorithm>

namespace mavs {
namespace vehicle {
UavController::UavController() {
	speed_pid_.SetKp(2.0);
	speed_pid_.SetKi(0.1);
	speed_pid_.SetKd(0.1);
	speed_pid_.SetSetpoint(20.0f);
	alt_pid_.SetKp(0.01);
	alt_pid_.SetKi(0.0);
	alt_pid_.SetKd(0.0);
	alt_pid_.SetSetpoint(250.0f);
	roll_control_.TurnOnLooping();
	roll_control_.SetMaxLookAhead(80.0f);
	roll_control_.SetMinLookAhead(10.0f);
	roll_control_.SetSteeringParam(10.0f);
	roll_control_.SetWheelbase(25.0f);
	roll_control_.SetGoalThreshhold(50.0f);
}

FlightControl UavController::UpdateControl(Uav* uav, float dt) {
	FlightControl control;
	control.throttle = (float)speed_pid_.GetControlVariable(uav->GetAirspeed(), dt);
	control.pitch = -(float)alt_pid_.GetControlVariable(uav->GetAltitude(), dt);

	roll_control_.SetVehiclePosition(uav->GetPosition().x, uav->GetPosition().y);
	roll_control_.SetVehicleOrientation(uav->GetHeadingRadians());
	roll_control_.SetVehicleSpeed(0.0f);
	float thrott, steer, brake;
	roll_control_.GetDrivingCommand(thrott, steer, brake, dt);
	control.roll = std::max(-max_roll_, std::min(max_roll_, -10.0f * steer));
	return control;
}

void UavController::SetSpeedControllerParams(float p, float i, float d) {
	speed_pid_.SetKp(p);
	speed_pid_.SetKi(i);
	speed_pid_.SetKd(d);
}

void UavController::SetAltitudeControllerParams(float p, float i, float d) {
	alt_pid_.SetKp(p);
	alt_pid_.SetKi(i);
	alt_pid_.SetKd(d);
}
} // namespace vehicle
} //namespace mavs
