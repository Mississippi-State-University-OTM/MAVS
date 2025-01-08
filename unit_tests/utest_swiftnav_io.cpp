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
#include <iostream>
#include "sensors/ins/swiftnav_msg.h"

int main(int argc, char *argv[]) {

	mavs::swiftnav::sbpMessagesWithTimeStamp msg_out,msg_in;
	msg_out.pos_llh.lat = 100.0f;
	msg_out.pos_llh.lon = 200.0f;
	msg_out.pos_llh.height = -100.0f;
	msg_out.time = 1000;
	std::cout << msg_out << std::endl;
	msg_out.Write("output.sbp");

	msg_in.Read("output.sbp");
	std::cout << msg_in;

	std::cout << sizeof(msg_out) << " "<<sizeof(mavs::swiftnav::dwTime_t)<<" "<< sizeof(mavs::swiftnav::msg_pos_llh_t) << " " << sizeof(mavs::swiftnav::msg_vel_ned_t) << " " << sizeof(mavs::swiftnav::msg_dops_t) << " " << sizeof(mavs::swiftnav::msg_utc_time_t) << " " << sizeof(mavs::swiftnav::msg_orient_euler_t) << " " << sizeof(mavs::swiftnav::msg_orient_quat_t) << " " <<  sizeof(mavs::swiftnav::msg_angular_rate_t) << " " << sizeof(mavs::swiftnav::msg_imu_raw_t) << " " << sizeof(mavs::swiftnav::msg_mag_raw_t) << " " << sizeof(mavs::swiftnav::swiftnavFLAGS) << " " << std::endl;
	
	return 0;
}