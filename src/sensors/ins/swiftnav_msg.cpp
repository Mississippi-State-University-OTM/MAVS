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
#include <sensors/ins/swiftnav_msg.h>

#include <iostream>
#include <sstream>
#include <mavs_core/math/utils.h>

namespace mavs {
namespace swiftnav {

std::ostream& operator<< (std::ostream& out, const msg_pos_llh_t& msg) {
	out << "Latitude: " << msg.lat << std::endl;
	out << "Longitude: " << msg.lon << std::endl;
	out << "Height: " << msg.height << std::endl;
	return out;
}

std::ostream& operator<< (std::ostream& out, const msg_vel_ned_t& msg) {
	out << "Velocity: (" << msg.n << ", " << msg.e << ", " << msg.d << ")" << std::endl;
	return out;
}

std::ostream& operator<< (std::ostream& out, const msg_dops_t& msg) {
	out << "DOPS: " << msg.gdop << " " << msg.pdop << " " << msg.tdop << " " << msg.hdop << " " << msg.vdop << std::endl;
	return out;
}

std::ostream& operator<< (std::ostream& out, const msg_gps_time_t& msg) {
	out << "GPS Time: " << msg.wn << " " << msg.tow << " " << msg.ns_residual << std::endl;
	return out;
}


std::ostream& operator<< (std::ostream& out, const msg_utc_time_t& msg) {
	out << "UTC Time: " << (int)msg.month << "/" << (int)msg.day << "/" << msg.year << " " << (int)msg.hours << ":" << (int)msg.minutes << ":" << (int)msg.seconds << "." << msg.ns << std::endl;
	return out;
}

std::ostream& operator<< (std::ostream& out, const msg_baseline_heading_t& msg) {
	out << "Heading: " << msg.heading << " " << msg.n_sats << std::endl;
	return out;
}

std::ostream& operator<< (std::ostream& out, const msg_orient_euler_t& msg) {
	out << "Roll, Pitch, Yaw: (" << msg.roll << ", " << msg.pitch << ", " << msg.yaw << ")" << std::endl;
	return out;
}

std::ostream& operator<< (std::ostream& out, const msg_orient_quat_t& msg) {
	out << "Orientation: (" << msg.w << ", " << msg.x << ", " << msg.y << ", " << msg.z << ")" << std::endl;
	return out;
}

std::ostream& operator<< (std::ostream& out, const msg_angular_rate_t& msg) {
	out << "Angular Rate: (" << msg.x << ", " << msg.y << ", " << msg.z << ")" << std::endl;
	return out;
}

std::ostream& operator<< (std::ostream& out, const msg_imu_raw_t& msg) {
	out << "IMU Acceleration: (" << msg.acc_x << ", " << msg.acc_y << ", " << msg.acc_z << ")" << std::endl;
	out << "IMU Angular Rate: (" << msg.gyr_x << ", " << msg.gyr_y << ", " << msg.gyr_z << ")" << std::endl;
	return out;
}

std::ostream& operator<< (std::ostream& out, const msg_mag_raw_t& msg) {
	out << "Magnetometer: (" << msg.mag_x << ", " << msg.mag_y << ", " << msg.mag_z << ")" << std::endl;
	return out;
}

std::ostream& operator<< (std::ostream& out, const swiftnavFLAGS& msg){
	out << "FLAGS: " << msg.pos_llh << " " << msg.vel_ned << " " << msg.dops << " " << msg.utc_time << " " << msg.orient_euler << " " << msg.orient_quat << " " << msg.angular_rate << " " << msg.imu_raw << " " << msg.mag_raw << std::endl;
	return out;
}

std::ostream& operator<< (std::ostream& out, const sbpMessagesWithTimeStamp& msg) {
	out << msg.pos_llh << msg.vel_ned << msg.dops << msg.utc_time << msg.orient_euler << msg.orient_quat << msg.angular_rate << msg.imu_raw << msg.mag_raw << msg.flags << std::endl;
	return out;
}

void sbpMessagesWithTimeStamp::AppendToFile(std::string ofname) {
	std::ofstream writefile;
	writefile.open(ofname.c_str(), std::ios::binary | std::ios::out | std::ios::app);
	writefile.write((char *)this, sizeof(sbpMessagesWithTimeStamp));
	writefile.close();
}

void sbpMessagesWithTimeStamp::Write(std::string ofname) {
	std::ofstream writefile;
	writefile.open(ofname.c_str(), std::ios::binary | std::ios::out);
	writefile.write((char *)this, sizeof(sbpMessagesWithTimeStamp));
	writefile.close();
}

void sbpMessagesWithTimeStamp::Read(std::string ifname) {
	std::ifstream infile;
	infile.open(ifname.c_str(), std::ios::binary | std::ios::in);
	infile.read((char *)this, sizeof(sbpMessagesWithTimeStamp));
	infile.close();
}

} //namespace swiftnav
} //namespace mavs