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
* \file pts_read_write_example.cpp
* 
* Demonstrates how to read and write a binary pts file
* in the NVIDIA format
*
* Usage: >./pts_read_write_example lidar_output.pts
*
* The code will load the binary file, print some info, then write it back out. 
*
* \author Chris Goodin
* 
* \date 11/14/2018
*/
#include <iostream>
#include <fstream>
#include <vector>

#include <interfaces/drive_px2.h>

struct Packet {
	mavs::nvidia::dwLidarDecodedPacket packet_info;
	std::vector<mavs::nvidia::dwLidarPointXYZI> points;
};

struct PointCloud {
	mavs::nvidia::dwLidarProperties header;
	std::vector<Packet> packets;
};

PointCloud ReadPointCloud(std::string pts_file) {
	PointCloud cloud;
	std::ifstream ptsfile;
	ptsfile.open(pts_file.c_str(), std::ios::binary | std::ios::in);
	ptsfile.read((char *)&cloud.header, sizeof(mavs::nvidia::dwLidarProperties));
	cloud.packets.resize(cloud.header.packetsPerSpin);
	for (int i = 0; i < (int)cloud.header.packetsPerSpin; i++) {
		ptsfile.read((char *)&cloud.packets[i].packet_info, sizeof(mavs::nvidia::dwLidarDecodedPacket));
		cloud.packets[i].points.resize(cloud.packets[i].packet_info.nPoints);
		mavs::nvidia::dwLidarPointXYZI *points = new mavs::nvidia::dwLidarPointXYZI[cloud.packets[i].packet_info.nPoints];
		ptsfile.read((char *)points, cloud.packets[i].packet_info.nPoints * sizeof(mavs::nvidia::dwLidarPointXYZI));
		for (int j = 0; j < (int)cloud.packets[i].packet_info.nPoints; j++) {
		}
		delete[] points;
	}
	ptsfile.close();
	return cloud;
}

void WritePointCloud(PointCloud &cloud, std::string outfile) {
	std::ofstream ptsfile;
	ptsfile.open(outfile.c_str(), std::ios::binary | std::ios::out);
	//write header
	ptsfile.write((char *)&cloud.header, sizeof(mavs::nvidia::dwLidarProperties));
	for (int n = 0; n < (int)cloud.header.packetsPerSpin; n++) {
		//write the packet info
		ptsfile.write((char *)&cloud.packets[n].packet_info, sizeof(mavs::nvidia::dwLidarDecodedPacket));
		//write the points in the packet
		ptsfile.write((char *)&cloud.packets[n].points[0], cloud.header.pointsPerPacket * sizeof(mavs::nvidia::dwLidarPointXYZI));
	}
	ptsfile.close();
}

int main(int argc, char *argv[]) {
	if (argc <= 1) {
		std::cerr << "ERROR, must provide .pts file as input " << std::endl;
	}

	//input binary points file name
	std::string pts_file(argv[1]);

	PointCloud cloud = ReadPointCloud(pts_file);

	WritePointCloud(cloud, "output.pts");

	PointCloud incloud = ReadPointCloud("output.pts");

	if (cloud.header.pointsPerSpin == incloud.header.pointsPerSpin) {
		std::cout << "Success!" << cloud.header.pointsPerSpin << "==" << incloud.header.pointsPerSpin << std::endl;
	}
	else {
		std::cout << "Something went wrong: " << cloud.header.pointsPerSpin << "!=" << incloud.header.pointsPerSpin << std::endl;
	}

	return 0;
}