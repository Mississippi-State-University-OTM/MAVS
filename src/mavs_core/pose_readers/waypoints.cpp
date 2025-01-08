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
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>

#include <iostream>
#include <limits>
#include <algorithm>
#include <mavs_core/pose_readers/anvel_vprp_reader.h>
#include <mavs_core/math/utils.h>
#include <mavs_core/pose_readers/waypoints.h>
#include <mavs_core/pose_readers/trail.h>
#include <mavs_core/coordinate_systems/coord_conversions.h>
#include <mavs_core/math/segment.h>

namespace mavs {

Waypoints::Waypoints() {
	mapres_ = 2.0f; //1.0f;
	nx_ = 0;
	ny_ = 0;
	llc_.x = std::numeric_limits<float>::max();
	llc_.y = std::numeric_limits<float>::max();
	urc_.x = std::numeric_limits<float>::lowest();
	urc_.y = std::numeric_limits<float>::lowest();
}

Waypoints::~Waypoints() {

}

glm::vec2 Waypoints::GetWaypoint(int wp) {
	if (path_.empty()) {
		glm::vec2 p(0, 0);
		return p;
	}
	if (wp > (int)path_.size()) {
		return path_.back();
	}
	if (wp < 0) {
		return path_[0];
	}
	return path_[wp];
}

void Waypoints::Load(std::string input_file) {
	FILE* fp = fopen(input_file.c_str(), "rb");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	glm::vec2 utm_offset(0.0f, 0.0f);
	if (d.HasMember("UTM Offset")) {
		utm_offset.x = d["UTM Offset"][0].GetFloat();
		utm_offset.y = d["UTM Offset"][1].GetFloat();
	}

	if (d.HasMember("Waypoints")) {
		int num_waypoints = d["Waypoints"].Capacity();
		if (num_waypoints>0) {
			path_.clear();
			for (int i = 0; i<num_waypoints; i++) {
				glm::vec2 p;
				p.x = d["Waypoints"][i][0].GetFloat();
				p.y = d["Waypoints"][i][1].GetFloat();
				path_.push_back(p);
			}
		}
	}

	std::string format = "Local";
	if (d.HasMember("Format")) {
		format = d["Format"].GetString();
	}

	if (path_.size() > 0) {
		if (format == "LongLatAltEasting") {
			mavs::coordinate_system::CoordinateConverter converter;
			mavs::coordinate_system::LLA lla0;
			lla0.latitude = path_[0].y;
			lla0.longitude = -path_[0].x;
			lla0.altitude = 0.0;
			converter.SetLocalOrigin(lla0);
			for (int i = 0; i < path_.size(); i++) {
				mavs::coordinate_system::LLA lla;
				lla.latitude = path_[i].y;
				lla.longitude = -path_[i].x;
				lla.altitude = 0.0;
				mavs::coordinate_system::ENU enu = converter.LLA2ENU(lla);
				path_[i].x = (float)(-enu.y+utm_offset.x);
				path_[i].y = (float)(enu.x+utm_offset.y);
			}
		}
	}
	CreateSegmentMap();
}

void Waypoints::CreateFromPathMsg(mavs::Path path) {
	for (int i = 0; i < (int)path.poses.size(); i++) {
		glm::vec2 p(path.poses[i].pose.position.x, 
			path.poses[i].pose.position.y);
		path_.push_back(p);
	}
}

void Waypoints::Print() {
	for (int i = 0; i < (int)path_.size(); i++) {
		std::cout <<"("<< path_[i].x << ", " << path_[i].y<<")" << std::endl;
	}
}

mavs::Path Waypoints::GetPathMsg() {
	mavs::Path mp;
	for (int i = 0; i < (int)path_.size(); i++) {
		mavs::PoseStamped ps;
		ps.pose.position.x = path_[i].x;
		ps.pose.position.y = path_[i].y;
		ps.pose.position.z = 0.0;
		mp.poses.push_back(ps);
	}
	return mp;
}

float Waypoints::PointSegmentDistance(glm::vec2 s1, glm::vec2 s2, glm::vec2 p) {
	glm::vec2 v = s2 - s1;
	glm::vec2 u = p - s1;
	float t = glm::dot(u, v) / glm::dot(v, v);
	if (t < 0.0f) t = 0.0f;
	if (t > 1.0f) t = 1.0f;
	glm::vec2 pt = s1 + t * v;
	float dist = glm::length(pt - p);
	return dist;
}

float Waypoints::DistanceToPath(glm::vec2 point) {
	float closest = std::numeric_limits<float>::max();
	/*for (int k = 0; k < (int)(path_.size()-1); k++) {
		float d = PointSegmentDistance(path_[k], path_[k + 1], point);
		if (d<closest)closest = d; 
	}
	*/
	glm::vec2 v = point - llc_;
	int i = (int)floor(v.x/mapres_);
	int j = (int)floor(v.y/mapres_);
	if (i >= 0 && i < nx_ && j >= 0 && j < ny_) {
		for (int k = 0; k < segments_in_cell_[i][j].size(); k++) {
			int n = segments_in_cell_[i][j][k];
			float dist = PointSegmentDistance(path_[n], path_[n + 1], point);
			if (dist < closest) {
				closest = dist;
			}
		}
	}
	
	return closest;
}

int ncreated = 0;

void Waypoints::CreateSegmentMap() {
	llc_.x = llc_.x - mapres_;
	llc_.y = llc_.y - mapres_;
	urc_.x = urc_.x + mapres_;
	urc_.y = urc_.y + mapres_;
	glm::vec2 v = urc_ - llc_;
	nx_ = (int)ceil((v.x)/mapres_);
	ny_ = (int)ceil((v.y)/mapres_);
	if (nx_ <= 0 || ny_ <= 0) return;
	std::vector<int> segs;
	if (segments_in_cell_.size() != nx_) {
		segments_in_cell_ = mavs::utils::Allocate3DVector(nx_, ny_, 0, 0);
	}
	for (int i = 0; i < nx_; i++) {
		float xlo = llc_.x + mapres_ * (i - 2);
		float xhi = llc_.x + mapres_ * (i + 2);
		for (int j = 0; j < ny_; j++) {
			float ylo = llc_.y + mapres_ * (j - 2);
			float yhi = llc_.y + mapres_ * (j + 2);
			glm::vec2 ll(xlo, ylo);
			glm::vec2 ur(xhi, yhi);
			for (int k = 0; k < (int)(path_.size()-1); k++) {
				bool inbox = mavs::math::SegAABBIntersect(path_[k], path_[k + 1], ll, ur);
				if (inbox) {
					segments_in_cell_[i][j].push_back(k);
				}
			}
		}
	}
	ncreated++;
}

bool Waypoints::IsPointOnPath(glm::vec2 point, float dist, float &dp) {
	dp = 100.0f;
	/*if (point.x<(llc_.x-dist) || point.y<(llc_.y-dist) || point.x>(urc_.x+dist) || point.y>(urc_.y+dist)) {
		return false;
	}*/
	dp = DistanceToPath(point);
	if (dp < dist) return true;
	return false;
}

void Waypoints::CreateFromAnvelVprp(std::string anvel_file) {
	AnvelVprpReader path_reader;
	std::vector<mavs::Pose> poses = path_reader.Load(anvel_file, 1);
	for (int i = 0; i < (int)poses.size(); i++) {
		glm::vec2 point(poses[i].position.x, poses[i].position.y);
		path_.push_back(point);
	}

	llc_.x = std::numeric_limits<float>::max();
	llc_.y = std::numeric_limits<float>::max();
	urc_.x = std::numeric_limits<float>::lowest();
	urc_.y = std::numeric_limits<float>::lowest();
	for (int i = 0; i < (int)path_.size(); i++) {
		if (path_[i].x < llc_.x)llc_.x = path_[i].x;
		if (path_[i].y < llc_.y)llc_.y = path_[i].y;
		if (path_[i].x > urc_.x)urc_.x = path_[i].x;
		if (path_[i].y > urc_.y)urc_.y = path_[i].y;
	}
	CreateSegmentMap();
}

void Waypoints::AddPoint(glm::vec2 point) {
	path_.push_back(point);
	if (point.x < llc_.x)llc_.x = point.x; 
	if (point.y < llc_.y)llc_.y = point.y; 
	if (point.x > urc_.x)urc_.x = point.x;
	if (point.y > urc_.y)urc_.y = point.y; 
}

void Waypoints::SaveAsVprp(std::string anvel_file) {
	Trail trail;
	trail.SetPath(*this);
	trail.SaveToAnvelVprp(anvel_file);
	//for (int i = 0; i < path_.size(); i++) {
	//	std::cout << path_[i].x << " " << path_[i].y << std::endl;
	//}
}

void Waypoints::SaveAsJson(std::string output_file) {
	rapidjson::StringBuffer s;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(s);
	writer.SetIndent(' ', 2);

	writer.StartObject(); 
	{

		writer.Key("Waypoints");
		writer.StartArray();
		for (int i = 0; i < path_.size(); i++) {
			writer.StartArray();
			for (int j = 0; j < 2; j++) {
				writer.Double(path_[i][j]);
			}
			writer.EndArray();
		}
		writer.EndArray();
	}

	writer.EndObject();
	std::string outstring = s.GetString();
	std::ofstream fout(output_file.c_str());
	fout << outstring;
	fout.close();
} // SaveAsJson

} //namespace mavs