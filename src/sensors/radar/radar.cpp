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

#include <sensors/radar/radar.h>

#include <fstream>
#include <algorithm>
#include <iomanip>
#include <sstream>

#include <mavs_core/math/quick_sort.h>

namespace mavs {
namespace sensor {
namespace radar {

Radar::Radar() {
	// delphi radar medium range
	type_ = "radar";
	max_range_ = 60.0f;
	range_disc_ = 1.25f;
	return_thresh_ = (0.9f / (max_range_ * max_range_* max_range_ * max_range_));
	beam_width_ = 1.0f;
	range_rate_pos_ = 25.0f;
	range_rate_neg_ = -100.0f;
	fov_ = 90.0f;
	//Initialize(fov_,1.0f, 0.25f);
	display_width_ = 512;
	display_height_ = 512;
	first_display_ = true;
	image_filled_ = false;
	nsteps_ = 0;
	initialized_ = false;
}

Radar::~Radar() {

}

Radar::Radar(const Radar &radar) {

}

void Radar::Initialize(float hfov_degrees, float vfov_degrees, float angular_resolution_degrees) {
	initialized_ = true;
	beam_spot_points_.clear();
	beam_width_ = vfov_degrees * (float)mavs::kDegToRad;
	int num_vert = (int)(vfov_degrees / angular_resolution_degrees);
	int num_hor = (int)(hfov_degrees / angular_resolution_degrees);

	float ang_step = (float)(kDegToRad * angular_resolution_degrees);
	sample_resolution_ = ang_step;
	float hmin = (float)(-0.5f*hfov_degrees*kDegToRad);
	float hmax = (float)(0.5f*hfov_degrees*kDegToRad);
	float vmin = (float)(-0.5f*vfov_degrees*kDegToRad);
	float vmax = (float)(0.5f*vfov_degrees*kDegToRad);

	float alpha = hmin;
	while (alpha <= hmax) {
		float omega = vmin;
		while (omega <= vmax) {
			glm::vec3 dir(cos(omega)*cos(alpha),cos(omega)*sin(alpha), sin(omega));
			//glm::vec3 dir(cos(alpha), sin(alpha), 0.0f);
			beam_spot_points_.push_back(dir);
			omega += ang_step;
		}
		alpha += ang_step;
	}
	raw_returns_.resize(beam_spot_points_.size());
}

void Radar::Group() {
	float minw = 0.25f*tanf(sample_resolution_ * 0.5f);
	float range = 0.0f; 
	float angle = 0.0f; 
	float position_x = 0.0f; 
	float position_y = 0.0f; 
	float num_returned = 0.0f;
	float vx = 0.0f;
	float vy = 0.0f;
	float y_lo = max_range_;
	float y_hi = -max_range_;
	float last_range = raw_returns_[0].range;

	for (int i = 0; i < (int)raw_returns_.size(); i++) {
		if (raw_returns_[i].returned && fabsf(raw_returns_[i].range-last_range)<range_disc_) {
			range += raw_returns_[i].range;
			angle += raw_returns_[i].theta;
			position_x += raw_returns_[i].x;
			position_y += raw_returns_[i].y;
			vx += raw_returns_[i].vx;
			vy += raw_returns_[i].vy;
			num_returned = num_returned + 1.0f;
			if (raw_returns_[i].y > y_hi)y_hi = raw_returns_[i].y;
			if (raw_returns_[i].y < y_lo)y_lo = raw_returns_[i].y;
		}
		else {
			if (num_returned >= 0.99f) {
				RadarTarget object;
				object.id = raw_returns_[i].id;
				object.angle = angle / num_returned;
				object.range = range / num_returned;
				object.position_x = position_x / num_returned;
				object.position_y = position_y / num_returned;
				object.range_rate = vx / num_returned;
				object.lateral_rate = vy / num_returned;
				object.range_accleration = 0.0f;
				float min_width = object.range * minw;
				object.width = std::max(min_width,y_hi - y_lo);
				object.color = glm::dvec3(1.0, 1.0, 0.0);
				objects_.push_back(object);
			}
			range = 0.0f;
			angle = 0.0f;
			position_x = 0.0f;
			position_y = 0.0f;
			num_returned = 0.0f;
			vx = 0.0f;
			vy = 0.0f;
			y_lo = max_range_;
			y_hi = -max_range_;
		}
		last_range = raw_returns_[i].range;
	}
}

void Radar::GetTargets() {
	int numreturns = (int)raw_returns_.size();
	for (int i = 0; i < numreturns; i++) {
		raw_returns_[i].theta = (float)(fabs(atan2(beam_spot_points_[i].y, beam_spot_points_[i].x) + kPi));
	}

	utils::QuickSort(raw_returns_);

	//remove the pi so the future angle calculations are correct
	for (int i = 0; i < numreturns; i++) {
			raw_returns_[i].theta = (float)(raw_returns_[i].theta - kPi);
	}

	Group();
}

void Radar::ResetBuffers() {
	objects_.clear();
	for (int i = 0; i < (int)raw_returns_.size(); i++) {
		raw_returns_[i].returned = false;
	}
}

void Radar::Update(environment::Environment *env, double dt) {
	if (!initialized_)Initialize(fov_, 1.0f, 0.25f);
	ResetBuffers();
	// do the scan over the discrete directions to get all returns
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < (int)beam_spot_points_.size(); i++) {
		glm::vec3 direction = orientation_ * beam_spot_points_[i];
		direction = glm::normalize(direction);
		raytracer::Intersection inter =
			env->GetClosestIntersection(position_, direction);
		if (inter.dist > 0.0f && inter.dist < max_range_) {
			
			inter.normal = glm::normalize(inter.normal);
			float ref_mag = fabsf(glm::dot(inter.normal,direction))/(inter.dist*inter.dist* inter.dist * inter.dist);
			if (ref_mag >= return_thresh_) {
			//if (ref_mag > 0.25) {
				//RadarCoordinate c;
				// in radar coordinates
				raw_returns_[i].x = beam_spot_points_[i].x*inter.dist; 
				raw_returns_[i].y = beam_spot_points_[i].y*inter.dist;
				glm::vec3 returnvel = inter.velocity*direction;
				raw_returns_[i].vx = returnvel.x;
				raw_returns_[i].vy = returnvel.y;
				raw_returns_[i].range = inter.dist;
				raw_returns_[i].id = inter.object_id;
				raw_returns_[i].returned = true;
			}
		}
		else {
			raw_returns_[i].returned = false;
		}
	}

	//perform clustering algorithm to identify targets
	GetTargets();

	local_sim_time_ += local_time_step_;
	updated_ = true;
	nsteps_++;
}

void Radar::AnnotateFrame(environment::Environment *env, bool semantic) {
	object_label_nums_.resize(objects_.size());
	object_label_colors_.resize(objects_.size());
	for (int i = 0; i < (int)objects_.size(); i++) {
		int obj_num = objects_[i].id;
		std::string mesh_name = env->GetObjectName(obj_num);
		std::string label_name = env->GetLabel(mesh_name);
		int lab_num = env->GetLabelNum(label_name);
		//if (annotations_.count(obj_num) == 0) { // new annotation
		if (annotations_.count(label_name) == 0) { // new annotation
			glm::vec3 pos(objects_[i].position_x, objects_[i].position_y, 0.0f);
			if (semantic) {
				//annotations_[obj_num] = Annotation(label_name, pos, lab_num);
				annotations_[label_name] = Annotation(label_name, pos, lab_num);
				objects_[i].color = env->GetLabelColor(label_name);
			}
			else {
				//annotations_[obj_num] = Annotation(mesh_name, pos, obj_num);
				annotations_[label_name] = Annotation(mesh_name, pos, obj_num);
			}
			glm::quat q_obj = env->GetObjectOrientation(obj_num);
			//annotations_[obj_num].SetOrientation(q_obj);
			annotations_[label_name].SetOrientation(q_obj);
			raytracer::BoundingBox box = env->GetObjectBoundingBox(obj_num);
			//annotations_[obj_num].SetLLCorner(box.GetLowerLeft());
			//annotations_[obj_num].SetURCorner(box.GetUpperRight());
			annotations_[label_name].SetLLCorner(box.GetLowerLeft());
			annotations_[label_name].SetURCorner(box.GetUpperRight());
		}
		else { //existing annotation
			if (semantic) {
				//annotations_[obj_num] = Annotation(label_name, pos, lab_num);
				objects_[i].color = env->GetLabelColor(label_name);
			}
			else {
				//annotations_[obj_num] = Annotation(mesh_name, pos, obj_num);
			}
		}
		object_label_nums_[i] = lab_num;
		if (semantic) {
			object_label_colors_[i] = 255.0f*env->GetLabelColor(label_name);
		}
		else {
			object_label_colors_[i] = anno_colors_.GetColor(obj_num);
		}
	}
}

void Radar::SaveAnnotation() {
	std::ostringstream ss;
	ss << std::setw(5) << std::setfill('0') << nsteps_;
	std::string num_string(ss.str());
	std::string outname = prefix_ + type_ + name_ + num_string + "_annotated";
	std::string imout = outname + ".txt";
	std::string csvout = outname + ".csv";
	WriteLabeledTargetsToText(imout);
	SaveSemanticAnnotationsCsv(csvout);
}

void Radar::SaveSemanticAnnotationsCsv(std::string fname) {
	// Handle frame annotations
	std::ofstream annotationFile;
	annotationFile.open(fname.c_str());
	//annotationFile << "Name, Number, LL.x, LL.y, LL.z, UR.x, UR.y, UR.z" << std::endl;
	annotationFile << "Name, Number, pos.x, pos.y " << std::endl;
	//std::map<int, Annotation>::iterator iter;
	std::map<std::string, Annotation>::iterator iter;
	iter = annotations_.begin();
	int i = 0;
	for (iter = annotations_.begin(); iter != annotations_.end(); ++iter) {
		glm::vec3 pos = iter->second.GetLLCorner();
		//glm::vec3 ur = iter->second.GetURCorner();
		//glm::vec3 dim = ur - ll;
		//glm::vec3 pos = 0.5f*dim + ll;
		//glm::quat q = iter->second.GetOrientation();
		annotationFile << iter->second.GetName() << ", " << iter->second.GetClassNum() << ", " <<
			pos.x << ", " << pos.y << std::endl;
		i++;
	}
	annotationFile.close();
}


void Radar::WriteLabeledTargetsToText(std::string fname) {
	if (object_label_nums_.size() == objects_.size()) {
		std::ofstream fout;
		fout.open(fname.c_str());
		fout << "x y z intensity object" << std::endl;
		for (int i = 0; i < (int)objects_.size(); i++) {
			fout << objects_[i].position_x << " " << objects_[i].position_y << " " <<
				objects_[i].width << " " << object_label_nums_[i] << std::endl;
		}
		fout.close();
	}
	else {
		std::cerr << "ERROR, attempted to save RADAR labels, but frame has not yet been annotated." << std::endl;
	}
}

glm::vec2 Radar::SensorToImageCoords(glm::vec2 sensor_coords) {
	glm::vec2 image_coords;
	float dy2 = 0.5f*display_height_;
	image_coords.x = sensor_coords.x*display_width_ / max_range_;
	image_coords.y = sensor_coords.y*dy2 / max_range_ + dy2;
	return image_coords;
}

void Radar::FillImage() {
	//initialize some values
	const float green[3] = { 0.0f, 255.0f, 0.0f };
	const float blue[3] = { 175.0f, 175.0f, 255.0f };
	//const float yellow[3] = { 255.0f, 255.0f, 0.0f };
	if (first_display_)image_.assign(display_width_, display_height_, 1, 3, 0.0);
	image_ = 0.0f;

	// Plot the background of the radar image
	float y = (float)(max_range_ * tan(0.5f*fov_*kDegToRad));
	glm::vec2 lo(max_range_, -y);
	glm::vec2 hi(max_range_, y);
	glm::vec2 lo_pix = SensorToImageCoords(lo);
	glm::vec2 hi_pix = SensorToImageCoords(hi);
	int im2 = (int)(image_.height() / 2.0);
	image_.draw_line(0, im2, 
		image_.width(), im2, green, 1);
	image_.draw_line(0, im2,
		(int)lo_pix.x, (int)lo_pix.y, green, 1);
	image_.draw_line(0, im2,
		(int)hi_pix.x, (int)hi_pix.y, green, 1);
	image_.draw_circle(0, im2, image_.width(), green, 1.0,1);
	image_.draw_circle(0, im2, (int)(0.75*image_.width()), green, 1.0, 1);
	image_.draw_circle(0, im2, (int)(0.5*image_.width()), green, 1.0, 1);
	image_.draw_circle(0, im2, (int)(0.25*image_.width()), green, 1.0, 1);

	for (int i = 0; i < (int)raw_returns_.size(); i++) {
		if (raw_returns_[i].returned) {
			glm::vec2 pos(raw_returns_[i].x,raw_returns_[i].y);
			glm::vec2 pixpos = SensorToImageCoords(pos);
			image_.draw_point((int)pixpos.x, (int)pixpos.y, (float *)&blue);
		}
	}

	//plot the targets
	//int target_rad = (int)(0.008f*image_.height());
	for (int i =0 ; i < (int)objects_.size(); i++) {
		int target_rad = (int)(objects_[i].width*image_.height() / max_range_);
		target_rad = std::max(2, target_rad);
		glm::vec2 pos(objects_[i].position_x, objects_[i].position_y);
		glm::vec2 pixpos = SensorToImageCoords(pos);
		float scale = 0.5f*(1.0f+std::max(-1.0f, std::min(1.0f, objects_[i].range_rate) / 25.0f));
		//glm::vec3 this_col = 255.0*objects_[i].color;
		const float color[3] = {scale*255.0f, (1.0f-scale)*255.0f, 0.0f};
		image_.draw_circle((int)pixpos.x, (int)pixpos.y, target_rad, color);
	}

	image_filled_ = true;
	image_.mirror("y");
}

void Radar::Display() {
	FillImage();
	if (comm_rank_ == 0) {
		if (first_display_) {
			disp_.assign(display_width_, display_height_, name_.c_str());
			first_display_ = false;
		}
		disp_ = image_;
	}
}

void Radar::SaveImage(std::string fname) {
	if (!image_filled_)FillImage();
	image_.normalize(0, 255);
	image_.save(fname.c_str());
}

void Radar::SaveRaw() {

}

void Radar::Load(std::string input_file) {
	FILE* fp = fopen(input_file.c_str(), "rb");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	if (d.HasMember("Max Range")) {
		max_range_ = d["Max Range"].GetFloat();
	}

	if (d.HasMember("Field of View")) {
		fov_ = d["Field of View"].GetFloat();
	}

	if (d.HasMember("Lobe Size")) {
		beam_width_ = d["Lobe Size"].GetFloat();
	}

	float res = (float)(sample_resolution_ / kDegToRad);
	if (d.HasMember("Lobe Sample Resolution")) {
		res = d["Lobe Sample Resolution"].GetFloat();
	}

	Initialize(fov_, beam_width_, res);
}

void Radar::WriteObjectsToText(std::string outfile) {
	std::ofstream fout(outfile.c_str());
	for (int i = 0; i < (int)objects_.size(); i++) {
		fout << objects_[i].id << " " << objects_[i].position_x << " " << 
			objects_[i].position_y << std::endl;
	}
	fout.close();
}

void Radar::WriteLobeToText(std::string outfile) {
	std::ofstream fout(outfile.c_str());
	for (int i = 0; i < (int)raw_returns_.size(); i++) {
		if (raw_returns_[i].returned) {
			fout << raw_returns_[i].x<< " " << raw_returns_[i].y << std::endl;
		}
	}
	fout.close();
}

#ifdef USE_MPI  
void Radar::PublishData(int root, MPI_Comm broadcast_to) {

}
#endif

} //namespace radar
} //namespace sensor
} //namespace mavs