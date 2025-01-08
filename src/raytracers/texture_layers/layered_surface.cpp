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

#include <iostream>
#include <limits>
#include <mavs_core/data_path.h>

#include <raytracers/texture_layers/layered_surface.h>
#include <mavs_core/math/utils.h>

namespace mavs {
namespace raytracer {

LayeredSurface::LayeredSurface() {
	trail_width_ = 1.0f;
	/*mapres_ = 2.0f;
	nx_ = 0;
	ny_ = 0;
	trail_ll_ = glm::vec2(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	trail_ur_ = glm::vec2(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());*/
}

void LayeredSurface::SetTrail(Trail &trail) {
	trails_.resize(1);
	trails_[0] = trail;
	//InitTrail();
}

void LayeredSurface::AddTrail(Trail &trail) {
	trails_.push_back(trail);
	//InitTrail();
}

void LayeredSurface::LoadSurfaceTextures(std::string path_to_meshes, std::string texture_input_file) {
	if (!mavs::utils::file_exists(texture_input_file)) {
		std::cerr << "ERROR: File " << texture_input_file << " does not exist" << std::endl;
		exit(2);
	}
	FILE* fp = fopen(texture_input_file.c_str(), "rb");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);
	//MavsDataPath data_path;
	std::string path_to_layers = path_to_meshes; // data_path.GetPath();
	path_to_layers.append("surface_textures/");   // "/environments/ecosystem_inputs/surface_textures/");
	if (d.HasMember("Soil Layer")) {
		if (d["Soil Layer"].HasMember("Texture File") && d["Soil Layer"].HasMember("Texture Size")) {
			TextureLayer layer;
			std::string tname = path_to_layers;
			tname.append(d["Soil Layer"]["Texture File"].GetString());
			layer.LoadImage(tname);
			layer.SetSize(d["Soil Layer"]["Texture Size"].GetFloat());
			layer.SetPixdim(layer.GetSize() / (1.0f*layer.GetWidth()));
			layers_.push_back(layer);

			if (d["Soil Layer"].HasMember("Material")) {
				Material material;
				if (d["Soil Layer"]["Material"].HasMember("Spectrum File")) {
					material.refl = d["Soil Layer"]["Material"]["Spectrum File"].GetString();
				}
				if (d["Soil Layer"]["Material"].HasMember("Kd")) {
					material.kd.r = d["Soil Layer"]["Material"]["Kd"][0].GetFloat();
					material.kd.g = d["Soil Layer"]["Material"]["Kd"][1].GetFloat();
					material.kd.b = d["Soil Layer"]["Material"]["Kd"][2].GetFloat();
				}
				layers_[layers_.size() - 1].SetMaterial(material);
			}
		}
	}

	if (d.HasMember("Grass Layer")) {
		if (d["Grass Layer"].HasMember("Texture File") && d["Grass Layer"].HasMember("Texture Size")) {
			TextureLayer layer;
			std::string tname = path_to_layers;
			tname.append(d["Grass Layer"]["Texture File"].GetString());
			layer.LoadImage(tname);
			layer.SetSize(d["Grass Layer"]["Texture Size"].GetFloat());
			layer.SetPixdim(layer.GetSize() / (1.0f*layer.GetWidth()));
			layers_.push_back(layer);

			if (d["Grass Layer"].HasMember("Material")) {
				Material material;
				if (d["Grass Layer"]["Material"].HasMember("Spectrum File")) {
					material.refl = d["Grass Layer"]["Material"]["Spectrum File"].GetString();
				}
				if (d["Grass Layer"]["Material"].HasMember("Kd")) {
					material.kd.r = d["Grass Layer"]["Material"]["Kd"][0].GetFloat();
					material.kd.g = d["Grass Layer"]["Material"]["Kd"][1].GetFloat();
					material.kd.b = d["Grass Layer"]["Material"]["Kd"][2].GetFloat();
				}
				layers_[layers_.size() - 1].SetMaterial(material);
			}

		}
	}

	if (d.HasMember("Undergrowth Layer")) {
		if (d["Undergrowth Layer"].HasMember("Texture File") && d["Undergrowth Layer"].HasMember("Texture Size")) {
			TextureLayer layer;
			std::string tname = path_to_layers;
			tname.append(d["Undergrowth Layer"]["Texture File"].GetString());
			layer.LoadImage(tname);
			layer.SetSize(d["Undergrowth Layer"]["Texture Size"].GetFloat());
			layer.SetPixdim(layer.GetSize() / (1.0f*layer.GetWidth()));
			layers_.push_back(layer);
			if (d["Undergrowth Layer"].HasMember("Material")) {
				Material material;
				if (d["Undergrowth Layer"]["Material"].HasMember("Spectrum File")) {
					material.refl = d["Undergrowth Layer"]["Material"]["Spectrum File"].GetString();
				}
				if (d["Undergrowth Layer"]["Material"].HasMember("Kd")) {
					material.kd.r = d["Undergrowth Layer"]["Material"]["Kd"][0].GetFloat();
					material.kd.g = d["Undergrowth Layer"]["Material"]["Kd"][1].GetFloat();
					material.kd.b = d["Undergrowth Layer"]["Material"]["Kd"][2].GetFloat();
				}
				layers_[layers_.size() - 1].SetMaterial(material);
			}
		}
	}



	if (layers_.size() < 3) {
		std::cerr << "There was an error loading layers, three layers are needed" << std::endl;
	}
	//InitTrail();
}
/*
void LayeredSurface::InitTrail(){
	if (trails_.size() > 0) {
		for (int it = 0; it < trails_.size(); it++) {
			glm::vec2 ll = trails_[it].GetLowerLeft();
			glm::vec2 ur = trails_[it].GetUpperRight();
			if (ll.x < trail_ll_.x)trail_ll_.x = ll.x;
			if (ll.y < trail_ll_.y)trail_ll_.y = ll.y;
			if (ur.x > trail_ur_.x)trail_ur_.x = ur.x;
			if (ur.y > trail_ur_.y)trail_ur_.y = ur.y;
		}
		
		glm::vec2 v = trail_ur_ - trail_ll_;
		trail_width_ = trails_[0].GetTrailWidth();
		mapres_ = 2.0f*trail_width_;
		nx_ = (int)ceil((v.x+trail_width_) / mapres_)+1;
		ny_ = (int)ceil((v.y+trail_width_) / mapres_)+1;
		on_trail_ = mavs::utils::Allocate2DVector(nx_, ny_, false);
		float dist = trail_width_ + (float)kSqrt2 * mapres_;
#ifdef USE_OMP
#pragma omp parallel for schedule(dynamic)
#endif
		for (int i = 0; i < nx_; i++) {
			float x = trail_ll_.x + i * mapres_;
			for (int j = 0; j < ny_; j++) {
				float y = trail_ll_.y + j * mapres_;
				glm::vec2 p(x, y);
				for (int it = 0; it < trails_.size(); it++) {
					float dp;
					if (trails_[it].IsPointOnTrail(p, dist,dp)) {
						on_trail_[i][j] = true;
					}
				}
			}
		}
		
	} // if trails_.size()>0;
}
*/

bool LayeredSurface::IsPointOnTrail(float x, float y) {
	glm::vec2 p(x, y);
	//float width = 0.5f*trail_.GetTrailWidth();
	bool on_trail = false;
	float dp;
	for (int i = 0; i < trails_.size(); i++) {
		if (trails_[i].IsPointOnTrail(p, trail_width_,dp)) {
			on_trail = true;
			break;
		}
	}
	return on_trail;
}

Material* LayeredSurface::GetColorAndNormalAtPoint(glm::vec2 p, glm::vec3 &color, glm::vec3 &normal) {
	// determine the location of the pixel - track, trail, or forest
/*
	int i = (int)floor((p.x-trail_ll_.x)/mapres_);
	int j = (int)floor((p.y-trail_ll_.y)/mapres_);
	if (i>=0 && i<nx_ && j>=0 && j<ny_){
	if (on_trail_[i][j]){
*/
	Material *material = layers_[0].GetMaterial(); //new Material;
	bool on_anything = false;
	float dist_to_trail = 100.0;
	for (int it = 0; it < trails_.size(); it++) {
		float dp;
		bool on_trail = trails_[it].IsPointOnTrail(p, trail_width_, dp);
		if (dp < dist_to_trail)dist_to_trail = dp;
		if (on_trail) {
			/*if (trails_[it].IsPointInTracks(p)) {// soil / tracks
				GetLayerColorAndNormalAtPoint(p, 0, color, normal);
				on_anything = true;
			}
			else { //grass/ trail
				*/
				GetLayerColorAndNormalAtPoint(p, 1, color, normal);
				material = layers_[1].GetMaterial();
				on_anything = true;
			//}
		}
	}
	if (!on_anything) { //leaf litter
		if (dist_to_trail < (2.0*trail_width_)) {
			float a = (dist_to_trail - trail_width_);
			float s = 1.0f - (float)exp(-a * a);
			glm::vec3 c1, c2;
			GetLayerColorAndNormalAtPoint(p, 1, c1, normal);
			material = layers_[1].GetMaterial();
			GetLayerColorAndNormalAtPoint(p, 2, c2, normal);
			//std::cout << dist_to_trail << " " << a << " " << s << std::endl;
			color = s * c2 + (1.0f - s)*c1;
		}
		else {
			GetLayerColorAndNormalAtPoint(p, 2, color, normal);
			material = layers_[2].GetMaterial();
		}
	}
	/*
	} // on_trail map
	else {
		GetLayerColorAndNormalAtPoint(p, 2, color, normal);
	}
	} // if i,j
	else { //leaf litter
		GetLayerColorAndNormalAtPoint(p, 2,color,normal);
	}
	*/
	color = color/255.0f;
	return material;
}

void LayeredSurface::GetLayerColorAndNormalAtPoint(glm::vec2 p, int ln, glm::vec3 &color, glm::vec3 &normal) {
	float s = layers_[ln].GetSize();
	float x = p.x / s;
	float y = p.y / s;
	x -= floor(x);
	y -= floor(y);
	int w = layers_[ln].GetWidth();
	int h = layers_[ln].GetHeight();
	int nx = (int)(x * w);
	int ny = (int)(y * h);
	nx = nx % w;
	ny = ny % h;
	layers_[ln].GetColor(nx, ny, color);
	layers_[ln].GetNormal(nx, ny, normal);
}

} //namespace raytracer 
} //namespace mavs