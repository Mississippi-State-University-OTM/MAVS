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
// MPI has to be included first or 
// the compiler chokes up
#ifdef USE_MPI
#include <mpi.h>
#endif

#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>

#include <sensors/camera/lwir_camera.h>

#include <iostream>
#include <ctime>

#include <mavs_core/math/utils.h>

namespace mavs{
namespace sensor{
namespace camera{

float CameraEfficiencyCurve::GetEfficiencyAtWavelength(float wl_microns) {
	float eff = 0.0f;
	// check that the arrays are good
	if (wavelength.size() != efficiency.size() || wavelength.size() == 0 || efficiency.size() == 0)return eff;

	// check that the requested wl is in the range of available data
	if (wl_microns <= wavelength[0])return efficiency[0];
	if (wl_microns >= wavelength.back())return wavelength.back();

	// get the "segment" to use
	int nwl = 0;
	for (int i = 0; i < (int)wavelength.size() - 1; i++) {
		if (wl_microns >= wavelength[i] && wl_microns < wavelength[i + 1]) {
			nwl = i;
			break;
		}
	}

	float m = (efficiency[nwl + 1] - efficiency[nwl]) / (wavelength[nwl + 1] - wavelength[nwl]);
	float b = efficiency[nwl] - m * wavelength[nwl];
	eff = m * wl_microns + b;

	return eff;
}

LwirCamera::LwirCamera(int nx, int ny, float dx, float dy, float focal_length) {
	
  updated_ = false;
  local_sim_time_ = 0.0f;
	Initialize(nx, ny, dx, dy, focal_length);
  gamma_ = 0.75f;
	gain_ = 1.0f; 
	
	pixel_sample_factor_ = 1;

	camera_type_ = "lwir";

	air_temperature_ = 285.0f; // Kelvin
	// 0.73223 + 0.006349 Tdp Celsius, see "DETERMINATION OF THE CLEAR SKY EMISSIVITYFOR USE IN COOL STORAGE ROOF AND ROOF POND APPLICATIONS
	sky_emittance_ = 0.82f; 
	background_color_ = IntegrateLuminance(sky_emittance_, air_temperature_);

	//h_planck_ = 6.62607015e-34f; // J/Hz
	//k_boltzmann_ = 1.380649e-23f; // Joule*Kelvin
	//c_light_ = 299792458.0f; // m/s
	//hc_ = 1.98644586e-25f; // h_planck_ * c_light_;
	
}

LwirCamera::LwirCamera() {

}

LwirCamera::~LwirCamera(){

}

void LwirCamera::LoadThermalData(std::string input_file) {
	if (!mavs::utils::file_exists(input_file)) {
		std::cerr << "ERROR!! Could not find requested thermal input file " << input_file << ", exiting." << std::endl;
		exit(29);
	}

	FILE* fp = fopen(input_file.c_str(), "rb");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	if (d.HasMember("Air Temperature")) {
		air_temperature_ = d["Air Temperature"].GetFloat();
	}

	if (d.HasMember("Dewpoint Celsius")) {
		float dp = d["Dewpoint Celsius"].GetFloat();
		sky_emittance_ = 0.73223f + 0.006349f*dp;	
	}

	background_color_ = IntegrateLuminance(sky_emittance_, air_temperature_);

	if (d.HasMember("Camera Efficiency")) {
		std::vector<float> wl;
		std::vector<float> eff;
		for (int i = 0; i < (int)d["Camera Efficiency"].Capacity(); i++) {
			if (d["Camera Efficiency"][i].Capacity() == 2) {
				wl.push_back(d["Camera Efficiency"][i][0].GetFloat());
				eff.push_back(d["Camera Efficiency"][i][1].GetFloat());
			}
		}
		camera_efficiency_.wavelength = wl;
		camera_efficiency_.efficiency = eff;
	}

	if (d.HasMember("Objects")) {
		for (int i = 0; i < (int)d["Objects"].Capacity(); i++) {
			std::string objname = d["Objects"][i]["Name"].GetString();
			ThermalObject object;
			object.emittance = d["Objects"][i]["Emittance"].GetFloat();
			object.temperature = d["Objects"][i]["Temperature"].GetFloat();
			thermal_objects_[objname] = object;
		}
	}
}

float LwirCamera::IntegrateLuminance(float emittance, float temperature) {
	float l = 0.0f;
	float dlambda = 0.1E-6f; // microns
	float lambda = 8.0E-6f; //microns
	float hc2 = 5.95521486e-17f;  //2*h*c*c
  float	hc_over_k = 0.0143877688f;
	while (lambda < 14.0E-6f) {
		float Rl = camera_efficiency_.GetEfficiencyAtWavelength(1.0E6f*lambda);
		float denom = expf(hc_over_k / (temperature*lambda));
		if (denom > 0.0f) {
			l += Rl * (hc2 / powf(lambda, 5.0f)) / denom;
		};
		lambda += dlambda;
	}
	l = l * emittance*dlambda;
	return l;
}

void LwirCamera::Update(environment::Environment *env, double dt){
  //define directions
  glm::vec3 look_to_f = focal_length_*look_to_;
  glm::vec3 look_side_d = horizontal_pixdim_*look_side_;
  glm::vec3 look_up_d = vertical_pixdim_*look_up_;
	env->GetScene()->TurnOnLabeling();
  ZeroBuffer();

#ifdef USE_OMP  
#pragma omp parallel for schedule(dynamic)
#endif  
  for (int i=0;i<num_horizontal_pix_;i++){
    for (int j=0;j<num_vertical_pix_;j++){

			int n = j + i * num_vertical_pix_;
			
			glm::vec3 cumulative_pix_color(0.0f, 0.0f, 0.0f);
			for (int k = 0; k < pixel_sample_factor_; k++) {
				glm::vec3 direction = look_to_f
					+ (i - half_horizontal_dim_ + mavs::math::rand_in_range(0.0f, 1.0f))*look_side_d
					+ (j - half_vertical_dim_ + mavs::math::rand_in_range(0.0f, 1.0f))*look_up_d;

				direction = glm::normalize(direction);
				glm::vec3 pix_color(background_color_, background_color_, background_color_);

				raytracer::Intersection inter =
					env->GetClosestIntersection(position_, direction);
				if (inter.dist > 0.0f) {
					// only consider emitted radiation in the simplest approximation
					// look up the object temperature and emittance based on the object name
					std::string obj_name = inter.object_name;
					if (inter.label == "rough trail") { obj_name = "rough trail"; }
					if (inter.label == "smooth trail") { obj_name = "smooth trail"; }

					float emittance = thermal_objects_[obj_name].emittance;
					float temperature = thermal_objects_[obj_name].temperature;
					
					// integrate the signal
					float luminance = IntegrateLuminance(emittance, temperature);
					// lambertian model
          float alpha = fabsf(inter.normal.x*direction.x + inter.normal.y*direction.y + inter.normal.z*direction.z);
					//final intensity
					float intens = alpha * luminance;
					pix_color = glm::vec3(intens, intens, intens);
					range_buffer_[n] = inter.dist;
				}

				cumulative_pix_color += pix_color;
			}

			cumulative_pix_color = cumulative_pix_color / (1.0f*pixel_sample_factor_);
			for (int c = 0; c < 3; c++)image_(i, j, c) = cumulative_pix_color[c];
    }
  }

	CopyBufferToImage();
	
	image_.normalize(0, 255);

  if (log_data_) {
    SaveImage(utils::ToString(local_sim_time_)+log_file_name_); 
	}
  local_sim_time_ += local_time_step_;
  updated_ = true;
}

} //namespace camera
} //namespace sensor
} //namespace mavs
