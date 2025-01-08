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
#include <sensors/camera/rccb_camera.h>
#include <mavs_core/math/utils.h>

namespace mavs {
namespace sensor {
namespace camera {


void RccbCamera::Update(environment::Environment *env, double dt) {
	CheckFreq(dt);
	RgbCamera::RenderFrame(env, dt);

	// see : Karanam, G. "Interfacing Red/Clear Sensors to ADSP-BF609� Blackfin Processors." Analog Devices, Norwood, Engineer-to-Engineer Note EE-358, Rev 1 (2013). 
	for (int i = 0; i < num_horizontal_pix_; i++) {
		for (int j = 0; j < num_vertical_pix_; j++) {
			int n = j + i * num_vertical_pix_;
			//glm::vec3 oldcol = image_buffer_[n];
			//float new_green = 0.3f*oldcol.x +  0.59f*oldcol.y + 0.11f*oldcol.z;
			float new_green = 0.3f*image_(i,j,0) + 0.59f*image_(i,j,1) + 0.11f*image_(i,j,2);
			//glm::vec3 newcol(oldcol.x, new_green, oldcol.z);
			//image_buffer_[n] = newcol;
			image_(i, j, 1) = new_green;
		}
	}

	CopyBufferToImage();

	if (log_data_) {
		SaveImage(utils::ToString(local_sim_time_) + log_file_name_);
	}

	local_sim_time_ += local_time_step_;
	updated_ = true;
}


} //namespace camera
} //namespace sensor
} //namespace mavs