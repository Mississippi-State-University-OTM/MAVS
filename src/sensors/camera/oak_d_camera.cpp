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
#include <sensors/camera/oak_d_camera.h>
#include <iostream>
#include <algorithm>
#include <ctime>
#include <limits>
#include <glm/glm.hpp>
#include <mavs_core/math/utils.h>
#include <raytracers/fresnel.h>
#ifdef USE_OMP
#include <omp.h>
#endif

namespace mavs{
namespace sensor{
namespace camera{

OakDCamera::OakDCamera(){
    baseline_cm_ = 7.5f;
    block_size_ = 5;
    int offset = 2 * block_size_;
	//left_cam_.Initialize(1280, 800, 1280.0f * 3.0E-6f, 800.0f * 3.0E-6f, 0.00235f);
	//right_cam_.Initialize(1280, 800, 1280.0f * 3.0E-6f, 800.0f * 3.0E-6f, 0.00235f);
	left_cam_.Initialize(640+offset, 400, (1280.0f + 2.0f*offset)* 3.0E-6f, 800.0f * 3.0E-6f, 0.00235f);
	right_cam_.Initialize(640+offset, 400, (1280.0f + 2.0f*offset) * 3.0E-6f, 800.0f * 3.0E-6f, 0.00235f);
	//center_cam_.Initialize(4056, 3040, 4056.0f * 1.55E-6f, 3040.0f * 1.55E-6f, 0.00481f);
	//center_cam_.Initialize(1014, 760, 4056.0f * 1.55E-6f, 3040.0f * 1.55E-6f, 0.00481f);
    center_cam_.Initialize(640, 400, 1280.0f * 3.0E-6f, 800.0f * 3.0E-6f, 0.00235f);
	left_cam_.SetRelativePose(glm::vec3(0.0f, 0.5f * baseline_cm_ / 100.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	right_cam_.SetRelativePose(glm::vec3(0.0f, -0.5f * baseline_cm_ / 100.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    max_depth_cm_ = 1200.0f;
    left_cam_.SetAntiAliasing("corners");
    left_cam_.SetPixelSampleFactor(4);
    right_cam_.SetAntiAliasing("corners");
    right_cam_.SetPixelSampleFactor(4);
    center_cam_.SetAntiAliasing("corners");
    center_cam_.SetPixelSampleFactor(4);
    display_type_ = "both";
}

OakDCamera::~OakDCamera(){

}

OakDCamera::OakDCamera(const OakDCamera& cam) {
    max_depth_cm_ = cam.max_depth_cm_;
    left_cam_ = cam.left_cam_;
    right_cam_ = cam.right_cam_;
    center_cam_ = cam.center_cam_;
    depth_disp_ = cam.depth_disp_;
    depth_cm_ = cam.depth_cm_;
    block_size_ = cam.block_size_;
    baseline_cm_ = cam.baseline_cm_;
    display_type_ = cam.display_type_;
}

void OakDCamera::SetBlockSize(int bs) {
    block_size_ = bs;
    int offset = 2 * block_size_;
    left_cam_.Initialize(640 + offset, 400, (1280.0f + 2.0f * offset) * 3.0E-6f, 800.0f * 3.0E-6f, 0.00235f);
    right_cam_.Initialize(640 + offset, 400, (1280.0f + 2.0f * offset) * 3.0E-6f, 800.0f * 3.0E-6f, 0.00235f);
}

void OakDCamera::SetPose(glm::vec3 pos, glm::quat ori) {
	center_cam_.SetPose(pos, ori);
}

void OakDCamera::SetRelativePose(glm::vec3 pos, glm::quat ori) {
	center_cam_.SetRelativePose(pos, ori);
}

void OakDCamera::Update(environment::Environment* env, double dt) {
	center_cam_.Update(env, dt);
    Pose pose = center_cam_.GetPose(); // this includes the offsets that have been set
    left_cam_.SetPose(pose.position, pose.quaternion);
    right_cam_.SetPose(pose.position, pose.quaternion);
	left_cam_.Update(env, dt);
	right_cam_.Update(env, dt);
    GetDepth();
}

void OakDCamera::Display() {
	if (display_type_=="rgb" || display_type_=="both")center_cam_.Display();
    if (display_type_ == "range" || display_type_ == "both") {
        if (depth_cm_.height() > 0) {
            if (depth_disp_.height() == 0) {
                depth_disp_.assign(depth_cm_.width(), depth_cm_.height(), "Depth (cm)", 1);
            }
            depth_disp_ = depth_cm_;
        }
    }
    if (display_type_!="rgb" && display_type_!="both" && display_type_!="range"){
        center_cam_.Display();
    }
}

float OakDCamera::GetCmDepthAtPixel(int u, int v) {
    float depth = max_depth_cm_;
    if (u >= 0 && v >= 0 && u < depth_cm_.width() && v < depth_cm_.height()) {
        depth = (max_depth_cm_ / 255.0f) * depth_cm_(u, v, 0);
    }
    return depth;
}

float OakDCamera::GetCmDepthErrorAtPixel(int u, int v) {
    float derr = 0.0f;
    if (u >= 0 && v >= 0 && u < depth_cm_.width() && v < depth_cm_.height()) {
        derr = depth_error_[u][v];
    }
    return derr;
}

glm::vec3 OakDCamera::GetColorFromDepth(float depth_cm) {
    glm::vec3 color(0.0f, 0.0f, 0.0f);
    float w = std::min(depth_cm / max_depth_cm_, 1.0f);
    color.r = 255.0f * w;
    color.b = 255.0f * (1.0f - w);
    return color;
}

void OakDCamera::GetDepth() {
    // see: https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth#how-baseline-distance-and-focal-length-affect-depth

    cimg_library::CImg<float> left_image = *left_cam_.GetCImg();
    cimg_library::CImg<float> right_image = *right_cam_.GetCImg();

    int width = left_image.width();
    int height = left_image.height();
    int channels = left_image.spectrum();

    cimg_library::CImg<float> depth_tmp;
    depth_tmp.assign(width, height, 1, 3, 0);

    int offset = 2 * block_size_;
    if (depth_cm_.width() == 0) {
        depth_cm_.assign(width - offset, height, 1, 3, 0);
        depth_error_ = utils::Allocate2DVector(width - offset, height, 0.0f);
    }

    int max_disparity = std::max(2*block_size_,2*(int)(441.24f * baseline_cm_ / max_depth_cm_)); //64;
#ifdef USE_OMP
#pragma omp parallel for
#endif
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {

            int best_match = 0;
            int min_sad = INT_MAX;

            for (int d = 0; d < max_disparity; d++) {
                
                if (x - d < 0) {
                    glm::vec3 color = GetColorFromDepth(max_depth_cm_);
                    depth_tmp.draw_point(x, y, (float*)&color);
                    depth_error_[x][y] = 0.0f;
                }
                
                int sad = 0;
                
                for (int i = -block_size_ / 2; i <= block_size_ / 2; i++) {
                    for (int j = -block_size_ / 2; j <= block_size_ / 2; j++) {

                        if (y + i < 0 || y + i >= height || x + j - d < 0 || x + j >= width) {
                            continue;
                        }
                        
                        for (int c = 0; c < channels; c++) {
                            sad += (int)abs(left_image(x + j, y + i, 0, c) - right_image(x + j - d, y + i, 0, c));
                        }

                    }
                }
                
                if (sad < min_sad) {
                    min_sad = sad;
                    best_match = d;
                }
                
            }
            float depth = max_depth_cm_;
            if (best_match > 0) {
                depth = 441.25f * (baseline_cm_ / (float)best_match);   
            }
            if (x>=offset)depth_error_[x-offset][y] = (depth * depth / (baseline_cm_ * 100.0f * left_cam_.GetFocalLength())) * best_match;
            glm::vec3 color = GetColorFromDepth(depth);
            depth_tmp.draw_point(x, y, (float*)&color);
        }
    }
    depth_cm_ = depth_tmp.get_crop(offset, 0, depth_tmp.width()-1, depth_tmp.height()-1);
}

} //namespace camera
} //namespace sensor
} //namespace mavs
