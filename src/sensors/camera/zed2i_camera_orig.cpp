/*
MIT License

Copyright (c) 2024 Mississippi State University

Permission is hereby granted... [license text unchanged]
*/

// ===== CONFIGURATION DEFINES ===== //
//#define USE_NEURAL_ACCELERATION  // Uncomment for AI depth enhancement (requires OpenCV DNN)
#define USE_OMP                   // Comment out to disable multi-core processing
//#define USE_MPI                  // Uncomment for MPI support

#ifdef USE_MPI
#include <mpi.h>
#endif
#include <sensors/camera/zed2i_camera.h>
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

namespace mavs {
namespace sensor {
namespace camera {

// ===== HARDWARE CONSTANTS (Calibration Data) ===== //
constexpr float ZED2I_BASELINE = 12.0f;       //# Camera baseline in cm (12cm for ZED2i)
constexpr int MAX_DISPARITY = 256;            //# Hardware disparity limit (pixels)
constexpr float ZED2I_FOCAL_LENGTH = 700.0f;  //# Focal length in pixels (calibration sheet)
constexpr int DEFAULT_WIDTH = 2208;           //# 2K mode: 2208x1242 (adjust for other resolutions)
constexpr int DEFAULT_HEIGHT = 1242;

Zed2iCamera::Zed2iCamera() {
    // ===== RUNTIME CONFIGURATION ===== //
    baseline_cm_ = ZED2I_BASELINE;
    depth_mode_ = DepthMode::NEURAL;          //# ULTRA|NEURAL|PERFORMANCE
    depth_confidence_threshold_ = 50;         //# Range 0-100 (recommended 40-70)
    
    // ===== SENSOR INITIALIZATION ===== //
    const float pixel_pitch = 2.0E-6f;        //# Pixel size in meters (2μm for ZED2i)
    left_cam_.Initialize(DEFAULT_WIDTH, DEFAULT_HEIGHT, 
                        DEFAULT_WIDTH * pixel_pitch,
                        DEFAULT_HEIGHT * pixel_pitch,
                        0.0028f);             //# Lens aperture diameter (m)
    
    right_cam_.Initialize(DEFAULT_WIDTH, DEFAULT_HEIGHT,
                        DEFAULT_WIDTH * pixel_pitch,
                        DEFAULT_HEIGHT * pixel_pitch,
                        0.0028f);

    // ===== STEREO RIG CONFIGURATION ===== //
    const float y_offset = baseline_cm_ / 200.0f; //# Convert 12cm baseline to ±6cm offsets
    left_cam_.SetRelativePose(glm::vec3(0.0f, y_offset, 0.0f), 
                             glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    right_cam_.SetRelativePose(glm::vec3(0.0f, -y_offset, 0.0f),
                              glm::quat(1.0f, 0.0f, 0.0f, 0.0f));

    // ===== DEPTH RANGE SETTINGS ===== //
    max_depth_cm_ = 2000.0f;  //# 20m max (25m possible but less accurate)
    min_depth_cm_ = 50.0f;    //# 50cm min (due to baseline)

    // ===== QUALITY SETTINGS ===== //
    left_cam_.SetAntiAliasing("corners");    //# "none"|"corners"|"full"
    left_cam_.SetPixelSampleFactor(4);       //# 1-4 (higher=better quality)
    right_cam_.SetAntiAliasing("corners");
    right_cam_.SetPixelSampleFactor(4);

    // ===== POST-PROCESSING ===== //
    display_type_ = "both";                 //# "both"|"rgb"|"range"
    temporal_stabilization_ = true;         //# Reduces flickering
    spatial_filtering_ = true;              //# Reduces noise
}

void Zed2iCamera::GetDepth() {
    // Get stereo image pair
    cimg_library::CImg<float> left_image = *left_cam_.GetCImg();
    cimg_library::CImg<float> right_image = *right_cam_.GetCImg();
    const int width = left_image.width();
    const int height = left_image.height();
    const int channels = left_image.spectrum();

    // Initialize buffers on first run
    if (depth_cm_.width() == 0) {
        depth_cm_.assign(width, height, 1, 3, 0);          // RGB depth map
        depth_error_ = utils::Allocate2DVector(width, height, 0.0f);  // Error in cm
        confidence_map_.assign(width, height, 1, 1, 0);    // Confidence 0-255
    }

    // Calculate max disparity (clamped to hardware limit)
    int max_disparity = static_cast<int>(ZED2I_FOCAL_LENGTH * baseline_cm_ / min_depth_cm_);
    max_disparity = std::min(max_disparity, MAX_DISPARITY);

    // Main processing loop
    #ifdef USE_OMP
    #pragma omp parallel for collapse(2)  //# Parallelize over image area
    #endif
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Stereo matching
            int best_disp = FindBestDisparity(x, y, left_image, right_image, 
                                            max_disparity, channels);
            
            // Convert to depth
            float depth_cm = (best_disp > 0) ? 
                (ZED2I_FOCAL_LENGTH * baseline_cm_) / best_disp :
                max_depth_cm_;
            
            // Store results
            StoreDepthResults(x, y, depth_cm);
        }
    }

    // Post-processing
    if (temporal_stabilization_) {
        ApplyTemporalFilter(depth_tmp); //# Alpha=0.8 blending
    }
    if (spatial_filtering_) {
        ApplySpatialFilter(depth_tmp);  //# Bilateral filter
    }

    depth_cm_ = depth_tmp;
}

// ===== CORE STEREO MATCHING ===== //
int Zed2iCamera::FindBestDisparity(int x, int y, 
                                  const cimg_library::CImg<float>& left,
                                  const cimg_library::CImg<float>& right,
                                  int max_disp, int channels) {
    int best_disp = 0;
    int min_sad = INT_MAX;
    const int window = GetWindowSizeForMode(); //# 3/5/7 based on depth_mode_

    for (int d = 0; d < max_disp; d++) {
        if (x - d < 0) continue; // Skip invalid disparities

        // Compute SAD over window
        int sad = 0, valid_pixels = 0;
        for (int i = -window/2; i <= window/2; i++) {
            for (int j = -window/2; j <= window/2; j++) {
                if (!InBounds(x+j, y+i, left.width(), left.height())) continue;
                
                valid_pixels++;
                for (int c = 0; c < channels; c++) {
                    sad += abs(left(x+j,y+i,0,c) - right(x+j-d,y+i,0,c));
                }
            }
        }

        // Update best match
        if (valid_pixels > 0 && (sad/valid_pixels) < min_sad) {
            min_sad = sad/valid_pixels;
            best_disp = d;
        }
    }

    return best_disp;
}

// ===== DEPTH MODE CONFIGURATION ===== //
int Zed2iCamera::GetWindowSizeForMode() const {
    switch (depth_mode_) {
        case DepthMode::ULTRA: return 7;     //# 7x7 window (highest quality)
        case DepthMode::NEURAL: return 5;    //# 5x5 window (balanced)
        case DepthMode::PERFORMANCE: return 3; //# 3x3 window (fastest)
        default: return 5;
    }
}

// ===== POST-PROCESSING FILTERS ===== //
void Zed2iCamera::ApplyTemporalFilter(cimg_library::CImg<float>& current) {
    static cimg_library::CImg<float> previous = current;
    const float alpha = 0.8f; //# Blend factor (0.5-0.9)
    
    cimg_forXY(current, x, y) {
        for (int c = 0; c < current.spectrum(); c++) {
            current(x,y,0,c) = alpha*current(x,y,0,c) + (1-alpha)*previous(x,y,0,c);
        }
    }
    previous = current;
}

void Zed2iCamera::ApplySpatialFilter(cimg_library::CImg<float>& img) {
    const float sigma_space = 1.5f;  //# Spatial kernel size
    const float sigma_color = 30.0f; //# Color similarity threshold
    
    cimg_library::CImg<float> filtered = img;
    cimg_forXY(img, x, y) {
        if (x <= 0 || y <= 0 || x >= img.width()-1 || y >= img.height()-1) continue;
        
        float sum_weights = 0.0f;
        glm::vec3 sum(0.0f);
        const glm::vec3 center(img(x,y,0,0), img(x,y,0,1), img(x,y,0,2));

        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                glm::vec3 neighbor(img(x+i,y+j,0,0), img(x+i,y+j,0,1), img(x+i,y+j,0,2));
                float space_weight = exp(-(i*i + j*j)/(2.0f*sigma_space*sigma_space));
                float color_weight = exp(-glm::length(center-neighbor)/(2.0f*sigma_color*sigma_color));
                float weight = space_weight * color_weight;
                
                sum += weight * neighbor;
                sum_weights += weight;
            }
        }

        if (sum_weights > 0) {
            filtered(x,y,0,0) = sum.x / sum_weights;
            filtered(x,y,0,1) = sum.y / sum_weights;
            filtered(x,y,0,2) = sum.z / sum_weights;
        }
    }
    img = filtered;
}

} // namespace camera
} // namespace sensor
} // namespace mavs
