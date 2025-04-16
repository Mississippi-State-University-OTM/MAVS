/**
 * \class Zed2iCamera
 *
 * ZED2i stereo camera implementation for MAVS
 *
 * \author [Your Name]
 * \date [Current Date]
 */

#ifndef ZED2I_CAMERA_H
#define ZED2I_CAMERA_H

#include <sensors/camera/rgb_camera.h>

namespace mavs {
namespace sensor {
namespace camera {

class Zed2iCamera : public Sensor {
public:
    /// Depth processing modes
    enum class DepthMode {
        ULTRA,       // Highest quality (7x7 window)
        NEURAL,      // AI-enhanced (5x5 window)
        PERFORMANCE  // Fastest (3x3 window)
    };

    /// Constructor
    Zed2iCamera();

    /// Destructor
    ~Zed2iCamera();

    /// Copy constructor
    Zed2iCamera(const Zed2iCamera& cam);

    /// Inherited sensor update method
    void Update(environment::Environment* env, double dt) override;

    /// Set sensor pose
    void SetPose(glm::vec3 pos, glm::quat ori) override;

    /// Set relative sensor pose
    void SetRelativePose(glm::vec3 pos, glm::quat ori) override;

    /// Display camera output
    void Display();

    /// Set maximum measurable depth in meters
    void SetMaxDepth(float md) { max_depth_cm_ = 100.0f * md; }

    /// Set depth processing mode
    void SetDepthMode(DepthMode mode) { depth_mode_ = mode; }

    /// Set confidence threshold (0-100)
    void SetConfidenceThreshold(int threshold) { 
        depth_confidence_threshold_ = std::clamp(threshold, 0, 100); 
    }

    /// Toggle temporal filtering
    void EnableTemporalFilter(bool enable) { temporal_stabilization_ = enable; }

    /// Toggle spatial filtering
    void EnableSpatialFilter(bool enable) { spatial_filtering_ = enable; }

    /// Get depth at pixel (cm)
    float GetDepthAtPixel(int u, int v);

    /// Get depth error at pixel (cm)
    float GetDepthErrorAtPixel(int u, int v);

    /// Get current camera pose
    Pose GetPose() { return left_cam_.GetPose(); }

    // Image dimensions
    int GetWidth() { return left_cam_.GetWidth(); }
    int GetHeight() { return left_cam_.GetHeight(); }
    int GetDepthImageWidth() { return left_cam_.GetWidth(); } // Full resolution
    int GetDepthImageHeight() { return left_cam_.GetHeight(); }

    // Buffer access
    float* GetImageBuffer() { return left_cam_.GetImageBuffer(); }
    int GetBufferSize() { return left_cam_.GetBufferSize(); }
    float* GetDepthBuffer() { return depth_cm_.data(); }
    int GetDepthBufferSize() { return depth_cm_.size(); }

    // Configuration
    void SetDisplayType(const std::string& display_type) { 
        display_type_ = display_type; 
    }

private:
    // Stereo cameras
    RgbCamera left_cam_;
    RgbCamera right_cam_;

    // Depth processing
    DepthMode depth_mode_;
    float max_depth_cm_;
    float min_depth_cm_;
    float baseline_cm_;
    int depth_confidence_threshold_;
    bool temporal_stabilization_;
    bool spatial_filtering_;

    // Output
    cimg_library::CImgDisplay depth_disp_;
    cimg_library::CImg<float> depth_cm_;
    cimg_library::CImg<unsigned char> confidence_map_;
    std::vector<std::vector<float>> depth_error_;
    std::string display_type_;

    // Core functions
    void ComputeDepth();
    int FindBestDisparity(int x, int y, 
                         const cimg_library::CImg<float>& left,
                         const cimg_library::CImg<float>& right);
    void ApplyTemporalFilter(cimg_library::CImg<float>& depth);
    void ApplySpatialFilter(cimg_library::CImg<float>& depth);
    glm::vec3 GetColorFromDepth(float depth_cm);
    int GetWindowSizeForMode() const;
};

} // namespace camera
} // namespace sensor
} // namespace mavs

#endif // ZED2I_CAMERA_H
