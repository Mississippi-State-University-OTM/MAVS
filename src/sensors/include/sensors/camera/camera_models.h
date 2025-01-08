/*
Non-Commercial License - Mississippi State University Autonomous Vehicle Software (MAVS)

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

*NOTICE*
Thank you for your interest in MAVS, we hope it serves you well!  We have a few small requests to ask of you that will help us continue to create innovative platforms:
-To share MAVS with a friend, please ask them to visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to register and download MAVS for free.
-Feel free to create derivative works for non-commercial purposes, i.e. academics, U.S. government, and industry research.  If commercial uses are intended, please contact us for a commercial license at otm@msstate.edu or jonathan.rudd@msstate.edu.
-Finally, please use the ACKNOWLEDGEMENT above in your derivative works.  This helps us both!

Please visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to view the full license, or email us if you have any questions.

Copyright 2018 (C) Mississippi State University
*/
/**
 * \file camera_models.h
 *
 * Declares several default camera models
 *
 * \author Chris Goodin
 *
 * \date 3/23/2018
 */  
#ifndef CAMERA_MODELS_H
#define CAMERA_MODELS_H

#include <sensors/camera/rgb_camera.h>
#include <sensors/camera/path_tracer.h>
#include <sensors/camera/simple_camera.h>

namespace mavs{
namespace sensor{
namespace camera{

///Simple camera, color, with 3.5 mm lens
class LowRes : public SimpleCamera {
public:
	/// Create simple camera sensor with 3.5 mm lens
	LowRes() {
		Initialize(384, 384, 0.0035f, 0.0035f, 0.0035f);
	}

};

///Machine vision camera for use with neural network
class MachineVision : public RgbCamera {
public:
	/// Create simple camera sensor with 3.5 mm lens
	MachineVision() {
		Initialize(224, 224, 0.0035f, 0.0035f, 0.0035f);
		SetGamma(0.6f);
		SetAntiAliasing("oversampled");
		SetPixelSampleFactor(3);
	}

};

///Machine vision path traced camera for use with neural network
class MachineVisionPathTraced : public PathTracerCamera {
public:
	/// Create simple camera sensor with 3.5 mm lens
	MachineVisionPathTraced(int num_iter, int max_depth, float rr_val) {
		Initialize(224, 224, 0.0035f, 0.0035f, 0.0035f);
		SetGamma(0.6f);
		SetNumIterations(num_iter);
		SetMaxDepth(max_depth);
		SetRRVal(rr_val);
	}
};

///Hi-def camera for cool visulizations. SLOW!
class HDPathTraced : public PathTracerCamera {
public:
	/// Create simple camera sensor with 3.5 mm lens
	HDPathTraced(int num_iter, int max_depth, float rr_val) {
		Initialize(1620, 1080, 0.0225f, 0.015f, 0.009f);
		SetGamma(0.6f);
		SetNumIterations(num_iter);
		SetMaxDepth(max_depth);
		SetRRVal(rr_val);
	}
};

/// Phantom4 camera using path tracing
class Phantom4CameraPathTraced : public PathTracerCamera {
	// see: https://www.dji.com/phantom-4-pro/info#specs
public:
	/// Initialize with number of iterations, max depth, and cutoff value
	Phantom4CameraPathTraced(int num_iter, int max_depth, float rr_val) {
		//Initialize(4864, 3648, 0.02032f, 0.01524f, 0.028f);
		Initialize(1216, 912, 0.02032f, 0.01524f, 0.028f);
		SetGamma(0.85f);
		SetNumIterations(num_iter);
		SetMaxDepth(max_depth);
		SetRRVal(rr_val);
	}
};

/// Lower res Phantom4 camera using path tracing
class Phantom4CameraPathTracedLowRes : public PathTracerCamera {
	// see: https://www.dji.com/phantom-4-pro/info#specs
public:
	/// Initialize with number of iterations, max depth, and cutoff value
	Phantom4CameraPathTracedLowRes(int num_iter, int max_depth, float rr_val) {
		Initialize(304, 228, 0.02032f, 0.01524f, 0.028f);
		SetGamma(0.85f);
		SetNumIterations(num_iter);
		SetMaxDepth(max_depth);
		SetRRVal(rr_val);
	}
};

/// Low-res Phantom4 camera using primary rays only
class Phantom4CameraLowRes : public RgbCamera {
public:
	Phantom4CameraLowRes() {
		Initialize(304, 228, 0.02032f, 0.01524f, 0.028f);
		SetGamma(0.6f);
	}
};


/// Low-res Phantom4 camera using primary rays only
class Phantom4Camera : public RgbCamera {
public:
	Phantom4Camera() {
		//Initialize(4864, 3648, 0.02032f, 0.01524f, 0.028f);
		Initialize(1216, 912, 0.02032f, 0.01524f, 0.028f);
		SetGamma(0.6f);
	}
};


/// UAV camera using path tracing
class UavCameraPathTraced : public PathTracerCamera {
	// see: https://www.dji.com/mavic-2/info#specs
public:
	/// Initialize with number of iterations, max depth, and cutoff value
	UavCameraPathTraced(int num_iter, int max_depth, float rr_val) {
		//Initialize(5472, 3648, 0.05568f, 0.03712f, 0.035f);
		Initialize(1368, 912, 0.05568f, 0.03712f, 0.035f);
		SetGamma(0.85f);
		SetNumIterations(num_iter);
		SetMaxDepth(max_depth);
		SetRRVal(rr_val);
	}
};

/// UAV camera using path tracing
class UavCameraPathTracedLowRes : public PathTracerCamera {
	// see: https://www.dji.com/mavic-2/info#specs
public:
	/// Initialize with number of iterations, max depth, and cutoff value
	UavCameraPathTracedLowRes(int num_iter, int max_depth, float rr_val) {
		Initialize(342, 228, 0.05568f, 0.03712f, 0.035f);
		SetGamma(0.85f);
		SetNumIterations(num_iter);
		SetMaxDepth(max_depth);
		SetRRVal(rr_val);
	}
};

/// UAV camera using path tracing
class UavCameraLowRes : public RgbCamera {
public:
	UavCameraLowRes() {
		Initialize(684, 456, 0.05568f, 0.03712f, 0.035f);
		SetGamma(0.6f);
	}
};

///Hi-def camera for cool visulizations. SLOW!
class HalfHDPathTraced : public PathTracerCamera {
public:
	/// Create simple camera sensor with 3.5 mm lens
	HalfHDPathTraced(int num_iter, int max_depth, float rr_val) {
		Initialize(810, 540, 0.0225f, 0.015f, 0.009f);
		SetGamma(0.6f);
		SetNumIterations(num_iter);
		SetMaxDepth(max_depth);
		SetRRVal(rr_val);
	}
};

///Flea3, color, with 4 mm lens
class Flea3_4mm : public RgbCamera {
 public:
  /// Create Flea3 camera sensor with 4 mm lens
  Flea3_4mm(){
    Initialize(1280,1024,0.006784f,0.0054272f,0.004f);
  }

};

///Sony XCD-V60, color, with 3.5 mm lens
class XCD_V60 : public RgbCamera {
 public:
  /// Create XCD-V60 camera sensor with 3.5 mm lens
  XCD_V60(){
    Initialize(640,480,0.0024f,0.0018f,0.0035f);
		gamma_ = 0.75f;
		gain_ = 0.85f;
		//anti_aliasing_type_ = "adaptive";
  }
};

///Sony 1080P Hi-def camera, 
class HD1080 : public RgbCamera {
public:
	/// Create 1080P camera 
	HD1080() {
		Initialize(1620, 1080, 0.0225f, 0.015f, 0.009f);
		gamma_ = 0.65f;
		gain_ = 1.0f;
		SetAntiAliasing("oversampled");
		SetPixelSampleFactor(5);
		SetExposureTime(0.002f);
	}
};

class Sf3325: public RgbCamera {
public:
	/**
	* Sekonix SF3325-100
	* See: http://sekolab.com/wp-content/uploads/2020/02/SF332X-10X_2Mega-LVDS-Automotive-Camera-Datasheet_Ver-2.2.5_190726.pdf
	*/
	Sf3325() {
		Initialize(1928, 1208, 0.005784f, 0.003624f, 0.005f);
		gamma_ = 0.65f;
		gain_ = 1.0f;
		SetAntiAliasing("oversampled");
		SetPixelSampleFactor(5);
		SetExposureTime(0.002f);
	}
};

/// Sekonix camera using path tracing
class Sf3325PathTraced : public PathTracerCamera {
	// see: https://www.dji.com/mavic-2/info#specs
public:
	/// Initialize with number of iterations, max depth, and cutoff value
	Sf3325PathTraced(int num_iter, int max_depth, float rr_val) {
		Initialize(1928, 1208, 0.005784f, 0.003624f, 0.005f);
		SetGamma(0.85f);
		SetNumIterations(num_iter);
		SetMaxDepth(max_depth);
		SetRRVal(rr_val);
	}
};

/// Sekonix camera using path tracing
class Sf3325PathTracedMedium : public PathTracerCamera {
	// see: https://www.dji.com/mavic-2/info#specs
public:
	/// Initialize with number of iterations, max depth, and cutoff value
	Sf3325PathTracedMedium(int num_iter, int max_depth, float rr_val) {
		Initialize(1024, 642, 0.005784f, 0.003624f, 0.005f);
		SetGamma(0.85f);
		SetNumIterations(num_iter);
		SetMaxDepth(max_depth);
		SetRRVal(rr_val);
	}
};

/// Sekonix camera using path tracing
class Sf3325PathTracedLow : public PathTracerCamera {
	// see: https://www.dji.com/mavic-2/info#specs
public:
	/// Initialize with number of iterations, max depth, and cutoff value
	Sf3325PathTracedLow(int num_iter, int max_depth, float rr_val) {
		Initialize(241, 151, 0.005784f, 0.003624f, 0.005f);
		SetGamma(0.85f);
		SetNumIterations(num_iter);
		SetMaxDepth(max_depth);
		SetRRVal(rr_val);
	}
};

} //namespace mavs
} //namespace sensor
} //namespace camera

#endif