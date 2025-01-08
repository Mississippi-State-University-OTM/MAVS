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
 * \class SphericalCamera
 *
 * A color camera that renders a spherical projection
 * of the hemisphere above the camera
 * 
 * \author Chris Goodin
 *
 * \date 10/1/2018
 */

#ifndef SPHERICAL_CAMERA_H
#define SPHERICAL_CAMERA_H

#include <sensors/camera/camera.h>

namespace mavs{
namespace sensor{
namespace camera{

class SphericalCamera : public Camera {
 public:
  SphericalCamera();
  ~SphericalCamera();

  SphericalCamera(const SphericalCamera &cam){
    //image_buffer_ =  cam.image_buffer_;
    image_ = cam.image_;
    num_horizontal_pix_ = cam.num_horizontal_pix_;
    num_vertical_pix_ = cam.num_vertical_pix_;
    focal_length_ = cam.focal_length_;
    focal_array_width_ = cam.focal_array_width_;
    focal_array_height_ = cam.focal_array_height_;
    horizontal_pixdim_ = cam.horizontal_pixdim_;
    vertical_pixdim_ = cam.vertical_pixdim_;
    half_horizontal_dim_ = cam.half_horizontal_dim_;
    half_vertical_dim_ = cam.half_vertical_dim_;
    gamma_ = cam.gamma_;
    gain_ = cam.gain_;
  }

  /**
   * Updates the spherical camera image.
   * \param env Pointer to the environment data structure.
   * \param dt Time step of render
   */
  void Update(environment::Environment *env, double dt);


private:

};

} //namespace camera
} //namespace sensor
} //namespace mavs

#endif
