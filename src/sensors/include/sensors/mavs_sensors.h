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
#pragma once
//mavs sensors
//cameras
#include <sensors/camera/rgb_camera.h>
#include <sensors/camera/rccb_camera.h>
#include <sensors/camera/fisheye_camera.h>
#include <sensors/camera/simple_camera.h>
#include <sensors/camera/camera_models.h>
#include <sensors/camera/spherical_camera.h>
#include <sensors/camera/micasense_red_edge.h>
#include <sensors/camera/nir_camera.h>
#include <sensors/camera/oak_d_camera.h>
#include <sensors/camera/lwir_camera.h>
//compass
#include <sensors/compass/compass.h>
//gps
#include <sensors/gps/gps.h>
// imu
#include <sensors/imu/imu.h>
#include <sensors/imu/imu_simple.h>
#include <sensors/imu/mems_sensor.h>
#include <sensors/imu/gyro.h>
#include <sensors/imu/accelerometer.h>
#include <sensors/imu/magnetometer.h>
//lidar
#include <sensors/lidar/lidar.h>
#include <sensors/lidar/lms_291.h>
#include <sensors/lidar/hdl64e.h>
#include <sensors/lidar/hdl64e_simple.h>
#include <sensors/lidar/hdl512e.h>
#include <sensors/lidar/hdl32e.h>
#include <sensors/lidar/vlp16.h>
#include <sensors/lidar/m8.h>
#include <sensors/lidar/os1.h>
#include <sensors/lidar/os1_16.h>
#include <sensors/lidar/os2.h>
#include <sensors/lidar/robosense32.h>
#include <sensors/lidar/four_pi.h>
#include <sensors/lidar/anvel_api_lidar.h>
//radar
#include <sensors/radar/radar.h>
#include <sensors/radar/radar_models.h>
//notional
#include <sensors/occupancy_grid_detector/occupancy_grid_detector.h>
#include <sensors/object_detector/object_detector.h>
//ins
#include <sensors/ins/swiftnav_piksi.h>
//rtk
#include <sensors/rtk/rtk.h>