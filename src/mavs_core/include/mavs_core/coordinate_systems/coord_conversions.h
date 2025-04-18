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
 * \class CoordinateConverter
 * Class that converts between latitude/longitude/altitude (LLA), 
 * Universal Transerse Mercator (UTM), Earth-Centered Earth-Fixed, (ECEF), 
 * and East-North-Up (ENU) coordinate systems.
 * Some code modified from from the code posted on 
 * "www.gpsy.com/gpsinfo/geotoutm", 4 Dec 2017 by Chuck Gantz
 * Other code modified from MATLAB file exchange, where noted.
 *
 * \author Chris Goodin
 *
 * \date 12/13/2017
 */
#ifndef COORD_CONVERSIONS_H_
#define COORD_CONVERSIONS_H_
#include <math.h>

#include <vector> 
#include <string> 

#include <mavs_core/coordinate_systems/ellipsoid.h>
#include <mavs_core/math/matrix.h>

namespace mavs{
  //namespace environment{
  
namespace coordinate_system{
 
  /// Latitude-Longitude-Altitude coordinates.
  struct LLA{
    double latitude;
    double longitude;
    double altitude;
  };
  
  /// Earth-Centered, Earth-Fixed coordinates.
  struct ECEF{
    double x;
    double y;
    double z;
  };
  
  /// Local East-North-Up coordinates.
  struct ENU{
    double x;
    double y; 
    double z;
  };

  /// Universal Transverse Mercator coordinates.
  struct UTM{
    double x;
    double y;
    double altitude;
    char zone_char;
    int zone_num;
  };
  
  //} //namespace coordinate_system

///CoordinateConverter class. By default the reference ellipsoid is WGS84.
class CoordinateConverter{
 public:

  /// Create a CoordinateConverter
  CoordinateConverter();

  /// Set the reference ellipsoid using codes defined in ellipsoid.h
  void SetReferenceEllipsoid(int ellips);

  coordinate_system::ECEF LLA2ECEF(coordinate_system::LLA lla);
  
  coordinate_system::LLA ECEF2LLA(coordinate_system::ECEF ecef);
  
  coordinate_system::LLA UTM2LLA(coordinate_system::UTM utm);
 
  coordinate_system::UTM LLA2UTM(coordinate_system::LLA lla);

  coordinate_system::ECEF UTM2ECEF(coordinate_system::UTM utm);
  
  coordinate_system::UTM ECEF2UTM(coordinate_system::ECEF ecef);
  
  coordinate_system::ECEF ENU2ECEF(coordinate_system::ENU enu);
  
  coordinate_system::ENU ECEF2ENU(coordinate_system::ECEF ecef);

  coordinate_system::LLA ENU2LLA(coordinate_system::ENU enu);

  coordinate_system::ENU LLA2ENU(coordinate_system::LLA lla);

  coordinate_system::UTM ENU2UTM(coordinate_system::ENU enu);

  coordinate_system::ENU UTM2ENU(coordinate_system::UTM utm);

  /// Set the local origin in UTM coordinates.
  void SetLocalOrigin(coordinate_system::UTM utm);

  /// Set the local origin in Lat-Long-Alt coordinates.
  void SetLocalOrigin(coordinate_system::LLA lla);

  /// Set the local origin in LLA without using a structure
  void SetLocalOrigin(double lat, double lon, double alt);

  /// Set the local origin in ECEF coordinates.
  void SetLocalOrigin(coordinate_system::ECEF ecef);

 private:
  int ref_ellips_;
  coordinate_system::ECEF local_origin_ecef_;
  coordinate_system::LLA  local_origin_lla_;
  mavs::math::Matrix rot_transpose_;
  mavs::math::Matrix rot_transpose_inverse_;
  mavs::math::Matrix reference_position_;
  Ellipsoid ellipsoid_;
  double a_; 
  double e_; 
  double e2_; 
  double one_minus_e2; 

  char UTMLetterDesignator(double lat);
  void SetMatrices();
};

} //namespace coordinate_system
} //namespace mavs

#endif
