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
 * \class SolarPosition
 *
 * Code to calculate the horizontal coordinates of the sun (azimuth and zenith)
 * based on the date, time, and observer position in lat/long.
 * Taken from calculations on wikipedia.
 * See https://en.wikipedia.org/wiki/Julian_day
 * See https://en.wikipedia.org/wiki/Celestial_coordinate_system
 *
 * Also calculates the Lunar position, adapted from
 * Jensen, Henrik Wann, et al. "A physically-based night sky model."
 * Proceedings of the 28th annual conference on Computer graphics and interactive techniques.ACM, 2001.
 * 
 *
 * \author Chris Goodin
 *
 * \date 1/4/2018
 */

#ifndef SOLAR_POSITION_H
#define SOLAR_POSITION_H

#include <vector>
#include <string>

#include <mavs_core/environment/date_time.h>
#include <glm/glm.hpp>

namespace mavs{
namespace environment{

/**
 * Star info from the Yale Bright Star Database
 */
struct yale_star {
	double right_ascension;
	double declination;
	double brightness;
	double color_temperature;
	glm::vec3 direction;
};

inline bool operator==(const DateTime& left, const DateTime& right){
  if (left.year == right.year &&
      left.month == right.month &&
      left.day == right.day &&
      left.hour == right.hour &&
      left.minute == right.minute &&
      left.second == right.second &&
      left.time_zone == right.time_zone ){
    return true;
  }
  else {
    return false;
  }
}
 
struct HorizontalCoordinate{
  /// Local solar azimuth in radians
  double azimuth;
  /// Local solar zenith in radians
  double zenith;
};

class SolarPosition{
 public:
	 //SolarPosition constructor
	 SolarPosition();

  ~SolarPosition(){}

	/**
	* Set the local date and time
	* \param date_time The DateTime data structure
	*/
	void SetDateTime(DateTime date_time);

	/**
	* Set the latitude and longitude of the observer
	* \param lat Latitude in decimal degrees, N = +
	* \param lon Longitude in decimal degrees, W = +
	*/
	void SetLocalLatLong(double lat, double lon);

	/**
	* Get the star brightness in a given direction
	* \param direction The viewing direction in local ENU coordinates
	*/
	float GetStarBrightness(glm::vec3 direction, float pixdim);

  /**
   * Gets the current horizontal coordinates of the sun at the current
	 * date, time, latitude, and longitude.
   */
  HorizontalCoordinate GetSolarPosition();
  
	/**
	* Gets the current horizontal coordinates of the moon 
	* at the current date, time, latitude, and longitude.
	*/
	HorizontalCoordinate GetLunarPosition();

	/**
	* Get the fraction of lunar intensity, based on moon phase
	* 1.0 is full moon, 0.0 is new moon
	*/
	float GetLunarIntensityFraction();

	/// Return the current julian date
	double GetJulianDate() { return jd_; }

 private:


	 /**
	 * Convert date in Gregorian calendar to Julian day.
	 * See https://en.wikipedia.org/wiki/Julian_day
	 * \param year 4 Digit year
	 * \param month Month from 1-12
	 * \param day Day from 1-31
	 * \param hour The current GMT hour, 0-23
	 * \param minute The current minute, 0-59
	 * \param second The current second, 0-59
	 */
	 double JulianDate(double year, double month, double day,
		 double hour, double minute, double second);

  /**
   * See https://en.wikipedia.org/wiki/Position_of_the_Sun
   * \param declination The output solar declination for the
	 current julian data
	 * \param right_acension the output solar right ascension for the current julian data
   */
  void EclipticCoordinates(double &declination, double &right_ascension);

  /**
   * Compute the GMST for the current Julian Date
   */
  double GreenwichMeanSiderealTime();

  /**
   * Convert equatorial coordinates to horizonatal coordinates. 
   * See https://en.wikipedia.org/wiki/Celestial_coordinate_system
   * \param h hour-angle
   * \param delta declination
   */
  HorizontalCoordinate EquatorialToHorizontal(double h, double delta);

	/**
	* Get geocentric lat/long of the moon
	*/
	glm::dvec2 GetMoonGeocentric();

	/// Loads the 9000+ brightest stars
	void LoadStarData();

	/**
	* Convert right angle / declination to zenith/azimuth for a give date, time, and location
	* \param ra Right ascension in degrees
	* \param dec Declination in degrees
	*/
	HorizontalCoordinate RightAscensionDeclinationToAzimuthZenith(double ra, double dec);

	void UpdateStarList();

	/// Update the GMST and LMST
	void UpdateSidereal();

	std::vector <yale_star> stars_;
	double lat_; //local latitude in decimal degrees
	double lon_; //local longitude in decimal degrees
	double lat_rad_; //local latitude in radians
	double lon_rad_; //local longitude in radians
	double gmst_; //gmst in decimial degrees
	double lmst_; //lmst in decimal degrees
	double jd_; //current julian data
	double T_; // terrestrial time

	//star map parameters
	std::vector<std::vector< std::vector<yale_star> > > star_map_;
	double map_res_;
	double map_scale_;
	int n_az_;
	int n_zen_;

	/**
	* Puts a given zenith and azimuth (degrees) into map coordinates
	* \param zenith Zenith in degrees
	* \param azimuth Azimuth in degrees
	*/
	glm::ivec2 ZenAzToMap(double zenith, double azimuth);

	HorizontalCoordinate EclipticToLocal(glm::dvec2 lambda_beta);

	//glm::dmat3 Rx(double ang_radians);
	//glm::dmat3 Ry(double ang_radians);
	//glm::dmat3 Rz(double ang_radians);
	//glm::mat3 GetRotateMoonToLocal();

}; //class Solar Position

} //namespace environment
} //namespace mavs
 
#endif
