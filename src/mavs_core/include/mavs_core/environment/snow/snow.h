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
* \class Snow
*
* The Snow class creates a snow cloud and returns 
* ray intersections with snowflakes. Tracks flake dynamics.
* 
* See:
* Sekhon, R. S., & Srivastava, R. C. (1970). 
* Snow size spectra and radar reflectivity. 
* Journal of the Atmospheric Sciences, 27(2), 299-307.
* 
* and
* 
* Rasshofer, R. H., Spies, M., & Spies, H. (2011). 
* Influences of weather phenomena on automotive laser radar systems. 
* Advances in radio science, 9(B. 2), 49-60.
*
* \author Chris Goodin
*
* \date 6/7/2019
*/
#ifndef SNOW_H
#define SNOW_H

#include <vector>
#include <mavs_core/environment/snow/snowflake.h>

namespace mavs {
namespace environment {

class Snow {
public:
	/**
	* Create a snow system. 
	* It is always centered at (0,0)
	* It "follows" the sensor
	*/
	Snow();

	/**
	* Initialize the snow system
	* \param rate The snowfall rate in mm/h, typically 0-15
	* \param temperature The temperature in Celsius
	*/
	void Initialize(float rate, float temperature);

	/**
	* Update the snow system, moving each flake
	* \param dt The time step in seconds
	* \param wind The wind direction and speed in m/s
	*/
	void Update(float dt, glm::vec3 wind);

	/**
	* Get the closest intersection with a snowflake.
	* Returns an opacity value
	* \param position The global position of the vector origin
	* \param direction The global direction of the vector
	* \param pixdim The size of the pixel on a side (meters)
	* \param snowdist In/out - distance to the closest snowflake
	*/
	float GetClosestIntersection(glm::vec3 position, glm::vec3 direction, float pixdim, float &snowdist);

	/**
	* Return the absorption coefficient of the snowstorm
	* in inverse km. See formula from Rasshofer 2011
	*/
	float GetAlpha() { return alpha_; }

private:

	std::vector<Snowflake> snowflakes_;
	std::vector<std::vector<unsigned int> > sectors_;
	float snow_cylinder_radius_;
	float snow_cylinder_height_;

	float sector_step_;
	int num_sectors_;

	float temperature_;
	float rate_;
	float alpha_;
};

} // namespace environment
} // namespace mavs

#endif