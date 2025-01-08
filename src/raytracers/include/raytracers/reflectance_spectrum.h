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
* \class ReflectanceSpectrum
*
* Spectral reflectance class for MAVS
* Wavelengths are in microns (10-6 meters)
* Reflectance values are [0,1]
*
* \author Chris Goodin
*
* \date 2/12/2019
*/
#ifndef REFLECTANCE_SPECTRUM_H
#define REFLECTANCE_SPECTRUM_H

#include <vector>
#include <string>
#include <unordered_map>

namespace mavs{

class ReflectanceSpectrum{
public:
	/// Create an empty spectrum
	ReflectanceSpectrum();

	/**
	* Create a reflectance spectrum with the specified input file
	* \param infile The spectrum file to load
	*/
	ReflectanceSpectrum(std::string infile) {
		Load(infile);
	}

	/**
	* Load a mavs spectrum file. 
	* Format is two column space delimited. 
	* First column is the wavelength in micrometers
	* Second column is the reflecatance at each wavelength
	* Should have a .spec file extension
	* Pairs should be ordered in increasing wavelength
	* \param infile The spectrum file to load
	*/
	void Load(std::string infile);

	/**
	* Get the reflectance at a given wavelength
	* \param wl The wavelength in micrometers
	*/
	float GetReflectanceAtWavelength(float wl);

	/**
	* Return the name of the spectrum as a string
	*/
	std::string GetName() { return name_; }

private:
	std::vector<float> wavelength_;
	std::vector<float> reflectance_;
	std::unordered_map<float,float> repeat_vals_;
	std::string name_;
};

} // namespace mavs

#endif
