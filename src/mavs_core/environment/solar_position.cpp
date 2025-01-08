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
#include <mavs_core/environment/solar_position.h>

#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>

#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>

#include <mavs_core/math/constants.h>
#include <mavs_core/data_path.h>
#include <mavs_core/math/utils.h>
#include <glm/glm.hpp>

namespace mavs{
namespace environment{

static double ClipDegrees(double x){
  while (x > 360.0) x = x - 360.0;
  while (x < 0.0) x = x + 360.0;
  return x;
}

static double ClipHours(double x){
  while (x > 24.0) x = x - 24.0;
  while (x < 0.0) x = x + 24.0;
  return x;
}

SolarPosition::SolarPosition() {
	lat_ = 32.3033;
  	lon_ = 90.8742;
	lat_rad_ = lat_ * mavs::kDegToRad;
	lon_rad_ = lon_ * mavs::kDegToRad;
	map_res_ = 2.5; // in degrees
	n_az_ = (int)(360.0 / map_res_);
	n_zen_ = (int)(90.0 / map_res_);
	map_scale_ = mavs::kRadToDeg / map_res_;
	LoadStarData();
}

void SolarPosition::SetDateTime(DateTime date_time) {
	jd_ = JulianDate((double)date_time.year, (double)date_time.month,
		(double)date_time.day,
		(double)(date_time.hour - date_time.time_zone),
		(double)(60-date_time.minute), (double)date_time.second);
	T_ = (jd_ - 2451545.0) / 36525.0; // Meeus, 1st ed, 11.1
	UpdateSidereal();
	UpdateStarList();
}

void SolarPosition::SetLocalLatLong(double lat, double lon) {
	lat_ = lat;
	lon_ = lon;
	lat_rad_ = lat * mavs::kDegToRad;
	lon_rad_ = lon * mavs::kDegToRad;
	UpdateSidereal();
	UpdateStarList();
}

void SolarPosition::UpdateSidereal() {
	gmst_ = GreenwichMeanSiderealTime();
	lmst_ = 15*gmst_ - lon_;
}

double SolarPosition::JulianDate(double year, double month, double day,
				double hour, double minute, double second){
  int y = (int)year;
  int m = (int)month;
  int d = (int)day;
  y+=8000;
  if(m<3) { y--; m+=12; }
  int julian_daynum = (y*365) +(y/4) -(y/100) +(y/400) -1200820 +
    (m*153+3)/5-92 + d-1;
  double time_offset =  (12.0-hour)/24.0 + minute/1440.0 + second/86400.0;
  double julian_date = ((double)julian_daynum) + time_offset;
  return julian_date;
}

void SolarPosition::EclipticCoordinates( double &declination,
			 double &right_ascension){
  double n = jd_ - 2451545.0;
  //mean longitude
  double L = ClipDegrees(280.460 + 0.9856474*n);
  //mean anomaly
  double g = ClipDegrees(357.528 + 0.9856003*n);
  double g_rad = kDegToRad*g;
  double two_g_rad = 2*g_rad;
  double ecliptic_longitude = L + 1.915*sin(g_rad)+0.020*sin(two_g_rad);
  double long_rad = kDegToRad*ecliptic_longitude;
  double obliquity = ClipDegrees(23.439-0.0000004*n);
  double obliq_rad = kDegToRad*obliquity;

  //calculate to equatorial coordinates;
  right_ascension = atan2(cos(obliq_rad)*sin(long_rad),cos(long_rad));
  declination = asin(sin(obliq_rad)*sin(long_rad));
}

double SolarPosition::GreenwichMeanSiderealTime(){
  double d = jd_ - 2451545.0;
	//aa.usno.navy.mil/faq/docs/GAST.php
  gmst_ = 18.697374558 + 24.06570982441908*d;
	// Meeus 1st edition, 11.4
	//gmst_ = 280.46061837 + 360.98564736629*d + 0.000387933*T_*T_ - T_ * T_*T_ / 38710000.0;
	gmst_ = ClipHours(gmst_);
  return gmst_;
}

HorizontalCoordinate SolarPosition::EquatorialToHorizontal(double h,double delta){
  double x = -sin(lat_rad_)*cos(delta)*cos(h) + cos(lat_rad_)*sin(delta);
  double y = cos(delta)*sin(h);
  HorizontalCoordinate solpos;
  solpos.azimuth = -atan2(y,x);
  solpos.zenith = kDegToRad*90 -
    asin(sin(lat_rad_)*sin(delta)+cos(lat_rad_)*cos(delta)*cos(h));
  return solpos;
}

HorizontalCoordinate SolarPosition::GetSolarPosition(){
  double declination, right_ascension;
  EclipticCoordinates(declination,right_ascension);
  double hour_angle = (15*gmst_-lon_)*kDegToRad - right_ascension;
  HorizontalCoordinate solpos = EquatorialToHorizontal(hour_angle, declination);
  return solpos;
}

// See "A Physically-Based Night Sky Model", (1999), appendix
glm::dvec2 SolarPosition::GetMoonGeocentric() { 
	//double l_prime = 3.8104 + 8399.7091*T_;
	double l_prime = 3.8104 + 9399.7091*T_;
	double m_prime = 2.3554 + 8328.6911*T_;
	double m = 6.23 + 628.3019*T_;
	double d = 5.1985 + 7771.3772*T_;
	double f = 1.6280 + 8433.4633*T_;
	
	glm::dvec2 lambda_beta;
	//longitude
	lambda_beta.x = l_prime 
		+ 0.1098*sin(m_prime)
		+ 0.0222*sin(2 * d - m_prime)
		+ 0.0115*sin(2 * d)
		+ 0.0037*sin(2 * m_prime)
		- 0.0032*sin(m)
		- 0.0020*sin(2 * f)
		+ 0.0010*sin(2 * d - 2 * m_prime)
		+ 0.0010*sin(2 * d - m - m_prime)
		+ 0.0009*sin(2 * d + m_prime)
		+ 0.0008*sin(2 * d - m)
		+ 0.0007 * sin(m_prime - m)
		- 0.0006*sin(d)
		- 0.0005*sin(m + m_prime);
	//latitude
	lambda_beta.y = 0.0895*sin(f)
		+ 0.0049*sin(m_prime + f)
		+ 0.0048*sin(m_prime - f)
		+ 0.0030*sin(2 * d - f)
		+ 0.0010*sin(2 * d + f - m_prime)
		+ 0.0008*sin(2 * d - f - m_prime)
		+ 0.0006*sin(2 * d + f);
	return lambda_beta;
}


HorizontalCoordinate SolarPosition::EclipticToLocal(glm::dvec2 lambda_beta) {
	// http://jgiesen.de/elevazmoon/basics/meeus.htm#horizon
	double L = lambda_beta.x;
	double B = lambda_beta.y;
	double eps = 23.0 + 26.0 / 60.0 + 21.448 / 3600.0 - (46.8150*T_ + 0.00059*T_*T_ - 0.001813*T_*T_*T_) / 3600;
	double X = cos(B)*cos(L);
	double Y = cos(eps)*cos(B)*sin(L) - sin(eps)*sin(B);
	double Z = sin(eps)*cos(B)*sin(L) + cos(eps)*sin(B);
	double R = sqrt(1 - Z * Z);
	double delta = asin(sin(eps)*cos(B)*sin(L) + cos(eps)*sin(B)); // in radians
	double at = atan2((sin(L)*cos(eps) - tan(B)*sin(eps)) , cos(L));
	if (at < 0) at = at + mavs::kPi;
	double RA = at; // in radians
	double tau = mavs::kDegToRad*lmst_ - RA;
	double alt = asin(sin(B)*sin(delta) + cos(B)*cos(delta)*cos(tau));
	double az = atan2(-sin(tau) , (cos(B)*tan(delta) - sin(B)*cos(tau)));
	HorizontalCoordinate za;
	za.zenith = mavs::kPi_2 - alt;
	za.azimuth = az;
	return za;
}

// See "A Physically-Based Night Sky Model", (1999), appendix
HorizontalCoordinate SolarPosition::GetLunarPosition() {
	glm::dvec2 lambda_beta =  GetMoonGeocentric();
	HorizontalCoordinate lunpos = EclipticToLocal(lambda_beta);
	return lunpos;
}

HorizontalCoordinate SolarPosition::RightAscensionDeclinationToAzimuthZenith(double ra, double dec) {
	HorizontalCoordinate hc;
	double H = mavs::kDegToRad*(lmst_ - ra);
	double delta = mavs::kDegToRad*dec;

	// formulas from http://stjarnhimlen.se/comp/tutorial.html#6
	// check against http://www.stargazing.net/mas/al_az.htm
	double x = cos(H)*cos(delta);
	double y = sin(H)*cos(delta);
	double z = sin(delta);
	double xhor = x * sin(lat_rad_) - z * cos(lat_rad_);
	double yhor = y;
	double zhor = x * cos(lat_rad_) + z * sin(lat_rad_);
	hc.azimuth = atan2(yhor, xhor) + mavs::kPi;
	double alt = asin(zhor);
	hc.zenith = mavs::kPi_2 - alt;

	/*//Meeus 1st edition, 12.5
	double elev = asin(sin(lat_rad_)*sin(delta) - cos(lat_rad_)*cos(delta)*cos(H));
	hc.zenith = mavs::kPi_2 - elev;
	//Meeus 1st editsion, 12.6
	hc.azimuth = atan2(sin(H), (cos(H)*sin(lat_rad_)-tan(delta)*cos(lat_rad_))) + mavs::kPi;
	*/
	return hc;
}

void SolarPosition::LoadStarData() {
	mavs::MavsDataPath mavs_path;
	std::string input_file = mavs_path.GetPath();
	input_file.append("/environments/stardata/bsc5-short.json");
	FILE* fp = fopen(input_file.c_str(), "rb");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	int nstars = (int)d.Size();
	stars_.resize(nstars);
	for (rapidjson::SizeType i = 0; i < d.Size(); i++) {
		if (d[i].HasMember("K")) {
			std::string ctemp = d[i]["K"].GetString();
			stars_[i].color_temperature = atof(ctemp.c_str());
		}
		if (d[i].HasMember("Dec")) { //declination
			std::string dms = d[i]["Dec"].GetString();
			double degrees, minutes, seconds;
			sscanf(dms.c_str(), "%lf %lf %lf", &degrees, &minutes, &seconds);
			stars_[i].declination = degrees + (minutes / 60.0) + (seconds / 3600.0);
		}
		
		if (d[i].HasMember("RA")) { //right ascension
			std::string hms = d[i]["RA"].GetString();
			double hours, minutes, seconds;
			sscanf(hms.c_str(), "%lf %lf %lf", &hours, &minutes, &seconds);
			stars_[i].right_ascension = 15.0*(hours + (minutes / 60.0) + (seconds / 3600.0));
		}
		if (d[i].HasMember("V")) {
			std::string vms = d[i]["V"].GetString();
			double vm = atof(vms.c_str());
			stars_[i].brightness = pow(10.0, 0.4*(vm - 19.0));
		}
	}
}

glm::ivec2 SolarPosition::ZenAzToMap(double zenith, double azimuth) {
	glm::ivec2 n;
	n.y = (int)(map_scale_*zenith);
	n.x = (int)(map_scale_*azimuth);
	return n;
}

void SolarPosition::UpdateStarList() {	
	star_map_.clear();
	std::vector <yale_star> empty_star_list;
	star_map_ = mavs::utils::Allocate2DVector(n_az_, n_zen_, empty_star_list);
	for (int i = 0; i < (int)stars_.size(); i++) {
		HorizontalCoordinate za = RightAscensionDeclinationToAzimuthZenith(stars_[i].right_ascension, stars_[i].declination);
		double elev = mavs::kPi_2 - za.zenith;
		if (elev > 0.0) {
			glm::ivec2 c = ZenAzToMap(za.zenith, za.azimuth);
			if (c.x >= 0 && c.y >= 0 && c.x<n_az_ && c.y<n_zen_) {
				stars_[i].direction.x = (float)(cos(za.azimuth)*sin(za.zenith));
				stars_[i].direction.y = (float)(sin(za.azimuth)*sin(za.zenith));
				stars_[i].direction.z = (float)cos(za.zenith);
				star_map_[c.x][c.y].push_back(stars_[i]);
			}
		}
	}
}

float SolarPosition::GetStarBrightness(glm::vec3 direction, float pixdim) {
	double l = 0.0f;
	double z = acos(direction.z);
	double a = atan2(direction.y,direction.x);
	if (a < 0.0)a = mavs::k2Pi + a;
	glm::ivec2 c = ZenAzToMap(z, a);
	//Betelgeuse, the typical star occcupies about 2E-4 solid angle in sky
	double exponent = -5000.0 / pixdim;
	for (int ci = c.x - 1; ci <= c.x + 1; ci++) {
		int i = ci;
		if (ci == n_az_)i = 0;
		if (ci < 0)i = n_az_ + ci;
		for (int cj = c.y - 1; cj <= c.y + 1; cj++) {
			int j = cj;
			if (cj == n_zen_)j = 0;
			if (cj < 0)j = n_zen_ + cj;
			if (i >= 0 && i < n_az_ && j >= 0 && j < n_zen_) {
				for (int k = 0; k < (int)star_map_[i][j].size(); k++) {
					float theta = (float)acos(glm::dot(direction, star_map_[i][j][k].direction));
					float dot = 0.0001f*(float)exp(exponent * theta*theta);
					l += (float)sqrt(star_map_[i][j][k].brightness)*dot;
				}
			}
		}
	}
	return (float)l;
}

float SolarPosition::GetLunarIntensityFraction() {
	float days_since = (float)jd_ - 2451549.5f;
	float new_moons = days_since / 29.53f;
	float phase = new_moons - floor(new_moons);
	float intens = (float)sin(mavs::kPi*phase);
	return intens;
}

/*
HorizontalCoordinate SolarPosition::GetLunarPosition() {
glm::dvec2 lambda_beta = GetMoonGeocentric();
HorizontalCoordinate lunpos;
lunpos.azimuth = mavs::kDegToRad*ClipDegrees(mavs::kRadToDeg*lambda_beta.x);
lunpos.zenith = mavs::kPi_2-lambda_beta.x;
return lunpos;
}
*/
/*
//https://www.aa.quae.nl/en/reken/hemelpositie.html#4,
HorizontalCoordinate SolarPosition::GetLunarPosition() {
//https://www.aa.quae.nl/en/reken/hemelpositie.html#4, equations 71-74
double L = mavs::kDegToRad*(218.316 + 13.176396*T_); //geocentric ecliptic longitude, radians
double M = mavs::kDegToRad*(134.963 + 13.064993*T_); // mean anomaly, radians
double F = mavs::kDegToRad*(93.272 + 13.229350*T_); // distance, radians
double lambda = mavs::kDegToRad*(22.44 + 6.289*sin(M)); //geocentric ecliptic longitude, radians
double beta = mavs::kDegToRad*(5.128*sin(F)); //geocentric ecliptic latitude, radians
//https://www.aa.quae.nl/en/reken/hemelpositie.html#4, equations 21-24
double epsilon = 23.4397; //degrees
double delta = asin(sin(beta)*cos(epsilon) + cos(beta)*sin(epsilon)*sin(lambda)); //declination, radians
double alpha = atan2(sin(lambda)*cos(epsilon) - tan(beta)*sin(epsilon), cos(lambda)); //right ascension, radians
//https://www.aa.quae.nl/en/reken/hemelpositie.html#4, equation 31
double H = mavs::kDegToRad*lmst_ - alpha; // hour angle , radians
//https://www.aa.quae.nl/en/reken/hemelpositie.html#4, equations 35-36
double A = atan2(sin(H), cos(H)*sin(lat_rad_) - tan(delta)*cos(lat_rad_)); //azimuth, radians
double h = asin(sin(lat_rad_)*sin(delta) + cos(lat_rad_)*cos(delta)*cos(H)); //altitude, radians
HorizontalCoordinate lunpos;
lunpos.azimuth = A;
lunpos.zenith = mavs::kPi_2 - h;
return lunpos;
}
*/
/*
glm::dmat3 SolarPosition::Rx(double t) {
	glm::dmat3 R;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			R[i][j] = 0.0;
		}
	}
	double c = cos(t);
	double s = sin(t);
	R[0][0] = 1.0;
	R[1][1] = c;
	R[1][2] = -s;
	R[2][1] = s;
	R[2][2] = c;
	return R;
}

glm::dmat3 SolarPosition::Ry(double t) {
	glm::dmat3 R;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			R[i][j] = 0.0;
		}
	}
	double c = cos(t);
	double s = sin(t);
	R[1][1] = 1.0;
	R[0][0] = c;
	R[0][2] = s;
	R[2][0] = -s;
	R[2][2] = c;
	return R;
}

glm::dmat3 SolarPosition::Rz(double t) {
	glm::dmat3 R;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			R[i][j] = 0.0;
		}
	}
	double c = cos(t);
	double s = sin(t);
	R[2][2] = 1.0;
	R[0][0] = c;
	R[0][1] = -s;
	R[1][0] = s;
	R[1][1] = c;
	return R;
}

// See "A Physically-Based Night Sky Model", (1999), appendix
glm::mat3 SolarPosition::GetRotateMoonToLocal() {
	glm::dmat3 R = Ry(lat_rad_ - mavs::kPi_2)*Rz(-lmst_)*Rz(0.01118*T_)*Ry(-0.00972*T_)*Rz(0.001118*T_);
	return R;
}
*/
} // namespace environment
} // namespace mavs
