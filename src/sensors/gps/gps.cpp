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
#include <sensors/gps/gps.h>

#include <iostream>
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <stdlib.h>

#include <glm/glm.hpp>
#include <rinex.h>

#include <mavs_core/math/matrix.h>
#include <mavs_core/math/utils.h>
#include <mavs_core/math/constants.h>

#include <gps/gps_default.h>

namespace mavs{
namespace sensor{
namespace gps{

static std::ofstream fout_;

Gps::Gps(){
  updated_ = false;
  nsteps_ = 0;
  reciever_position_.x = 0.0;
  reciever_position_.y = 0.0;
  reciever_position_.z = 0.0;
  type_ = "gps";
  prefix_ = "./";
  sat_status_.service = 1;
  sat_status_.status = -1;
  gps_mode_ = 3; //default is perfect
  z200_ = 0.0;
  pressure_kpa_ = 101.0;
  // multipath cutoff is generally difficult to compute
  // however, according to
  // www.navipedia.net/index.php/Multipath
  // it is typically less than 2-3 meters
  multipath_cutoff_dist_ = 3.0;
  ComputeMultipathDirections();
	gps_types_["normal"] = 0;
	gps_types_["dual band"] = 1;
	gps_types_["differential"] = 2;
	gps_types_["perfect"] = 3;
  SetDefaultSatellites();
}

Gps::~Gps(){
  if (fout_.is_open())fout_.close();
}

void Gps::SetDefaultSatellites(){
  for (int i=0;i<34;i++){
    Satellite sat;
    sat.SetPosition(gps_default[i][0],gps_default[i][1],gps_default[i][2]);
    sat.SetVelocity(gps_default[i][3],gps_default[i][4],gps_default[i][5]);
    sat.SetNormal(gps_default[i][6],gps_default[i][7],gps_default[i][8]);
    sat.SetOrbitRadius(gps_default[i][9]);
    sat.SetOrbitVelocity(gps_default[i][10]);
    sat.SetSatelliteTime(gps_default[i][11]);
    satellite_.push_back(sat);
  }
}
  
void Gps::ComputeMultipathDirections(){
  double theta_step = 20.0*kDegToRad;
  double phi_step = theta_step;
  double theta_min = 0.0f;
  double phi_min = 0.0f;
  double theta_max = kPi_2 - theta_step;
  double phi_max = k2Pi - phi_step;
  double theta = theta_min;
  
  while (theta<theta_max){
    double phi = phi_min;
    while (phi<phi_max){
      double ct = cos(theta);
      glm::dvec3 dir(cos(phi)*ct,sin(phi)*ct,sin(theta));
      multipath_check_directions_.push_back(dir);
      phi = phi + phi_step;
    }
    theta = theta + theta_step;
  }
}

void Gps::SetType(std::string gps_type) {
	if (gps_types_.count(gps_type)) {
		gps_mode_ = gps_types_[gps_type];
	}
}

void Gps::SetType(int gps_type){
  if (gps_type == 0) gps_mode_ = 0;
  if (gps_type == 1) gps_mode_ = 1;
  if (gps_type == 2) gps_mode_ = 2;
  if (gps_type == 3) gps_mode_ = 3;
}

void Gps::CalculatePerfect(){
  coordinate_system::ENU enu;
  enu.x = position_.x;
  enu.y = position_.y;
  enu.z = position_.z;
  enu_.x = position_.x;
  enu_.y = position_.y;
  enu_.z = position_.z;
  coordinate_system::ECEF ecef = coord_convert_.ENU2ECEF(enu);
  reciever_position_.x = ecef.x;
  reciever_position_.y = ecef.y;
  reciever_position_.z = ecef.z;
  sat_status_.status = 1;
}

void Gps::Update(environment::Environment *env, double dt){
  local_origin_lla_ = env->GetLocalOrigin();
  coord_convert_.SetLocalOrigin(local_origin_lla_);
  SetRecieverTruePositionENU(position_.x, position_.y, position_.z);

  // simulate the returned gps velocity
  // GPS uses doppler effect to calculate velocity, resulting in error of
  // a few cm/sec
  glm::dvec3 vel((double)velocity_.x,(double)velocity_.y,(double)velocity_.z);
  glm::dvec3 vel_errvec(math::rand_in_range(-.02,.02),math::rand_in_range(-.02,.02),
			math::rand_in_range(-.02,.02));
  vel_enu_ = vel + vel_errvec;

  //GPS Mode 3 is a "perfect" sensor
  if (gps_mode_==3){
    CalculatePerfect();
  }
  else {
    for (int i=0;i<(int)satellite_.size();i++){
      satellite_[i].Update(dt);
    }
    GetSignals(env);
    Trilaterate();
    CalculateRecieverPositionENU();
  }  

  if (log_data_)WriteState();
  local_sim_time_ += local_time_step_;
  nsteps_++;
  updated_ = true;
}

void Gps::WriteState(){
  if (!fout_.is_open()){
    fout_.open(log_file_name_.c_str());
    fout_<<"Time(sec) enu.x enu.y enu.z"<<std::endl;
  }
  glm::dvec3 enu = GetRecieverPositionENU();
  fout_<<local_sim_time_<<" "<<
    enu.x<<" "<<
    enu.y<<" "<<
    enu.z<<" "<<
    std::endl;
}

void Gps::LoadNavFile(std::string filename){
  std::string theObsFileType;
  //int i;
  NGSrinex::RinexFile *myFile = new NGSrinex::RinexFile();
  
  try { myFile->setPathFilenameMode( filename, std::ios::in );  }
  catch( NGSrinex::RinexFileException &openExcep ){
    std::cerr << "Error opening the file: " << filename << std::endl
	      << "Rinex File Exception: " 
              << std::endl << openExcep.getMessage() << std::endl
	      << "Terminating program DRIVER. " << std::endl << std::endl;
  }
  theObsFileType = myFile->getRinexFileType();
  delete myFile;
  myFile = 0;
  
  if( theObsFileType[0] == 'N' ){ // if file is NAV 
    NGSrinex::RinexNavFile mynav;
    NGSrinex::PRNBlock  currentPRNBlock;
    
    try{
      mynav.setPathFilenameMode(filename,std::ios_base::in);
    }
    catch( NGSrinex::RinexFileException &openExcep ){
      std::cerr << "Error opening file: " << filename << std::endl;
      std::cerr << " RinexFileException is: " << std::endl <<
	openExcep.getMessage() << std::endl;
    }
    try{
      mynav.readHeader();
    }
    catch( NGSrinex::RequiredRecordMissingException &headerExcep ){
      std::cerr << " RequiredRecordMissingException is: " << std::endl <<
	headerExcep.getMessage() << std::endl;
    }
    
    //read the PRN Blocks
    try{
      while( mynav.readPRNBlock( currentPRNBlock ) != 0 ){
	//add satellite to the list
	Satellite sat;
	sat.Init(currentPRNBlock);
	satellite_.push_back(sat);
      }
    }
    catch( NGSrinex::RinexReadingException &readingExcep ){
      std::cerr << " RinexReadingException is: " << std::endl <<
	readingExcep.getMessage() << std::endl;
    }
  }
  else{
    std::cerr<<"ERROR: Input file was not a valid NAV file"<<std::endl;
  }
}

double Gps::TraceTo(glm::dvec3 rec_pos, glm::dvec3 sat_pos,
		    environment::Environment *env){
  double dist = -1.0;
  if (sat_pos.z>0.0){
    glm::dvec3 to_sat = sat_pos-rec_pos;
    double dist_to_sat = glm::length(to_sat);
    to_sat = to_sat/(double)dist_to_sat;
    raytracer::Intersection inter = env->GetClosestIntersection(rec_pos,to_sat);
    //check for direct path
    if (inter.dist<0.0 || inter.dist>dist_to_sat){
      dist = dist_to_sat;
    }
    else{ // check for multipath
      double mp_dist = 0.0;
      #pragma omp parallel for schedule(dynamic)
      for (int i=0;i<(int)multipath_check_directions_.size();i++){
	glm::dvec3 dir = multipath_check_directions_[i];
	raytracer::Intersection mp = env->GetClosestIntersection(rec_pos,dir);
	if (mp.dist>0 && mp.dist<=multipath_cutoff_dist_){
	  glm::dvec3 point = rec_pos + 0.999*mp.dist*dir;
	  //see if point is "lit" by satellite
	  glm::dvec3 mp_to_sat = sat_pos-point;
	  double mp_to_sat_dist = glm::length(mp_to_sat);
	  mp_to_sat = mp_to_sat/mp_to_sat_dist;
	  bool blocked = env->GetAnyIntersection(point,mp_to_sat);
	  if (!blocked){
	    if (mp.dist>mp_dist) {
	      dist = mp_to_sat_dist + mp_dist;
	      mp_dist = mp.dist;
	    }
	  }
	}							 
      }
    }
  }
  return dist;
}

glm::dvec3 Gps::GetPosENU(int satNum){
  glm::dvec3 p;
  if (gps_mode_ == 3) { //perfect sensor
    p = position_;
  }
  else {
    glm::dvec3 position = satellite_[satNum].GetPosition();
    coordinate_system::ECEF ecef;
    ecef.x = position.x;
    ecef.y = position.y;
    ecef.z = position.z;
    coordinate_system::ENU enu = coord_convert_.ECEF2ENU(ecef);
    p.x = enu.x;
    p.y = enu.y;
    p.z = enu.z;
  }
  return p;
}

double Gps::ErrorEphem(double r, double theta){
  double Rref = 2.655E7; //meters
  double c = 299792000.0; //m/s
  double dr = sqrt( (1+0.5*sin(theta))*pow(10.0,2.0*Rref/r)+ 
		    (c*c*1.0E-18)+0.0918);
  return dr;
}

double Gps::ErrorIono(double epsilon){
  double se = sin(epsilon);
  double dr = 2.037/( sqrt(se*se+0.076)+se);
  return dr;
}

double Gps::ErrorTropoWet(double epsilon, double theta){
  double zeq = 0.1;
  double a0 = 0.00124;
  double a1 = 4.0E-5;
  double la0 = 2.0;
  double d0 = 7.4E-8;
  double d1 = -1.6E-8;
  double ld0 = 0.0;
  double z0 = 11836.0;
  double z1 = 619.0;
  double lz0 = 3.0;
  double lc0 = 0.0;
  double bm0 = 0.002905;
  double cm0 = 0.0634;
  double cm1 = 0.0014;
  double al   = a0 + a1*cos(2*(theta-la0));
  double dl   = d0 + d1*cos(2*(theta-ld0));
  double zref = z0 + z1*cos(2*(theta-lz0));
  double a = al + dl*(z200_-zref);
  double b = bm0;
  double c = cm0 + cm1*(theta-lc0);
  double se = sin(epsilon);
  double numerator   = 1.0 + a/(1.0+b/(1.0+c));
  double denominator = se + a/(se+b/(se+c));
  double dr = zeq*(numerator/denominator );
  return dr;
}

double Gps::ErrorTropoDry(double theta){
  double dr = 2.2779*pressure_kpa_/
    (1.0-0.00266*cos(2*theta)-0.0028*local_altitude_);
  return dr;
}

double Gps::GetAtmosphericErrors(double r, double epsilon, double theta){
  double de = ErrorEphem(r, theta); 
  double di = ErrorIono(epsilon);
  double dtw = ErrorTropoWet(epsilon,theta);
  double dtd = ErrorTropoDry(theta);
	double dr = de;
	if (gps_mode_ == 1) {
		//dual band, add in ionosphere errors
		dr += di;
	}
	if (gps_mode_ == 0) {
		//normal, add in troposphere errors as well
		dr += (dtw + dtd);
	}
  return dr;
}

void Gps::GetSignals(environment::Environment *env){
  broadcast_positions_.clear();
  broadcast_distances_.clear();
  for (int i=0;i<(int)satellite_.size();i++){
    glm::dvec3 satpos = GetPosENU(i);
    double dist = TraceTo(reciever_true_position_,satpos,env);
    if (dist>0.0){
      broadcast_positions_.push_back(satellite_[i].GetPosition());
      //Add the atmospheric errors to the distance
      glm::dvec3 R = satpos - reciever_true_position_;
      double epsilon = atan(R.z/sqrt(R.x*R.x + R.y*R.y));
      double dR = GetAtmosphericErrors(glm::length(R), epsilon, 
				       local_latitude_);
      dist = dist + dR;
      broadcast_distances_.push_back(dist);
    }
  }
}

void Gps::Trilaterate(){
  sat_status_.status = -1;
  int num_signals = (int)broadcast_distances_.size();
  if (num_signals <4) return;

  mavs::math::Matrix G(num_signals,4);;
  mavs::math::Matrix drho(num_signals,1);
  mavs::math::Matrix delta(4,1); 
  double deltamag = 1.0E6;
  double tolerance = 1.0E-3;
  int loop_counter = 0;
  //estimate current position using the last measured position
  glm::dvec3 pr = reciever_position_; 
  while (deltamag>tolerance && loop_counter<10){
    for (int i=0;i<(int)broadcast_positions_.size();i++){
      glm::dvec3 d = pr-broadcast_positions_[i];
      double dmag = glm::length(d);
      d = d/dmag; 
      G(i,0)=d.x; G(i,1)=d.y; G(i,2)=d.z; G(i,3)=1.0;
      drho(i,0) = broadcast_distances_[i] - dmag;
    }
    mavs::math::Matrix GT = G.Transpose();
    delta = (((GT*G).Inverse())*GT)*drho; 
    pr.x = pr.x + delta(0,0);
    pr.y = pr.y + delta(1,0);
    pr.z = pr.z + delta(2,0);
		dilution_of_precision_.x = delta(0, 0);
		dilution_of_precision_.y = delta(1, 0);
		dilution_of_precision_.z = delta(2, 0);
    deltamag = sqrt(delta(0,0)*delta(0,0) + delta(1,0)*delta(1,0) 
		    + delta(2,0)*delta(2,0));
    loop_counter++;
  }
  if (loop_counter>=10) return;
  sat_status_.status = gps_mode_;
  reciever_position_ = pr;
}

NavSatStatus Gps::GetRosNavSatStatus(){
  return sat_status_;
}

NavSatFix Gps::GetRosNavSatFix(){
  NavSatFix fix;
  fix.status = sat_status_;
  if (sat_status_.status>=0){
    coordinate_system::ECEF ecef;
    ecef.x = reciever_position_.x;
    ecef.y = reciever_position_.y; 
    ecef.z = reciever_position_.z;
    coordinate_system::LLA lla = coord_convert_.ECEF2LLA(ecef);
    fix.latitude = lla.latitude;
    fix.longitude = lla.longitude;
    fix.altitude = lla.altitude;
    fix.position_covariance_type = 0;
  }
  return fix;
}

#ifdef USE_MPI
void Gps::PublishData(int root,MPI_Comm broadcast_to){
  MPI_Bcast(&updated_,1,MPI_LOGICAL,root,broadcast_to);
  if (updated_){
    int myid;
    MPI_Comm_rank(broadcast_to,&myid);
    MPI_Bcast(&reciever_position_,3,MPI_DOUBLE,root,broadcast_to);
    MPI_Bcast(&sat_status_.status,1,MPI_DOUBLE,root,broadcast_to);
    MPI_Bcast(&sat_status_.service,1,MPI_DOUBLE,root,broadcast_to);
    MPI_Bcast(&enu_,3,MPI_DOUBLE,root,broadcast_to);
    MPI_Bcast(&vel_enu_,3,MPI_DOUBLE,root,broadcast_to);
    updated_ = false;
  }
}
#endif
  
void Gps::SetRecieverTruePositionENU(double x, double y, double z){
  reciever_true_position_.x = x;
  reciever_true_position_.y = y;
  reciever_true_position_.z = z;
}

glm::dvec3 Gps::GetRecieverPositionLLA() {
	coordinate_system::ECEF ecef;
	ecef.x = reciever_position_.x;
	ecef.y = reciever_position_.y;
	ecef.z = reciever_position_.z;
	coordinate_system::LLA lla = coord_convert_.ECEF2LLA(ecef);
	glm::dvec3 pos(lla.latitude, lla.longitude, lla.altitude);
	return pos;
}

glm::dvec3 Gps::GetRecieverPositionUTM() {
	coordinate_system::ECEF ecef;
	ecef.x = reciever_position_.x;
	ecef.y = reciever_position_.y;
	ecef.z = reciever_position_.z;
	coordinate_system::UTM utm = coord_convert_.ECEF2UTM(ecef);
	glm::dvec3 pos(utm.x, utm.y, utm.altitude);
	return pos;
}


void Gps::CalculateRecieverPositionENU(){
  coordinate_system::ECEF ecef;
  ecef.x = reciever_position_.x;
  ecef.y = reciever_position_.y;
  ecef.z = reciever_position_.z;
  coordinate_system::ENU enu = coord_convert_.ECEF2ENU(ecef);
  enu_.x = enu.x;
  enu_.y = enu.y;
  enu_.z = enu.z;
}

void Gps::SaveRaw() {
	std::ostringstream ss;
	ss << std::setw(5) << std::setfill('0') << nsteps_;
	std::string num_string(ss.str());

	std::string base_name = prefix_ + type_ + name_ + num_string;

	std::string outname = base_name + ".txt";
	NavSatFix fix = GetRosNavSatFix();

	std::ofstream fout;
	fout.open(outname.c_str());

	fout << "header:" << std::endl;
	fout << "\tseq: " << fix.header.seq << std::endl;
	fout << "\tstamp: " << std::endl;
	fout << "\t\tsecs: " << fix.header.time << std::endl;
	fout << "\tframe_id: " << fix.header.frame_id << std::endl;

	fout << "status: " << std::endl;
	fout << "\tstatus: " << std::to_string(fix.status.status) << std::endl;
	fout << "\tservice: " << fix.status.service << std::endl;

	fout << "latitude " << fix.latitude << std::endl;
	fout << "longitude " << fix.longitude << std::endl;
	fout << "altitude " << fix.altitude << std::endl;

	fout << "position_covariance: [";
	for (double cov : fix.position_covariance) {
		fout << " " << cov;
	}
	fout << "]" << std::endl;
	
	fout << "position_covariance_type: " << std::to_string(fix.position_covariance_type) << std::endl;
	fout.close();
}

} //namespace gps
} //namespace sensor
} //namespace mavs

