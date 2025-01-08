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
#include <sensors/gps/satellite.h>
#include <math.h>
#include <glm/glm.hpp>

namespace mavs{
namespace sensor{
namespace gps{

const double kGravitationalConstant = 6.67E-11;
const double kEarthMass = 5.972E24;
  
Satellite::Satellite(){}

void Satellite::Update(double dt){
  double s = dt*orbit_velocity_;
  double theta = s/orbit_radius_;
  //use rodrigues formula to rotate to the new position
  double ctheta = cos(theta);
  glm::dvec3 old_pos = position_;
  position_ = position_*ctheta + 
    (glm::cross(orbit_normal_,position_)*sin(theta)) + 
    position_*dot(orbit_normal_,position_)*(1-ctheta);
  velocity_ = (position_-old_pos)/dt;
  satellite_time_ = satellite_time_ + dt;
}

void Satellite::Init(NGSrinex::PRNBlock &prb){
  // Code modified from:  
  // https://www.ngs.noaa.gov/gps-toolbox/bc_velo.htm
  double bPI = 3.1415926535898;
  double bGM84 = 3.986005e14;
  double bOMEGAE84 = 7.2921151467e-5;
  double roota = prb.getSqrtA();
  double toe = prb.getToe();
  double m0 = prb.getMo();
  double e = prb.getEccen();
  double delta_n = prb.getDeltan(); 
  double smallomega = prb.getLilOmega();
  double cus = prb.getCus(); 
  double cuc = prb.getCuc();  
  double crs = prb.getCrs();
  double crc = prb.getCrc();
  double cis = prb.getCis();
  double cic = prb.getCic();
  double idot = prb.getIdot();  
  double i0 =  prb.getIo();
  double bigomega0 = prb.getBigOmega();
  double earthrate =  bOMEGAE84;
  double bigomegadot = prb.getBigOmegaDot(); 
  satellite_time_ = prb.getTransmTime(); 
  double t = satellite_time_;
  double A;
  double n0, n;
  double tk;
  double mk, ek, tak, ik, omegak, phik, uk, rk; //vk,
  double corr_u, corr_r, corr_i;
  double xpk, ypk;
  double xk, yk, zk;
  double mkdot, ekdot, takdot, ukdot, ikdot, rkdot, omegakdot;
  double xpkdot, ypkdot;
  double xkdot, ykdot, zkdot;
  A = roota*roota;           //roota is the square root of A
  n0 = sqrt(bGM84/(A*A*A));  //bGM84 is what the ICD-200 calls Greek mu
  tk = t - toe;              //t is the time of the pos. & vel. request.
  n = n0 + delta_n;
  mk = m0 + n*tk;
  mkdot = n;
  ek = mk;
  //Overkill for small e
  for(int iter=0; iter<7; iter++) ek = mk + e*sin(ek); 
  ekdot = mkdot/(1.0 - e*cos(ek));
  //In the line, below, tak is the true anomaly (which is nu in the ICD-200).
  tak = atan2( sqrt(1.0-e*e)*sin(ek), cos(ek)-e);
  takdot = sin(ek)*ekdot*(1.0+e*cos(tak))/(sin(tak)*(1.0-e*cos(ek)));
  
  phik = tak + smallomega;
  corr_u = cus*sin(2.0*phik) + cuc*cos(2.0*phik);
  corr_r = crs*sin(2.0*phik) + crc*cos(2.0*phik);
  corr_i = cis*sin(2.0*phik) + cic*cos(2.0*phik);
  uk = phik + corr_u;
  rk = A*(1.0-e*cos(ek)) + corr_r;
  ik = i0 + idot*tk + corr_i;
  
  ukdot = takdot +2.0*(cus*cos(2.0*uk)-cuc*sin(2.0*uk))*takdot;
  rkdot = A*e*sin(ek)*n/(1.0-e*cos(ek)) + 
    2.0*(crs*cos(2.0*uk)-crc*sin(2.0*uk))*takdot;
  ikdot = idot + (cis*cos(2.0*uk)-cic*sin(2.0*uk))*2.0*takdot;
  
  xpk = rk*cos(uk);
  ypk = rk*sin(uk);
  
  xpkdot = rkdot*cos(uk) - ypk*ukdot;
  ypkdot = rkdot*sin(uk) + xpk*ukdot;
  
  omegak = bigomega0 + (bigomegadot-earthrate)*tk - earthrate*toe;
  
  omegakdot = (bigomegadot-earthrate);
  
  xk = xpk*cos(omegak) - ypk*sin(omegak)*cos(ik);
  yk = xpk*sin(omegak) + ypk*cos(omegak)*cos(ik);
  zk =                   ypk*sin(ik);
  
  xkdot = ( xpkdot-ypk*cos(ik)*omegakdot )*cos(omegak)
    - ( xpk*omegakdot+ypkdot*cos(ik)-ypk*sin(ik)*ikdot )*sin(omegak);
  ykdot = ( xpkdot-ypk*cos(ik)*omegakdot )*sin(omegak)
    + ( xpk*omegakdot+ypkdot*cos(ik)-ypk*sin(ik)*ikdot )*cos(omegak);
  zkdot = ypkdot*sin(ik) + ypk*cos(ik)*ikdot;
  
  position_.x = xk;
  position_.y = yk;
  position_.z = zk;
  velocity_.x = xkdot;
  velocity_.y = ykdot;
  velocity_.z = zkdot;
  orbit_normal_ = glm::normalize(glm::cross(position_, velocity_));
  orbit_radius_ = glm::length(position_);
  orbit_velocity_ = sqrt(kGravitationalConstant*kEarthMass/orbit_radius_); 
}

void Satellite::PrintState(){
  std::cout.precision(10);
  std::cout<<"{";
  std::cout<<position_.x<<", "<<position_.y<<", "<<position_.z<<", ";
  std::cout<<velocity_.x<<", "<<velocity_.y<<", "<<velocity_.z<<", ";
  std::cout<<orbit_normal_.x<<", "<<orbit_normal_.y<<", "<<
    orbit_normal_.z<<", ";
  std::cout<<orbit_radius_<<", "<<orbit_velocity_<<", "<<satellite_time_;
  std::cout<<"},"<<std::endl;
}
}//namespace gps
}//namespace sensor
}//namespace mavs
