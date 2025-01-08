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
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

namespace mavs {
namespace vehicle {

class PidController{
 public:
  PidController();

  double GetControlVariable(double measured_value, double dt);

  void SetSetpoint(double setpoint){setpoint_ = setpoint;}

  void SetKp(double kp){kp_=kp;}

  void SetKi(double ki){ki_ = ki;}

  void SetKd(double kd){kd_ = kd;}
  
 private:
  double kp_;
  double ki_;
  double kd_;
  double setpoint_;
  double previous_error_;
  double integral_;
};

} //namespace vehicle
} //namespace mavs
#endif
