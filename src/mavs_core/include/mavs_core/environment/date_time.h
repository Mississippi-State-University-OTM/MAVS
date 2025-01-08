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
#ifndef DATE_TIME_H
#define DATE_TIME_H

namespace mavs {
namespace environment {

/**
* Data structure holding date and time information
*/
struct DateTime {
	DateTime() {
		year = 2004;
		month = 6;
		day = 5;
		hour = 14;
		minute = 0;
		second = 0;
		millisecond = 0;
		time_zone = 6;
	}
	/// 4 digit year
	int year;
	/// Month from 1 to 12
	int month;
	/// Day from 1 to 31
	int day;
	/// Hour from 0 to 23
	int hour;
	/// Minute from 0 to 59
	int minute;
	/// Second from 0 to 59
	int second;
	/// millisecond from 0 to 999
	int millisecond;
	/// Time zone, CST=6
	int time_zone;
};

} //namespace environment
} //namespace mavs


#endif
