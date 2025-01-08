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
/**
* \file date_time_example.cpp
*
* An example of advancing the time in MAVS environment
*
* \author Chris Goodin
*
* \date 1/6/2024
*/
#include <mavs_core/environment/environment.h>

#include <iostream>


int main (int argc, char *argv[]){
	mavs::environment::Environment env;
	env.SetDateTime(2000, 1, 1, 0, 0, 0, 6);

	int sec_per_leap_year = 31622400;
	int sec_per_year = 31536000;
	int sec_per_week = 24 * 7 * 60 * 60;

	// advance time one year
	env.AdvanceTime((float)sec_per_leap_year);
	mavs::environment::DateTime date = env.GetDateTime();
	std::cout << "DATE: "<<date.month << "/" << date.day << "/" << date.year << ", TIME:" << date.hour << ":" << date.minute << ":" << date.second << ":" << date.millisecond << std::endl;

	// advance time 1:30.1
	env.AdvanceTime(90.1f);
	date = env.GetDateTime();
	std::cout << "DATE: " << date.month << "/" << date.day << "/" << date.year << ", TIME: " << date.hour << ":" << date.minute << ":" << date.second << ":" << date.millisecond << std::endl;

	// advance time 1.2 second
	env.AdvanceTime(1.2f);
	date = env.GetDateTime();
	std::cout << "DATE: " << date.month << "/" << date.day << "/" << date.year << ", TIME: " << date.hour << ":" << date.minute << ":" << date.second << ":" << date.millisecond << std::endl;

	// advance time 1 week
	env.AdvanceTime((float)sec_per_week);
	date = env.GetDateTime();
	std::cout << "DATE: " << date.month << "/" << date.day << "/" << date.year << ", TIME: " << date.hour << ":" << date.minute << ":" << date.second << ":" << date.millisecond << std::endl;

	// advance 4 more weeks
	env.AdvanceTime(4.0f*sec_per_week);
	date = env.GetDateTime();
	std::cout << "DATE: " << date.month << "/" << date.day << "/" << date.year << ", TIME: " << date.hour << ":" << date.minute << ":" << date.second << ":" << date.millisecond << std::endl;

  return 0;
}

