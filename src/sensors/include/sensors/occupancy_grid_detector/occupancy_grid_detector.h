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
 * \class OccupancyGridDetector
 *
 * Notionalized sensor that returns
 * an occupancy grid
 * 
 * \author Chris Goodin
 *
 * \date 12/10/2018
 */

#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H
#ifdef USE_MPI
#include <mpi.h>
#endif
#include <mavs_core/environment/environment.h>
#include <mavs_core/messages.h>
#include <sensors/sensor.h>
#include <CImg.h>

namespace mavs{
namespace sensor{
namespace ogd{


class OccupancyGridDetector : public Sensor {
 public:
  OccupancyGridDetector();

  ~OccupancyGridDetector();

	OccupancyGridDetector(float x_size, float y_size,
		float resolution_meters);

	///Inherited sensor update method from the sensor base class
	void Update(environment::Environment *env, double dt);

	/**
	* Initialize the scan pattern of the display
	*/
	void Initialize(float x_size, float y_size,
		float resolution_meters);

  /// make a deep copy
  virtual OccupancyGridDetector* clone() const{
    return new OccupancyGridDetector(*this);
  }

	OccupancyGrid GetGrid();

	OccupancyGrid GetBinaryGrid(float thresh);

	void SetMaxHeight(float max_height) {
		max_height_ = max_height;
	}

	void Display();

	void SetNumIgnore(int num_ignore){n_ignore_ = num_ignore;}

 private:
	 float width_;
	 float height_;
	 float resolution_;
	 float max_height_;
	 int nx_;
	 int ny_;
	 int n_ignore_;
	 std::vector< std::vector < int > > grid_;

	 //void FillImage();
	 //glm::vec2 SensorToImageCoords(glm::vec2 sensor_coords);
	 bool first_display_;
	 cimg_library::CImgDisplay disp_;
	 cimg_library::CImg<float> image_;
};

} //namespace ogd
} //namespace sensor
} //namespace mavs

#endif
