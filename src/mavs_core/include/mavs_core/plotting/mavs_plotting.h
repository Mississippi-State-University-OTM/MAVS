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
 * \class Mplot
 *
 * Class for doing generic plots with CImg
 * Accessible through they python interface to replace
 * pyplot, which doesn't update well in a loop
 * 
 * \author Chris Goodin
 *
 * \date 7/18/2019
 */

#ifndef MAVS_PLOTTING_H
#define MAVS_PLOTTING_H

#include <string>
#include <vector>
#include <CImg.h>

namespace mavs{
namespace utils{

struct Color{
	float r;
	float g;
	float b;
};

class Mplot{
public:
	/// Create a plotter
	Mplot();

	/// Mplot destructor
	~Mplot();

	/// Mplot copy constructor
	Mplot(const Mplot &plot);

	/**
	 * Expand a flattened array into a 3D vector if size width x height X depth
	 * \param width The width of the array
	 * \param height The height of the array
	 * \param depth The depth of the array
	 * \param data Pointer to the float array to expand
	 */
	std::vector<std::vector<std::vector<float> > > UnFlatten3D(int width, int height, int depth, float *data);

	/**
	 * Expand a flattened array into a 2D vector if size width x height
	 * \param width The width of the array
	 * \param height The height of the array
	 * \param data Pointer to the float array to expand
	 */
	std::vector<std::vector<float> > UnFlatten2D(int width, int height, float *data);

	/**
	 * Plot a colorized matrix
	 * \param data The matrix to plot
	 */
	void PlotColorMap(std::vector<std::vector<std::vector<float> > > &data);

	/**
	 * Plot a grayscale matrix
	 * \param data The matrix to plot
	 */
	void PlotGrayMap(std::vector<std::vector<float> > &data);

	/**
	 * Plot a scalar matrix with moreland colormap
	 * \param data The matrix to plot
	 */
	void PlotScalarColorMap(std::vector<std::vector<float> > &data);

	/**
	* Get the moreland color
	* \param x The variable
	* \param xmin The minimum value of the variable
	* \param xmax the maximum possible valuable of the variable
	*/
	Color MorelandColormap(float x, float xmin, float xmax);

	/**
	* Plot a trajectory
	* \param x The x values
	* \param y The y values
	*/
	void PlotTrajectory(std::vector<float> x, std::vector<float> y);

	void TurnOnAxis() { plot_axis_ = true; }

	void TurnOffAxis() { plot_axis_ = false; }

	/**
	* Plot a trajectory
	* \param x_data The x values to fit
	* \param y_data The y values to fit
	* \param y_fit The fit result
	*/
	void PlotCurveFit(std::vector<float> x_data, std::vector<float> y_data, std::vector<float> y_fit);

	/**
	* Add another trajectory to an existing trajector plot
	* \param x The x values
	* \param y The y values
	*/
	void AddToExistingTrajectory(std::vector<float> x, std::vector<float> y);

	/// Add a single point to an existing trajectory
	void AddToExistingTrajectory(float x, float y);

	/// Set the maximum window width (ie time range) for a trajectory plot
	void SetTrajectoryXSize(float x_max_size);

	/**
	* Set the title of the plot to be displayed.
	* \param title The title to be displayed.
	*/
	void SetTitle(std::string title) { title_ = title; }

	/**
	* Save the current plot to a .bmp image file.
	* \param filename The full path to the output file name with .bmp extension.
	*/
	void SaveCurrentPlot(std::string filename);

private:
	cimg_library::CImgDisplay disp_;
	cimg_library::CImg<float> image_;
	bool display_assigned_;
	bool image_assigned_;

	std::string title_;

	float traj_xmin_;
	float traj_ymin_;
	float traj_dp_x_;
	float traj_dp_y_;
	int traj_np_;
	float max_trajectory_width_;
	bool plot_axis_;
};

} //namespace utils
} //namespace mavs

#endif
