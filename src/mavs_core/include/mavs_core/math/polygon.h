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
 * \class Polygon
 * 
 * A 2D polygon class that with method to check if a point lies inside.
 * See algorithm listed on 
 * https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-
 * inside-a-polygon/
 *
 * \author Chris Goodin
 *
 * \date 1/11/2018
 */
#ifndef POLYGON_H
#define POLYGON_H

#include <vector>

#include <glm/glm.hpp>

namespace mavs{
namespace math{

class Polygon{
 public:
  ///Construct an empty polygon
  Polygon();

  /**
   * Construct a polygon with a list of points. The points should be in either
   * clockwise or counterclockwise order around the polygon. 
   * \param points Ordered list of points defining the polygon.
   */
  Polygon(std::vector<glm::vec2> points);

  ///Destructor
  ~Polygon();

  /**
   * Checks if a point is inside the polygon.
   * \param point Point to be checked
   */
  bool IsInside(glm::vec2 point);

  ///Returns a point at a random location inside the polygon
  glm::vec2 GetRandomInside();

	/// Return the number of points in the polygon
	int NumPoints() { return (int)polygon_.size(); }

	/// Return the coordinates of the i'th point
	glm::vec2 GetPoint(int i) { return polygon_[i]; }

	/// Return the area of the polygon
	float GetArea();

 private:
  std::vector<glm::vec2> polygon_;

  bool OnSegment(glm::vec2 p, glm::vec2 q, glm::vec2 r);

  bool SegmentIntersect(glm::vec2 p1, glm::vec2 q1, 
			glm::vec2 p2, glm::vec2 q2);

  int Orientation(glm::vec2 p, glm::vec2 q, glm::vec2 r);

  float llx_,lly_,urx_,ury_;
};

} //namespace math
} //namespace mavs

#endif
