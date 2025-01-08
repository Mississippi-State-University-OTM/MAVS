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
 * \file segment.h
 * 
 * Methods for performing intersections and
 * operations on line segments
 *
 * \author Chris Goodin
 *
 * \date 3/5/2019
 */
#ifndef SEGMENT_H
#define SEGMENT_H
#include <glm/glm.hpp>

namespace mavs{
namespace math{

/**
* Returns if two lines intersect
* \param ps1 First endpoint of first line
* \param pe1 Second endpoint of first line
* \param ps2 First endpoint of second line
* \param pe2 Second endpoint of second line
*/
bool SegSegIntersect(glm::vec2 ps1, glm::vec2 pe1, glm::vec2 ps2, glm::vec2 pe2, glm::vec2 &inter_point);

/**
* Return if a segment intersects a rectangle
* \param p1 First endpoint of the segment
* \param p2 Second endpoint of the segment
* \param r1 First corner of the rectangle
* \param r2 Second corner of the rectangle
* \param r3 Third corner of the rectangle
* \param r4 Fourth corner of the rectangle
*/
bool SegBoxIntersect(glm::vec2 p1, glm::vec2 p2, glm::vec2 r1, glm::vec2 r2, glm::vec2 r3, glm::vec2 r4, glm::vec2 &inter_point);

/**
* Check intersection of segment with axis aligned bounding box
* \param p1 First point of segment
* \param p2 Second point of segment
* \param ll Lower left corner of bounding box
* \param ur Upper right corner of bounding box
*/
bool SegAABBIntersect(glm::vec2 p1, glm::vec2 p2, glm::vec2 ll, glm::vec2 ur);

/**
* Check the intersection of a ray with a segment in 2D. 
* Returns the distance along the ray to the intersection.
* Returns -1.0 if there is no intersection
* \param origin Origin of the ray
* \param direction Direction of the ray
* \param p1 First endpoint of the segment
* \param p2 Second endpoint of the segment
* 
*/
float RaySegmentIntersection(glm::vec2 origin, glm::vec2 direction, glm::vec2 p1, glm::vec2 p2);

/**
* Return distance from a point to a line in 2D
* \param x1 First point on the line
* \param x2 Second point on the line 
* \param x0 The test point
*/
float PointLineDistance(glm::vec2 x1, glm::vec2 x2, glm::vec2 x0);

/**
* Return distance from a point to a segment
* \param ep1 First endpoint of the segment
* \param ep2 Second endpoint of the segment
* \param p The test point
*/
float PointToSegmentDistance(glm::vec2 ep1, glm::vec2 ep2, glm::vec2 p);

} //namespace math
} //namespace mavs

#endif
