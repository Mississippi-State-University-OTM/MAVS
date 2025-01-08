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
#ifndef ANNOTATION_COLORS_H
#define ANNOTATION_COLORS_H
#include <vector>
#include <glm/glm.hpp>

namespace mavs {
namespace sensor {

class AnnotationColors {
public:
	AnnotationColors();

	glm::vec3 GetColor(int i);

	void SetColor(int i, float r, float g, float b);

	void AddColor(float r, float g, float b);

	size_t GetNumColors() {
		return annotation_colors_.size();
	}

private:
	std::vector<glm::vec3> annotation_colors_; 
};

} // namespace sensors
} // namespace mavs

#endif