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
#include <sensors/object_detector/object_detector.h>

namespace mavs{
namespace sensor {

ObjectDetector::ObjectDetector() {
	SetScanProperties(-180.0f, 180.0f, 0.25f);
	SetMaxRange(30.0f);
	SetName("Object_Detector");
}

void ObjectDetector::Update(environment::Environment *env, double dt) {
	lidar::PlanarLidar::Update(env, dt);
	AnnotateFrame(env, false);
}

std::vector<Obstacle> ObjectDetector::GetObstacles() {
	std::vector<Obstacle> obstacles;
	std::map<int, mavs::sensor::Annotation> anno = GetObjectAnnotations();
	std::map<int, mavs::sensor::Annotation>::iterator it;
	float thresh = 1.0E12f;
	for (it = anno.begin(); it != anno.end(); it++) {
		glm::vec3 ll = it->second.GetLLCorner();
		glm::vec3 ur = it->second.GetURCorner();
		if (ll.x<thresh && ll.x>-thresh && ll.y<thresh && ll.y>-thresh &&
			ur.x<thresh && ur.x>-thresh && ur.y<thresh && ur.y>-thresh) {
			glm::vec3 center = 0.5f*(ll + ur);
			glm::vec3 size = ur - ll;
			Obstacle obs;
			obs.x = center.x;
			obs.y = center.y;
			obs.height = ur.z;
			obs.radius = 0.5f*(float)sqrt(size.x*size.x + size.y*size.y);
			if (obs.radius>0.0f)obstacles.push_back(obs);
		}
	}
	return obstacles;
}

} //namespace sensor
} //namespace mavs
