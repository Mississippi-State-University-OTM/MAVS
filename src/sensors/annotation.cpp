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
#include <sensors/annotation.h>

namespace mavs {
namespace sensor {

Annotation::Annotation(){
	class_number_ = -1;
}

Annotation::Annotation(std::string n, int min_x, int min_y, int max_x, int max_y) {
	name_ = n;
	pix_ll_.x = min_x;
	pix_ll_.y = min_y;
	pix_ur_.x = max_x;
	pix_ur_.y = max_y;
}

Annotation::Annotation(std::string n, glm::vec3 p, int num) {
	name_ = n;
	ll_ = p;
	ur_ = p;
	class_number_ = num;
}

Annotation::Annotation(std::string name, int r, int g, int b) {
	name_ = name;
	color_ = glm::ivec3(r, g, b);
}

} //namespace sensor
} //namespace mavs