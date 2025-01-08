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
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <time.h>
#include <CImg.h>
#ifdef None
#undef None
#endif
#include <glm/glm.hpp>
#include <mavs_core/math/utils.h>
#include "sensors/camera/add_rain_to_existing_image.h"

int main(int argc, char * argv[]) {

	std::string imagefile(argv[1]);
	float rate = (float)atof(argv[2]);
	cimg_library::CImg<float> image;
	image.load(imagefile.c_str());

	mavs::sensor::camera::AddRainToImage(image, rate, true);

	//image.display();
	std::string outfile = "raining_";
	outfile.append(imagefile);
	image.save(outfile.c_str());

	return 0;
}