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
#include <raytracers/texture_layers/texture_layer.h>

namespace mavs {
namespace raytracer {

TextureLayer::TextureLayer() {

}
//See: 
//https://squircleart.github.io/shading/normal-map-generation.html
void TextureLayer::CreateNormalMap() {
	// create a grayscale version of the image
	cimg_library::CImg<float> grayscale;
	grayscale.assign(image_.width(), image_.height(), 1, 1);
	for (int i = 0; i < image_.width(); i++) {
		for (int j = 0; j < image_.height(); j++) {
			float gray = (image_(i, j, 0, 0) + image_(i, j, 0, 1) + image_(i, j, 0, 2)) / 255.0f;
			gray = gray / 3.0f;
			grayscale(i, j, 0) = gray;
		}
	}

	normals_.assign(image_.width(), image_.height(), 1, 3);
	for (int i = 0; i < image_.width(); i++) {
		for (int j = 0; j < image_.height(); j++) {
			int ilo = i - 1;
			if (ilo == -1)ilo = image_.width() - 1;
			int ihi = i + 1;
			if (ihi == image_.width())ihi = 0;
			int jlo = j - 1;
			if (jlo == -1)jlo = image_.height() - 1;
			int jhi = j + 1;
			if (jhi == image_.height())jhi = 0;
			float dfdx = grayscale(ihi, j, 0) - grayscale(ilo, j, 0);
			float dfdy = grayscale(i, jhi, 0) - grayscale(i, jlo, 0);
			float norm = sqrt(dfdx*dfdx + dfdy * dfdy + 1.0f);
			normals_(i, j, 0, 0) = -dfdx / norm;
			normals_(i, j, 0, 1) = -dfdy / norm;
			normals_(i, j, 0, 2) = 1.0f / norm;
			//normals_(i, j, 0, 0) = 255.0f*0.5f*(normals_(i, j, 0, 0) + 1.0);
			//normals_(i, j, 0, 1) = 255.0f*0.5f*(normals_(i, j, 0, 1) + 1.0);
			//normals_(i, j, 0, 2) = 255.0f*(normals_(i, j, 0, 2));
		}
	}
	//normals_.save("normalmap.bmp");
}

void TextureLayer::LoadImage(std::string imagefile) {
	//load the image
	image_.load(imagefile.c_str());
	CreateNormalMap();
}

} //namespace raytracer 
} //namespace mavs