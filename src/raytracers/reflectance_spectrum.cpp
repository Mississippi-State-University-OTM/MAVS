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
#include <raytracers/reflectance_spectrum.h>
#include <fstream>
#include <algorithm> 
#include <mavs_core/math/utils.h>

namespace mavs{

ReflectanceSpectrum::ReflectanceSpectrum() {
	wavelength_.resize(0);
	reflectance_.resize(0);
}

void ReflectanceSpectrum::Load(std::string infile) {
	if (!mavs::utils::file_exists(infile)) {
		std::cerr << "ERROR: Spectrum file " << infile << " does not exist." << std::endl;
	}
	std::ifstream fin(infile);
	if (fin.is_open()) {
		while (!fin.eof()) {
			float w, r;
			fin >> w >> r;
			wavelength_.push_back(w);
			reflectance_.push_back(r);
		}
		wavelength_.pop_back();
		reflectance_.pop_back();

		// entries may be backwards, try reversing
		if (wavelength_[1] < wavelength_[0]) {
			std::reverse(wavelength_.begin(), wavelength_.end());
			std::reverse(reflectance_.begin(), reflectance_.end());
		}

		for (int i = 0; i < (wavelength_.size()-1); i++) {
			if (wavelength_[i + 1] <= wavelength_[i]) {
				std::cerr << "WARNING: " << infile << " may not be valid." << std::endl;
				std::cerr << "Wavelengths should be ordered from shortest to longest." << std::endl;
				break;
			}
		}
	}
	else {
		std::cerr << "ERROR: Failed to open spectrum file " << infile << std::endl;
	}
	fin.close();
	name_ = infile;
	mavs::utils::EraseSubString(name_, mavs::utils::GetPathFromFile(infile));
	mavs::utils::EraseSubString(name_, mavs::utils::GetFileExtension(infile));
	name_.pop_back();
	name_.erase(0, 1);
}

float ReflectanceSpectrum::GetReflectanceAtWavelength(float wl) {
	//std::cout<<"Wavelengthg = "<<wl<<" "<<repeat_vals_.count(wl)<<std::endl;
	//if (repeat_vals_.count(wl)==0){ // this is not thread safe 
		if (wavelength_.size() <= 0 || wavelength_.size() != reflectance_.size()) {
			return 0.0f;
		}
		else {
			if (wl<=wavelength_[0]){
				return reflectance_[0];
			}
			else if (wl >= wavelength_.back()) {
				return reflectance_.back();
			}
			else {
				for (int i = 0; i < (wavelength_.size() - 1); i++) {
					if (wl < wavelength_[i+1]) {
						float y0 = reflectance_[i];
						float y1 = reflectance_[i + 1];
						float x0 = wavelength_[i];
						float x1 = wavelength_[i + 1];
						float rho = (y0*(x1 - wl) + y1 * (wl - x0)) / (x1 - x0);
						//repeat_vals_[wl] = rho;
						return rho;
					}
				}
			}
		}
	//}
	//else {
	//	return repeat_vals_[wl];
	//}
	return 0.0f;
} // GetReflectanceAtWavelength

} // namespace mavs


