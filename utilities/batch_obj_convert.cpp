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
/**
* \file batch_convert_obj.cpp
*
* Convert a list of obj files to binary and save in place
*
* Usage: >./batch_obj_convert input_file_list.txt
*
* \author Chris Goodin
*
* \date 3/10/2020
*/
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <raytracers/mesh.h>
#include <mavs_core/math/utils.h>

int main(int argc, char *argv[]) {

	if (argc < 2) {
		std::cerr << "Usage: ./batch_obj_convert file_list.txt" << std::endl;
		exit(1);
	}
	std::vector<std::string> files_to_convert;
	std::ifstream fin(argv[1]);
	std::string infile;
	while (!fin.eof()) {
		fin >> infile;
		files_to_convert.push_back(infile);
	}
	files_to_convert.pop_back();

	std::string inpath = files_to_convert[0];

	for (int i = 1; i < files_to_convert.size(); i++) {
		std::string meshfile = files_to_convert[i];
		if (mavs::utils::file_exists(meshfile)) {
			std::cout << "Converting " << meshfile << std::endl;
			mavs::raytracer::Mesh mesh;
			mesh.Load(inpath, inpath+meshfile);
			std::string outfile_name = meshfile;
			for (int t = 0; t < 3; t++)outfile_name.pop_back();
			outfile_name.append("bin");
			mesh.WriteBinary(inpath,outfile_name);
		}
	}
	return 0;
}
