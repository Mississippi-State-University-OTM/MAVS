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
* \file scene_editor.cpp
*
* Add vegetation to a surface using polygons
*
* Usage: > scene_editor
*
* \author Chris Goodin
*
* \date 2/28/2019
*/
#ifdef USE_MPI
#include <mpi.h>
#endif
#include "scene_editor.h"
#include <sensors/io/user_io.h>

int main(int argc, char *argv[]) {
	std::string surface_file = mavs::io::GetInputFileName("Select surface mesh", "*.obj");
	if (surface_file.size() <= 4) return 0;

	mavs::utils::SceneEditor scene_editor;
	scene_editor.LoadSurface(surface_file);
	scene_editor.RunEditLoop();
	return 0;
}
