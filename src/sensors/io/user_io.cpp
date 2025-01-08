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
#include <sensors/io/user_io.h>

#include <algorithm>
#include <iostream>

#include <mavs_core/data_path.h>
#include <mavs_core/math/utils.h>
#include <tinyfiledialogs.h>


namespace mavs {
namespace io {

std::string GetInputFileName(std::string message, std::string ftype) {
	mavs::MavsDataPath data_path;
	std::string mavs_data_path = data_path.GetPath();
	char const * lTheOpenFileName;
	char const * lFilterPatterns[1] = { ftype.c_str() };
	lTheOpenFileName = tinyfd_openFileDialog(
		message.c_str(),
		mavs_data_path.c_str(),
		1,
		lFilterPatterns,
		NULL,
		0);
	if (lTheOpenFileName) {
		std::string obj_file(lTheOpenFileName);
		std::replace(obj_file.begin(), obj_file.end(), '\\', '/');
		return obj_file;
	}
	else {
		std::cerr << "File not found" << std::endl;
		std::string obj_file("");
		return obj_file;
	}
}

std::string GetSaveFileName(std::string message, std::string default_type, std::string ftype) {
	std::string save_file;
	char const * lTheSaveFileName;
	char const * lFilterPatterns[1] = { ftype.c_str() };
	lTheSaveFileName = tinyfd_saveFileDialog(
		message.c_str(),
		default_type.c_str(),
		1,
		lFilterPatterns,
		NULL);
	if (lTheSaveFileName) {
		std::string save_file(lTheSaveFileName);
		std::replace(save_file.begin(), save_file.end(), '\\', '/');
		return save_file;
	}
	else {
		std::cerr << "Save file not selected" << std::endl;
		std::string save_file("");
		return save_file;
	}
}

float GetUserNumericInput(std::string title, std::string message) {
	float val = 0.0f;
	char const * lTmp;
	lTmp = tinyfd_inputBox(title.c_str(), message.c_str(), "0.0");
	if (lTmp) {
		std::string tmp(lTmp);
		val = (float)mavs::utils::StringToDouble(tmp);
	}
	return val;
}

bool GetUserBool(std::string question, std::string message) {
	int answer = tinyfd_messageBox(question.c_str(),
		message.c_str(),
		"yesno", "question", 0);
	if (answer == 0) {
		return false;
	}
	else {
		return true;
	}
}

void DisplayUserInfo(std::string title, std::string message) {
	tinyfd_messageBox(title.c_str(), message.c_str(), "ok", "info", 1);
}

} //namespace io
} //namespace mavs