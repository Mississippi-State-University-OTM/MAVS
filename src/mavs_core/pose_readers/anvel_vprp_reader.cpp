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
#include <mavs_core/pose_readers/anvel_vprp_reader.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <mavs_core/math/utils.h>

namespace mavs{
  
static std::vector<std::string> GetWords(std::string line){
  std::vector<std::string> words;
  std::istringstream sline(line);
  std::string word;
  while(sline>>word){
    std::istringstream subline(word);
    std::string subword;
    while(std::getline(subline,subword,':')){
      words.push_back(subword);
    }
  }
  return words;
}

std::vector<mavs::Pose> AnvelVprpReader::Load(std::string fname,
						int pose_freq){
  std::ifstream fin(fname.c_str());
  std::string line;
  bool data_read = false;

  int lines_read=0;
  while(std::getline(fin,line)){
    std::vector<std::string> words = GetWords(line);
    if (words[0]=="Vehicle Objects"){
      num_vehicle_objects_ = utils::StringToInt(words[1]);
    }
    if (words[0]=="@Data"){
      data_read = true;
    }
    if (data_read && words.size()==8){
      if (lines_read%pose_freq==0){
	mavs::Pose p;
	for (int i=0;i<(int)words.size();i++){
	  p.position.x = utils::StringToDouble(words[1]);
	  p.position.y = utils::StringToDouble(words[2]);
	  p.position.z = utils::StringToDouble(words[3]);
	  p.quaternion.w = utils::StringToDouble(words[4]);
	  p.quaternion.x = utils::StringToDouble(words[5]);
	  p.quaternion.y = utils::StringToDouble(words[6]);
	  p.quaternion.z = utils::StringToDouble(words[7]);
	}
	poses_.push_back(p);
      }
      lines_read++;
    }
  }
  fin.close();
  return poses_;
}

} //namespace mavs
