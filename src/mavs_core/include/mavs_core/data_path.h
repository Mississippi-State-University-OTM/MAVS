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
#ifndef MAVS_DATA_PATH
#define MAVS_DATA_PATH

#include <string>
#include <fstream>
#include <mavs_core/math/utils.h>
#include <tinyfiledialogs.h>

namespace mavs{

#define xstr(s) mavs_str(s)
#define mavs_str(s) #s
  
class MavsDataPath{
 public:
  MavsDataPath(){
		bool loaded_from_file = false;
		if (mavs::utils::file_exists("mavs_config.txt")) {
			std::ifstream fin("mavs_config.txt");
			fin >> mavs_data_path_;
#ifdef USE_CHRONO
			fin >> chrono_data_path_;
#endif
			fin.close();
			loaded_from_file = true;
		}
		else {
			mavs_data_path_ = std::string(xstr(DATA_PATH));
#ifdef USE_CHRONO
			chrono_data_path_ = std::string(xstr(CHRONO_DATA_PATH));
			chrono_data_path_.append("/vehicle/");
#endif
		}

		//Now check if the data path is actually valid
		if (!mavs::utils::path_is_valid(mavs_data_path_)) {
			if (loaded_from_file) {
				std::cerr << "MAVS data path loaded from mavs_config.txt (" << mavs_data_path_ << ") is not valid." << std::endl;
			}
			else {
				std::cerr << "Default MAVS data path " << mavs_data_path_ << " is not valid." << std::endl;
			}
			SetDataDirectory();
		}

  }

  std::string GetPath(){return mavs_data_path_;}

  std::string GetChronoPath() { return chrono_data_path_; }
  
 private:
  std::string mavs_data_path_;
  std::string chrono_data_path_;

	void SetDataDirectory() {
		std::string message_string("Current data path of " + mavs_data_path_ + " is invalid.\n Press OK to select a new folder.");
		bool set_new = tinyfd_messageBox("Select MAVS Data path", message_string.c_str(), "ok", "info", 1);
		char const * lTheSelectFolderName;
		lTheSelectFolderName = tinyfd_selectFolderDialog("Select MAVS data folder", NULL);
		if (lTheSelectFolderName) {
			mavs_data_path_ = std::string(lTheSelectFolderName);
			std::ofstream fout("mavs_config.txt");
			fout << mavs_data_path_ << std::endl;
			fout.close();
		}
		if (!mavs::utils::path_is_valid(mavs_data_path_)) SetDataDirectory();
	}

};
 
} //namespace mavs

#endif
