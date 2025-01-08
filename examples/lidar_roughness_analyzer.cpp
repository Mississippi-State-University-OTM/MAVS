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
* \file lidar_roughness_analyzer.cpp
*
* Generate lidar data from a rough surface
*
* Usage: >./lidar_roughness_analyzer input_file.json
*
* input_file.json is the input file that specifies the properties
* of the surface and sensr
*
* The simulation will save several files
*
* scene_stats.txt - specifies the number of triangles in the scene
*
* lidar_output.bmp - A top-down rendering of the point cloud
*
* lidar_output.pts - The x-y-z-i space delimited column file of the points
*
* rough_scene.bmp - An rgb rendering of the scene from the point of view of the lidar sensor
*
* \date 3/7/2019
*/
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>
#include <sensors/mavs_sensors.h>
#include <mavs_core/math/utils.h>
#include <mavs_core/data_path.h>
#include "mavs_core/terrain_generator/heightmap.h"
#include "mavs_core/terrain_generator/random_surface.h"
#include <FastNoise.h>
#ifdef USE_EMBREE
#include <raytracers/embree_tracer/embree_tracer.h>
#endif
#include <ctime>
#ifdef Bool
#undef Bool
#endif

struct NoiseModel {
	float frequency;
	float magnitude;
	FastNoise model;
};

namespace lra_globals{
//variables
std::vector<NoiseModel> noise;
float mesh_resolution;
float mesh_size;
float sensor_height;
float drive_time;
float drive_speed;
float plant_density;
int num_iterations;
bool use_gaussian_surface;
std::string base_name;
mavs::sensor::lidar::Lidar *lidar;
mavs::terraingen::HeightMap *heightmap;
//constants
const float dt = 0.1f;
const std::string surface_mesh_file = "noisy_surface";
}

void LoadInputFile(std::string input_file) {
	if (!mavs::utils::file_exists(input_file)) {
		std::cout << "ERROR: Input file " << input_file << " does not exist." << std::endl;
		exit(2);
	}
	FILE* fp = fopen(input_file.c_str(), "rb");
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	lra_globals::use_gaussian_surface = false;
	if (d.HasMember("Noise Type")) {
		std::string noise_type = d["Noise Type"].GetString();
		if (noise_type == "gaussian" || noise_type == "Gaussian") {
			lra_globals::use_gaussian_surface = true;
		}
	}

	if (d.HasMember("Noise")) {
		for (int i = 0; i < (int)d["Noise"].Capacity(); i++) {
			NoiseModel noisemodel;
			if (d["Noise"][i].HasMember("Frequency")) {
				noisemodel.frequency = d["Noise"][i]["Frequency"].GetFloat();
				noisemodel.model.SetFrequency(noisemodel.frequency);
			}
			else {
				std::cerr << "ERROR: No Frequency value listed for noise model " << i << std::endl;
				exit(10);
			}
			if (d["Noise"][i].HasMember("Magnitude")) {
				noisemodel.magnitude = d["Noise"][i]["Magnitude"].GetFloat();
			}
			else {
				std::cerr << "ERROR: No Magnitude value listed for noise model " << i << std::endl;
				exit(11);
			}
			lra_globals::noise.push_back(noisemodel);
		}
	}
	else {
		std::cerr << "ERROR: No noise parameters listed in json file " << input_file<<std::endl;
		exit(1);
	}

	lra_globals::base_name = "lidar_output";
	if (d.HasMember("Output Name")) {
		lra_globals::base_name = d["Output Name"].GetString();
	}

	lra_globals::plant_density = 0.0f;
	if (d.HasMember("Plant Density")) {
		lra_globals::plant_density = d["Plant Density"].GetFloat();
		lra_globals::plant_density = mavs::math::clamp(lra_globals::plant_density, 0.0f, 10.0f);
	}

	lra_globals::mesh_size = 100.0f;
	if (d.HasMember("Mesh Size")) {
		lra_globals::mesh_size = d["Mesh Size"].GetFloat();
	}

	lra_globals::drive_time = 0.0;
	lra_globals::drive_speed = 0.0;
	if (d.HasMember("Driving Properties")) {
		if (d["Driving Properties"].HasMember("Speed")) {
			lra_globals::drive_speed = d["Driving Properties"]["Speed"].GetFloat();
		}
		if (d["Driving Properties"].HasMember("Duration")) {
			lra_globals::drive_time = d["Driving Properties"]["Duration"].GetFloat();
		}
	}

	lra_globals::mesh_resolution = 0.1f;
	if (d.HasMember("Mesh Resolution")) {
		lra_globals::mesh_resolution = d["Mesh Resolution"].GetFloat();
	}

	lra_globals::sensor_height = 1.5f;
	if (d.HasMember("Sensor Height")) {
		lra_globals::sensor_height = d["Sensor Height"].GetFloat();
	}

	lra_globals::num_iterations = 1;
	if (d.HasMember("Number of Scans")) {
		lra_globals::num_iterations = d["Number of Scans"].GetInt();
	}

	if (d.HasMember("Lidar Sensor Type")){
		std::string sensor_type = d["Lidar Sensor Type"].GetString();
		if (sensor_type == "HDL-32E") {
			lra_globals::lidar = new mavs::sensor::lidar::Hdl32E;
		}
		else if (sensor_type == "HDL-64E") {
			lra_globals::lidar = new mavs::sensor::lidar::Hdl64E;
		}
		else if (sensor_type == "VLP-16") {
			lra_globals::lidar = new mavs::sensor::lidar::Vlp16;
		}
		else if (sensor_type == "M8") {
			lra_globals::lidar = new mavs::sensor::lidar::MEight;
		}
		else if (sensor_type == "OS1") {
			lra_globals::lidar = new mavs::sensor::lidar::OusterOS1;
		}
		else if (sensor_type == "OS2") {
			lra_globals::lidar = new mavs::sensor::lidar::OusterOS2;
		}
	}
	else {
		lra_globals::lidar = new mavs::sensor::lidar::Hdl32E;
	}
}

void ReInitializeNoiseModels() {
	long int t = (long int)std::time(NULL);
	for (int i = 0; i < lra_globals::noise.size(); i++) {
		lra_globals::noise[i].model.SetSeed(t);
	}
}

void CreateSurface() {
	float llx = -0.5f*lra_globals::mesh_size;
	float lly = llx;
	float urx = 0.5f*lra_globals::mesh_size;
	float ury = urx;
	if (lra_globals::use_gaussian_surface) {
		mavs::terraingen::RandomSurface surface;
		surface.SetRoughnessParams(1.0f, 0.0f, 1.0f / lra_globals::noise[0].frequency, lra_globals::noise[0].magnitude);
		surface.GenerateGaussianHeightMap(llx, lly, urx, ury, lra_globals::mesh_resolution);
		lra_globals::heightmap = surface.GetHeightMapPointer();
	}
	else {
		lra_globals::heightmap->SetCorners(llx, lly, urx, ury);
		lra_globals::heightmap->SetResolution(lra_globals::mesh_resolution);
		int nx = (int)(ceil(urx - llx) / lra_globals::mesh_resolution) + 1;
		int ny = (int)(ceil(ury - lly) / lra_globals::mesh_resolution) + 1;
		lra_globals::heightmap->Resize(nx, ny);
		nx = (int)lra_globals::heightmap->GetHorizontalDim();
		ny = (int)lra_globals::heightmap->GetVerticalDim();
		for (int i = 0; i < nx; i++) {
			float x = (i + 0.5f)*lra_globals::mesh_resolution;
			for (int j = 0; j < ny; j++) {
				float y = (j + 0.5f)*lra_globals::mesh_resolution;
				float z = 0.0;
				for (int k = 0; k < lra_globals::noise.size(); k++) {
					z += (float)(lra_globals::noise[k].magnitude * lra_globals::noise[k].model.GetPerlin(x, y));
				}
				lra_globals::heightmap->SetHeight(i, j, z);
			}
		}
	}
	lra_globals::heightmap->WriteImage((lra_globals::surface_mesh_file + ".bmp"));
	lra_globals::heightmap->WriteObj(lra_globals::surface_mesh_file,true,false);
}

void SaveScene(std::string outfile) {
	rapidjson::StringBuffer s;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(s);
	writer.SetIndent(' ', 2);

	mavs::MavsDataPath mavs_data_path;
	std::string data_path = mavs_data_path.GetPath();
	std::string path_to_meshes = data_path + "/scenes/meshes/";

	writer.StartObject(); {

		writer.Key("PathToMeshes");
		writer.String(path_to_meshes.c_str());

		writer.Key("UseFullPaths");
		writer.Bool(true);

		std::string surf_name = lra_globals::surface_mesh_file + ".obj";
		std::string path_file = "temp_trail.vprp";
		mavs::Trail trail;
		glm::vec3 position(-0.499f*lra_globals::mesh_size, 0.0f, 0.0f);
		glm::quat orientation(1.0f, 0.0f, 0.0f, 0.0f);
		trail.AddPose(position, orientation);
		position.x = 0.0f;
		trail.AddPose(position, orientation);
		position.x = 0.499f*lra_globals::mesh_size;
		trail.AddPose(position, orientation);
		trail.SaveToAnvelVprp(path_file);
		//trail.SetPathFile(pathout);

		std::string texture_file = data_path + "/scenes/meshes/surface_textures/meadow_surfaces.json";
		writer.Key("Layered Surface");
		writer.StartObject(); {
			writer.Key("Mesh");
			writer.String(surf_name.c_str());
			writer.Key("Trail");
			writer.StartObject(); {
				writer.Key("Trail Width");
				writer.Double(2.14);
				writer.Key("Track Width");
				writer.Double(0.25);
				writer.Key("Wheelbase");
				writer.Double(1.57);
				writer.Key("Path");
				writer.String(path_file.c_str());
				writer.EndObject(); 
			}
			writer.Key("Layers");
			writer.String(texture_file.c_str());
			writer.EndObject(); 
		}

		//writer.Key("Object Labels");
		//writer.String("labels.json");
		//surface mesh array
		writer.Key("Surface Mesh");
		writer.StartArray(); {
			writer.StartObject(); {
				writer.Key("Mesh");
				writer.String(surf_name.c_str());
				writer.Key("Rotate Y to Z");
				writer.Bool(false);
				writer.Key("YawPitchRoll");
				writer.StartArray();
				for (int i = 0; i < 3; i++)writer.Double(0.0);
				writer.EndArray();
				writer.Key("Position");
				writer.StartArray();
				for (int i = 0; i < 3; i++)writer.Double(0.0);
				writer.EndArray();
				writer.Key("Scale");
				writer.StartArray();
				for (int i = 0; i < 3; i++)writer.Double(10.0);
				writer.EndArray();
				writer.EndObject(); 
			}
			writer.EndArray(); 
		}
		int num_plants = (int)(lra_globals::mesh_size * lra_globals::mesh_size*lra_globals::plant_density);
		// write plant objects
		writer.Key("Objects");
		writer.StartArray(); {
			//now loop through veg object instances
			writer.StartObject(); {
				writer.Key("Mesh");
				writer.String("GC17_3.obj"); //dandelion
				writer.Key("Rotate Y to Z");
				writer.Bool(false);
				writer.Key("Random");
				writer.StartObject(); {
					writer.Key("Offset");
					writer.StartArray(); {
						writer.Double(0.0);
						writer.Double(0.0);
						writer.Double(0.0);
						writer.EndArray();
						writer.Key("Number");
						writer.Int(num_plants);
						writer.Key("Polygon");
						writer.StartArray(); {
							writer.StartArray(); {
								writer.Double(-0.5*lra_globals::mesh_size);
								writer.Double(-0.5*lra_globals::mesh_size);
								writer.EndArray();
							}
							writer.StartArray(); {
								writer.Double(-0.5*lra_globals::mesh_size);
								writer.Double(0.5*lra_globals::mesh_size);
								writer.EndArray();
							}
							writer.StartArray(); {
								writer.Double(0.5*lra_globals::mesh_size);
								writer.Double(0.5*lra_globals::mesh_size);
								writer.EndArray();
							}
							writer.StartArray(); {
								writer.Double(0.5*lra_globals::mesh_size);
								writer.Double(-0.5*lra_globals::mesh_size);
								writer.EndArray();
							}
							writer.EndArray();
						}
						writer.Key("Scale");
						writer.StartArray(); {
							writer.Double(0.05);
							writer.Double(0.15);
							writer.EndArray();
						}
					}
					writer.EndObject();
				} // end random object
				writer.EndObject(); 
			} //end mesh object
			writer.EndArray(); 
		} // array of objects
		writer.EndObject(); 
	} // close json file
	std::string outstring = s.GetString();
	std::ofstream fout(outfile.c_str());
	fout << outstring;
	fout.close();
}

void ProcessPoints(std::vector<glm::vec3> &points, float sensor_height) {
	int np = (int)points.size();
	//std::unordered_map<int, float> min_angles;
	for (int i = 0; i < np; i++) {
			float r = sqrt(points[i].x*points[i].x + points[i].y*points[i].y);
			float theta_f = (float)(mavs::kPi_2 + atan(points[i].z / r));
			std::cout << theta_f << " " << (sensor_height+points[i].z) << std::endl;
			/*int theta = (int)(180.0*theta_f / mavs::kPi);
			if (min_angles.find(theta) == min_angles.end()) {
				min_angles[theta] = points[i].z;
			}
			else {
				if (points[i].z < min_angles[theta]) {
					min_angles[theta] = points[i].z;
				}
			}*/
	}

	/*std::unordered_map<int, float>::iterator it;
	for (it = min_angles.begin(); it != min_angles.end(); it++) {
		std::cout << it->first << " " << (sensor_height+it->second) << std::endl;
	}
	*/
}

int main (int argc, char *argv[]){
	if (argc < 2) {
		std::cerr << "ERROR: No input json file given. " << std::endl;
		exit(3);
	}
	// Load inputs
	std::string input_file(argv[1]);
	LoadInputFile(input_file);

	std::vector<glm::vec3> points;

	for (int i = 0; i < lra_globals::num_iterations; i++) {
		std::cout << "Generating terrain " << (i+1) << std::endl;
		//Create surface
		CreateSurface();

		//Save Scene
		std::string scenefile("temp_scene.json");
		SaveScene(scenefile);

#ifdef USE_EMBREE
		// Create the scene
		mavs::environment::Environment env;
		mavs::raytracer::embree::EmbreeTracer scene;
		scene.Load(scenefile);
		env.SetRaytracer(&scene);

		//mavs::sensor::camera::RgbCamera camera;

		// Set initial position and orientation of sensor
		glm::vec3 position(0.0f, 0.0f, 0.0f);
		//glm::vec2 pos(0.0f, 0.0f);
		glm::vec2 look_to(1.0f, 0.0f);
		glm::quat orientation(1.0f, 0.0f, 0.0f, 0.0f);

		/*int num_steps = 1;
		if (drive_time > 0.0f) {
			num_steps = (int)ceil(drive_time / dt);
		}

		for (int t = 0; t < num_steps; t++) {*/
			//std::cout << "Terrain " << (i + 1) << " of " << num_iterations << ", frame " << (t + 1) << " of " << num_steps << std::endl;
			std::cout << "Terrain " << (i + 1) << " of " << lra_globals::num_iterations << std::endl;
			
			// Update the sensors
			//heightmap.GetPoseAtPosition(pos, look_to, position, orientation);
			position.z = scene.GetSurfaceHeight(position.x, position.y) + lra_globals::sensor_height;
			lra_globals::lidar->SetPose(position, orientation);
			lra_globals::lidar->Update(&env, 0.1);
			std::vector<glm::vec3> these_points = lra_globals::lidar->GetPoints();
			for (int i = 0; i < (int)these_points.size(); i++) {
				if (!(these_points[i].x == 0.0f && these_points[i].y == 0.0f)) {
					points.push_back(these_points[i]);
				}
			}
			//camera.SetPose(position, orientation);
			//camera.Update(&env, 0.03);
			
			// Save data
			/*std::string snum = mavs::utils::ToString(i);
			std::string fnum = mavs::utils::ToString(t);
			std::string pts_name = base_name + "_scene" +snum + "_frame" + fnum + ".pts";
			std::string bmp_name = base_name + "_scene" + snum + "_frame" + fnum + ".bmp";
			std::string image_name = base_name + "_render_scene" + snum + "_frame" + fnum + ".bmp";
			lidar->RegisterPoints();
			lidar->WritePointsToText(pts_name);
			lidar->WritePointsToImage(bmp_name);
			camera.SaveImage(image_name);*/

			//Update vehicle position
		//	position.x = position.x + dt * drive_speed;
		//}
		// Reinitialize noise models for new terrains
		ReInitializeNoiseModels();
	}
	ProcessPoints(points, lra_globals::sensor_height);
#endif
  return 0;
}

