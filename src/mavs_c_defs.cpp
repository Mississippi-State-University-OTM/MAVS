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
* \file mavs_c_defs.cpp
*
* see: http://www.auctoris.co.uk/2017/04/29/calling-c-classes-from-python-with-ctypes/
*
*/
#ifndef USE_MPI

#include "mavs_c_defs.h"
//#include "vehicles/car/full_car.h"
#include <vehicles/rp3d_veh/mavs_rp3d_veh.h>
#include <sensors/mavs_sensors.h>
#include "sensors/camera/add_rain_to_existing_image.h"
#include <sensors/lidar/lidar_tools.h>
#include "raytracers/material.h"
#include <glm/gtx/rotate_vector.hpp>
#include <iostream>
#include <new>
#include <mavs_core/data_path.h>
#ifdef USE_CHRONO
#include <vehicles/chrono/chrono_wheeled_json.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32) || defined(WIN32)
#define EXPORT_CMD __declspec(dllexport)
#else
#define EXPORT_CMD 
#endif

	//--- Embree Raytracer interfaces --------------------------//
#ifdef USE_EMBREE
	EXPORT_CMD mavs::raytracer::embree::EmbreeTracer* NewEmbreeScene(void) {
		return new mavs::raytracer::embree::EmbreeTracer;
	}

	EXPORT_CMD void WriteEmbreeSceneStats(mavs::raytracer::embree::EmbreeTracer* rt, char* directory) {
		std::string outdir(directory);
		rt->WriteSceneStats(outdir);
	}

	EXPORT_CMD void LoadEmbreeScene(mavs::raytracer::embree::EmbreeTracer* rt, char* fname) {
		std::string infile = std::string(fname);
		bool loaded = rt->Load(infile);
	}

	EXPORT_CMD void LoadEmbreeSceneWithRandomSeed(mavs::raytracer::embree::EmbreeTracer* rt, char* fname) {
		int rand_seed = (int)time(NULL);
		std::string infile = std::string(fname);
		bool loaded = rt->Load(infile,rand_seed);
	}

	EXPORT_CMD void DeleteEmbreeScene(mavs::raytracer::embree::EmbreeTracer* rt) {
		delete rt;
	}

	EXPORT_CMD void SetEnvironmentScene(mavs::environment::Environment* env, mavs::raytracer::embree::EmbreeTracer* scene) {
		env->SetRaytracer(scene);
	}

	EXPORT_CMD float GetSurfaceHeight(mavs::raytracer::embree::EmbreeTracer* rt, float x, float y) {
		float height = rt->GetSurfaceHeight(x, y);
		return height;
	}

	EXPORT_CMD float * PutWaypointsOnGround(mavs::raytracer::embree::EmbreeTracer* rt, mavs::Waypoints * wp) {
		int np = (int)wp->NumWaypoints();
		float * heights = new float[np];
		for (int i = 0; i < np; i++) {
			glm::vec2 p = wp->GetWaypoint(i);
			heights[i] = rt->GetSurfaceHeight(p.x, p.y);
		}
		return heights;
	}

	EXPORT_CMD mavs::raytracer::embree::EmbreeTracer* CreateGapScene(float w, float l, float roughmag,
		float mr, char* basename, float plant_density, char* ecofile, char* output_dir, float gap_width, float gap_depth, float gap_angle_radians) {
		
		mavs::terraingen::RandomSceneInputs input;
		input.terrain_width = w;
		input.terrain_length = l;
		input.lo_mag = 0.0;
		input.hi_mag = roughmag;
		input.mesh_resolution = mr;
		input.trail_width = 0.0f;
		input.wheelbase = 0.0f;
		input.track_width = 0.0;
		input.path_type = "ridges";
		input.basename = std::string(basename);
		input.surface_rough_type = "gap";
		input.plant_density = plant_density;

		input.output_directory = std::string(output_dir); // ".";
		mavs::MavsDataPath mavs_data_path;
		std::string eco_path = mavs_data_path.GetPath() + "/ecosystem_files/";
		input.eco_file = eco_path + std::string(ecofile);
		
		
		mavs::terraingen::RandomScene random_scene;

		random_scene.SetInputs(input);
		//random_scene.PrintInputs();
		random_scene.SetGapProperties(gap_width, gap_depth, gap_angle_radians);
		random_scene.Create();

		std::string scene_file(basename);
		scene_file = random_scene.GetOutputDirectory() + "/" + scene_file + "_scene.json";

		mavs::raytracer::embree::EmbreeTracer * scene = new mavs::raytracer::embree::EmbreeTracer;

		scene->Load(scene_file);
		return scene;
	}

	EXPORT_CMD mavs::raytracer::embree::EmbreeTracer* CreateSceneFromRandom(float w, float l, float lm, float hm,
		float mr, float tw, float wb, float track, char* path_type, char * roughness_type, char* basename, float plant_density,
		 std::vector<glm::vec4> * potholes, char* ecofile, char* output_dir) {
		mavs::terraingen::RandomSceneInputs input;
		input.terrain_width = w;
		input.terrain_length = l;
		input.lo_mag = lm;
		input.hi_mag = hm;
		input.mesh_resolution = mr;
		input.trail_width = tw;
		input.wheelbase = wb;
		input.track_width = track;
		input.path_type = std::string(path_type);
		input.basename = std::string(basename);
		input.surface_rough_type = std::string(roughness_type);
		input.plant_density = plant_density;
		//input.pothole_depth = pothole_depth;
		//input.pothole_diameter = pothole_diameter;
		//input.number_potholes = num_potholes;
		//input.pothole_locations.resize(pothole_locations->size());
		input.potholes.resize(potholes->size());
		for (int p = 0; p < input.potholes.size(); p++) {
			glm::vec4 hole = potholes->at(p);
			input.potholes[p].location.x = hole.x;
			input.potholes[p].location.y = hole.y;
			input.potholes[p].depth = hole.z;
			input.potholes[p].diameter = hole.w;
		}
		//for (int i = 0; i < pothole_locations->size(); i++) {
		//	input.pothole_locations[i] = pothole_locations->at(i);
		//}

		//if (pothole_depth > 0.0f && pothole_diameter > 0.0f)input.potholes = true;
		input.output_directory = std::string(output_dir); // ".";
		mavs::MavsDataPath mavs_data_path;
		std::string eco_path = mavs_data_path.GetPath() + "/ecosystem_files/";
		input.eco_file = eco_path + std::string(ecofile);

		mavs::terraingen::RandomScene random_scene;

		random_scene.SetInputs(input);
		//random_scene.PrintInputs();

		random_scene.Create();
		
		std::string scene_file(input.basename);
		scene_file = random_scene.GetOutputDirectory()+"/"+scene_file + "_scene.json";
		
		std::string path_file(input.basename);
		path_file = path_file + "_path.vprp";
		
		mavs::raytracer::embree::EmbreeTracer * scene = new mavs::raytracer::embree::EmbreeTracer;

		scene->Load(scene_file);
		return scene;
	}

	EXPORT_CMD int AddAnimationToScene(mavs::raytracer::embree::EmbreeTracer* rt, mavs::raytracer::Animation *anim) {
		int id = rt->AddAnimation(*anim);
		return id;
	}

	EXPORT_CMD void SetAnimationPositionInScene(mavs::environment::Environment* env, int anim_id, float x, float y, float heading) {
		env->SetAnimationPosition(anim_id, x, y, heading);
	}

	EXPORT_CMD mavs::raytracer::Animation* NewMavsAnimation() {
		mavs::raytracer::Animation *anim = new mavs::raytracer::Animation;
		return anim;
	}

	EXPORT_CMD void DeleteMavsAnimation(mavs::raytracer::Animation *anim) {
		delete anim;
	}

	EXPORT_CMD void SetMavsAnimationScale(mavs::raytracer::Animation *anim, float scale) {
		anim->SetMeshScale(scale);
	}

	EXPORT_CMD void SetMavsAnimationRotations(mavs::raytracer::Animation *anim, bool y_to_x, bool y_to_z) {
		if (y_to_x)anim->SetRotateYToX(y_to_x);
		if (y_to_z)anim->SetRotateYToZ(y_to_z);
	}

	EXPORT_CMD void SetMavsAnimationSpeed(mavs::raytracer::Animation *anim, float speed) {
		anim->SetSpeed(speed);
	}

	EXPORT_CMD void SetMavsAnimationPosition(mavs::raytracer::Animation *anim, float x, float y) {
		anim->SetPosition(x, y);
	}

	EXPORT_CMD void SetMavsAnimationHeading(mavs::raytracer::Animation *anim, float heading) {
		anim->SetHeading(heading);
	}

	EXPORT_CMD void SetMavsAnimationBehavior(mavs::raytracer::Animation *anim, char *behavior) {
		std::string behave(behavior);
		anim->SetBehavior(behave);
	}

	EXPORT_CMD void MoveMavsAnimationToWaypoint(mavs::raytracer::Animation *anim, float dt, float x, float y) {
		anim->MoveToWaypoint(dt, glm::vec3(x, y, 0.0f));
	}

	EXPORT_CMD void LoadMavsAnimation(mavs::raytracer::Animation *anim, char *path_to_meshes, char *framefile_name) {
		std::string framefile(framefile_name);
		std::string path(path_to_meshes);
		anim->SetPathToMeshes(path);
		anim->LoadFrameList(framefile);
	}

	EXPORT_CMD void LoadAnimationPathFile(mavs::raytracer::Animation *anim, char *path_file) {
		std::string path(path_file);
		anim->LoadPathFile(path);
	}

#endif //use EMBREE

	EXPORT_CMD mavs::terraingen::GridSurface *LoadDem(char* fname, bool interp_no_data, bool recenter) {
		std::string asc_file_name(fname);
		mavs::terraingen::GridSurface *surface = new mavs::terraingen::GridSurface;
		std::cout << "Loading ascii file " << asc_file_name << std::endl;
		surface->LoadAscFile(asc_file_name);
		if (interp_no_data) surface->RemoveNoDataCells();
		if (recenter) surface->Recenter();
		return surface;
	}

	EXPORT_CMD void DownsampleDem(mavs::terraingen::GridSurface *surface, int dsfac) {
		surface->GetHeightMapPointer()->Downsample(dsfac, surface->GetNoDataValue());
	}

	EXPORT_CMD void DisplayDem(mavs::terraingen::GridSurface *surface) {
		surface->DisplaySlopes();
		surface->Display();
	}

	EXPORT_CMD void ExportDemToObj(mavs::terraingen::GridSurface *surface, char* obj_fname) {
		surface->WriteObj(std::string(obj_fname));
	}

	EXPORT_CMD void ExportDemToEsriAscii(mavs::terraingen::GridSurface *surface, char* esri_fname) {
		surface->WriteElevToGrd(std::string(esri_fname));
	}

	EXPORT_CMD std::vector<glm::vec2> * NewPointList2D() {
		std::vector<glm::vec2> * list = new std::vector<glm::vec2>;
		return list;
	}

	EXPORT_CMD void DeletePointList2D(std::vector<glm::vec2> * list) {
		delete list;
	}

	EXPORT_CMD void AddPointToList2D(std::vector<glm::vec2> * list, float x, float y) {
		list->push_back(glm::vec2(x, y));
	}

	EXPORT_CMD void DeletePointList4D(std::vector<glm::vec4> * list) {
		delete list;
	}

	EXPORT_CMD void AddPointToList4D(std::vector<glm::vec4> * list, float w, float x, float y, float z) {
		list->push_back(glm::vec4(w, x, y, z));
	}


	EXPORT_CMD mavs::utils::Mplot* NewMavsPlotter(void) {
		return new mavs::utils::Mplot;
	}

	EXPORT_CMD void DeleteMavsPlotter(mavs::utils::Mplot* plot) {
		delete plot;
	}

	EXPORT_CMD void PlotColorMatrix(mavs::utils::Mplot* plot, int width, int height, float *data) {
		std::vector<std::vector<std::vector<float> > > expanded_data = plot->UnFlatten3D(width, height, 3, data);
		plot->PlotColorMap(expanded_data);
	}

	EXPORT_CMD void PlotGrayMatrix(mavs::utils::Mplot* plot, int width, int height, float *data) {
		std::vector<std::vector<float> > expanded_data = plot->UnFlatten2D(width, height, data);
		plot->PlotGrayMap(expanded_data);
	}

	EXPORT_CMD void PlotTrajectory(mavs::utils::Mplot* plot, int np, float *x, float *y) {
		std::vector<float> xv, yv;
		xv.resize(np);
		yv.resize(np);
		for (int i = 0; i < np; i++) {
			xv[i] = x[i];
			yv[i] = y[i];
		}
		plot->PlotTrajectory(xv, yv);
	}

	EXPORT_CMD void AddPlotToTrajectory(mavs::utils::Mplot* plot, int np, float *x, float *y) {
		std::vector<float> xv, yv;
		xv.resize(np);
		yv.resize(np);
		for (int i = 0; i < np; i++) {
			xv[i] = x[i];
			yv[i] = y[i];
		}
		plot->AddToExistingTrajectory(xv, yv);
	}

	EXPORT_CMD mavs::utils::MapViewer* NewMavsMapViewer(float llx, float lly, float urx, float ury, float res) {
		glm::vec2 llc(llx, lly);
		glm::vec2 urc(urx, ury);
		return new mavs::utils::MapViewer(llc, urc, res);
	}

	EXPORT_CMD void DeleteMavsMapViewer(mavs::utils::MapViewer* map) {
		delete map;
	}

	EXPORT_CMD bool MapIsOpen(mavs::utils::MapViewer* map) {
		return map->IsOpen();
	}

	EXPORT_CMD void UpdateMap(mavs::utils::MapViewer* map, mavs::environment::Environment* env) {
		map->UpdateMap(env);
	}

	EXPORT_CMD void AddWaypointsToMap(mavs::utils::MapViewer* map, float *x, float *y, int nwp) {
		std::vector<glm::vec2> wp;
		wp.resize(nwp);
		for (int i = 0; i < nwp; i++) {
			wp[i] = glm::vec2(x[i], y[i]);
		}
		map->AddWaypoints(wp, mavs::utils::yellow);
	}

	EXPORT_CMD void AddCircleToMap(mavs::utils::MapViewer* map, float cx, float cy, float radius) {
		glm::vec2 c(cx, cy);
		map->AddCircle(c, radius, mavs::utils::green);
	}

	EXPORT_CMD void AddLineToMap(mavs::utils::MapViewer* map, float p0x, float p0y, float p1x, float p1y) {
		glm::vec2 p0(p0x, p0y);
		glm::vec2 p1(p1x, p1y);
		map->AddLine(p0, p1, mavs::utils::orange);
	}

	EXPORT_CMD void TurnOnMavsSceneLabeling(mavs::raytracer::Raytracer* rt) {
		rt->TurnOnLabeling();
	}
	EXPORT_CMD void TurnOffMavsSceneLabeling(mavs::raytracer::Raytracer* rt) {
		rt->TurnOffLabeling();
	}

	EXPORT_CMD mavs::environment::Environment* NewMavsEnvironment(void) {
		return new mavs::environment::Environment;
	}

	EXPORT_CMD void DeleteMavsEnvironment(mavs::environment::Environment* env) {
		delete env;
	}

	EXPORT_CMD void FreeEnvironmentScene(mavs::environment::Environment* env) {
		env->FreeRaytracer();
	}

	EXPORT_CMD void AdvanceEnvironmentTime(mavs::environment::Environment* env, float dt) {
		env->AdvanceTime(dt);
	}

	EXPORT_CMD float * GetSceneDensity(mavs::environment::Environment* env, float llx, float lly, float llz, float urx, float ury, float urz, float res) {
		
		glm::vec3 ur(urx, ury, urz);
		glm::vec3 ll(llx, lly, llz);
		std::vector<std::vector<std::vector<float> > > grid = env->GetVegDensityOnGrid(ll, ur, res);
		int nx = grid.size();
		int ny = 0;
		if (nx > 0)ny = grid[0].size();
		int nz = 0;
		if (ny > 0)nz = grid[0][0].size();
		float * data = new float[nx*ny*nz];
		int n = 0;
		for (int i = 0; i < nx; i++) {
			for (int j = 0; j < ny; j++) {
				for (int k = 0; k < nz; k++) {
					data[n] = grid[i][j][k];
					n++;
				}
			}
		}
		return data;
	}

	EXPORT_CMD void TurnSkyOnOff(mavs::environment::Environment* env, bool sky_on) {
		if (sky_on) {
			env->TurnOnSky();
		}
		else {
			env->TurnOffSky();
		}
	}

	EXPORT_CMD void SetSkyColor(mavs::environment::Environment* env, float r, float g, float b) {
		env->SetSkyConstantColor(r, g, b);
	}

	EXPORT_CMD void SetSunColor(mavs::environment::Environment* env, float r, float g, float b) {
		env->SetSunConstantColor(r, g, b);
	}

	EXPORT_CMD void SetSunLocation(mavs::environment::Environment* env, float azimuth_degrees, float zenith_degrees) {
		env->SetSunPosition(azimuth_degrees, zenith_degrees);
	}

	EXPORT_CMD void SetSunSolidAngle(mavs::environment::Environment* env, float solid_angle_degrees) {
		env->SetSunSolidAngle(solid_angle_degrees);
	}


	EXPORT_CMD void SetTime(mavs::environment::Environment* env, int hour) {
		mavs::environment::DateTime now = env->GetDateTime();
		env->SetDateTime(now.year, now.month, now.day, hour, now.minute, now.second, now.time_zone);
	}

	EXPORT_CMD void SetTimeSeconds(mavs::environment::Environment* env, int hour, int minute, int second) {
		mavs::environment::DateTime now = env->GetDateTime();
		env->SetDateTime(now.year, now.month, now.day, hour, minute, second, now.time_zone);
	}

	EXPORT_CMD void SetDate(mavs::environment::Environment* env, int year, int month, int day) {
		mavs::environment::DateTime now = env->GetDateTime();
		env->SetDateTime(year, month, day, now.hour, now.minute, now.second, now.time_zone);
	}

	EXPORT_CMD float * GetAnimationPosition(mavs::environment::Environment* env, int anim_num) {
		glm::vec3 pos = env->GetAnimationPosition(anim_num);
		float * p = new float[3];
		p[0] = pos.x;
		p[1] = pos.y;
		p[2] = pos.z;
		return p;
	}

	EXPORT_CMD int AddActorToEnvironment(mavs::environment::Environment* env, char* fname, bool auto_update) {
		std::string infile(fname);
		std::vector<int> actor_idv = env->LoadActors(infile);
		int actor_num = (int)(actor_idv.size() - 1);
		if (!auto_update) env->UnsetActorUpdate(actor_num);
		return actor_num;
	}

	EXPORT_CMD void SetActorPosition(mavs::environment::Environment* env, int actor_id,
		float position[3], float orientation[4]) {
		glm::vec3 p(position[0], position[1], position[2]);
		glm::quat q(orientation[0], orientation[1], orientation[2], orientation[3]);
		env->SetActorPosition(actor_id, p, q);
	}

	EXPORT_CMD void AddDustToEnvironment(mavs::environment::Environment* env, float px, float py, float pz, float vx, float vy, float vz, float dust_rate, float dustball_size, float velocity_randomization) {
		mavs::environment::ParticleSystem dust;
		dust.Dust();
		dust.SetSource(glm::vec3(px,py,pz), dustball_size, dust_rate);
		dust.SetInitialVelocity(vx,vy,vz);
		dust.SetVelocityRandomization(velocity_randomization, velocity_randomization, velocity_randomization);
		env->AddParticleSystem(dust);
	}

	EXPORT_CMD void AddDustToActor(mavs::environment::Environment* env, int actor_id) {
		mavs::environment::ParticleSystem dust;
		dust.Dust();
		env->AddParticleSystem(dust);
		int n = (int)env->GetNumParticleSystems() - 1;
		env->AssignParticleSystemToActor(n, actor_id);
	}

	EXPORT_CMD void AddDustToActorColor(mavs::environment::Environment* env, int actor_id, float cr, float cg, float cb) {
		mavs::environment::ParticleSystem dust;
		dust.Dust();
		dust.SetInitialColor(cr, cg, cb);
		env->AddParticleSystem(dust);
		int n = (int)env->GetNumParticleSystems() - 1;
		env->AssignParticleSystemToActor(n, actor_id);
	}

	EXPORT_CMD void UpdateParticleSystems(mavs::environment::Environment* env, float dt) {
		env->AdvanceParticleSystems(dt);
	}

	EXPORT_CMD void SetRainRate(mavs::environment::Environment* env, float rain_rate) {
		env->SetRainRate(rain_rate);
	}

	EXPORT_CMD void SetSnowRate(mavs::environment::Environment* env, float snow_rate) {
		env->SetSnowRate(snow_rate);
	}

	EXPORT_CMD void SetSnowAccumulation(mavs::environment::Environment* env, float snow_accum) {
		env->SetSnowAccumulation(snow_accum);
	}

	EXPORT_CMD void SetTurbidity(mavs::environment::Environment* env, float turbidity) {
		env->SetTurbidity(turbidity);
	}

	EXPORT_CMD void SetAlbedo(mavs::environment::Environment* env, float a_r, float a_g, float a_b) {
		env->SetAlbedoRgb(a_r, a_g, a_b);
	}

	EXPORT_CMD void SetWind(mavs::environment::Environment* env, float w_x, float w_y) {
		env->SetWind(w_x, w_y);
	}

	EXPORT_CMD void SetFog(mavs::environment::Environment* env, float fogginess) {
		float fog_k = 0.025f*5.0f*fogginess*1.0E-3f;
		env->SetFog(fog_k);
	}

	EXPORT_CMD void SetCloudCover(mavs::environment::Environment* env, float cloud_cover) {
		env->SetCloudCover(cloud_cover);
	}

	EXPORT_CMD void SetTerrainProperties(mavs::environment::Environment* env, char *soil_type, float soil_strength) {
		std::string st(soil_type);
		env->SetGlobalSurfaceProperties(st, soil_strength);
	}

	EXPORT_CMD int GetNumberOfObjectsInEnvironment(mavs::environment::Environment* env) {
		return env->GetNumberObjects();
	}

	EXPORT_CMD char * GetObjectName(mavs::environment::Environment* env, int object_id) {
		std::string str = env->GetObjectName(object_id);
		char* p = new char[str.length() + 1];
		strcpy(p, str.c_str());
		return p;
	}

	EXPORT_CMD float * GetObjectBoundingBox(mavs::environment::Environment* env, int object_id) {
		mavs::raytracer::BoundingBox bb = env->GetObjectBoundingBox(object_id);
		float * fbb = new float[6];
		fbb[0] = bb.GetLowerLeft().x;
		fbb[1] = bb.GetLowerLeft().y;
		fbb[2] = bb.GetLowerLeft().z;
		fbb[3] = bb.GetUpperRight().x;
		fbb[4] = bb.GetUpperRight().y;
		fbb[5] = bb.GetUpperRight().z;
		return fbb;
	}

	EXPORT_CMD void MoveHeadlights(mavs::environment::Environment* env, mavs::vehicle::Vehicle* veh, float front_offset, float headlight_width, int left_id, int right_id) {
		glm::vec3 veh_lt = veh->GetLookTo();
		glm::vec3 veh_ls = veh->GetLookSide();
		glm::vec3 veh_lu = veh->GetLookUp();
		glm::vec3 veh_pos = veh->GetPosition();
		glm::vec3 left_light_pos = veh_pos + front_offset*veh_lt + 0.5f*headlight_width*veh_ls;
		glm::vec3 right_light_pos = veh_pos + front_offset*veh_lt - 0.5f*headlight_width*veh_ls;
		env->MoveLight(left_id, left_light_pos, veh_lt);
		env->MoveLight(right_id, right_light_pos, veh_lt);
	}

	EXPORT_CMD int AddPointLight(mavs::environment::Environment* env, float cr, float cg, float cb, float px, float py, float pz) {
		int id = env->AddPointlight(glm::vec3(cr, cg, cb), glm::vec3(px, py, pz));
		return id;
	}

	EXPORT_CMD void MoveLight(mavs::environment::Environment* env, int id, float px, float py, float pz, float dx, float dy, float dz) {
		if (id >= 0 && id < env->GetNumLights()) {
			env->MoveLight(id, glm::vec3(px, py, pz), glm::vec3(dx, dy, dz));
		}
	}

	EXPORT_CMD int AddSpotLight(mavs::environment::Environment* env, float cr, float cg, float cb, float px, float py, float pz, float dx, float dy, float dz, float angle) {
		int id = env->AddSpotlight(glm::vec3(cr, cg, cb), glm::vec3(px, py, pz), glm::vec3(dx, dy, dz), angle);
		return id;
	}

	EXPORT_CMD int * AddHeadlightsToVehicle(mavs::environment::Environment* env, mavs::vehicle::Vehicle* veh, float front_offset, float headlight_width) {
		mavs::environment::Light headlight;
		headlight.type = 2; //spotlight
		headlight.angle = (float)(mavs::kDegToRad * 30.0f);
		//headlight.color = glm::vec3(12.0f, 10.0f, 5.0f);
		headlight.color = glm::vec3(6.0f, 5.0f, 2.5f);
		headlight.decay = 0.25f; 
		static int headlight_ids[2];
		headlight_ids[0] = env->AddLight(headlight); //left
		headlight_ids[1] = env->AddLight(headlight); //right
		MoveHeadlights(env, veh, front_offset, headlight_width, headlight_ids[0], headlight_ids[1]);
		return headlight_ids;
	}

	EXPORT_CMD void SetLocalOrigin(mavs::environment::Environment* env, double lat, double lon, double alt){
		env -> SetLocalOrigin(lat,lon,alt);
	}

	//--- Vehicle model constructors ------------//
	EXPORT_CMD mavs::vehicle::Vehicle* NewMavsRp3dVehicle() {
		mavs::vehicle::Vehicle* car = new mavs::vehicle::Rp3dVehicle;
		return car;
	}
	
	EXPORT_CMD float * GetMavsVehicleTirePositionAndOrientation(mavs::vehicle::Vehicle* veh, int tire_num) {
		//mavs::vehicle::Rp3dVehicle* vehicle = static_cast<mavs::vehicle::Rp3dVehicle *>(veh);
		glm::vec3 p = veh->GetTirePosition(tire_num);
		glm::quat o = veh->GetTireOrientation(tire_num);
		static float data[4];
		data[0] = p.x;
		data[1] = p.y;
		data[2] = p.z;
		data[3] = o.w;
		data[4] = o.x;
		data[5] = o.y;
		data[6] = o.z;
		return data;
	}

	EXPORT_CMD float GetRp3dVehicleTireDeflection(mavs::vehicle::Vehicle* veh, int i) {
		mavs::vehicle::Rp3dVehicle* vehicle = static_cast<mavs::vehicle::Rp3dVehicle *>(veh);
		float d = vehicle->GetPercentDeflectionOfTire(i);
		return d;
	}

	EXPORT_CMD float GetRp3dTireSteeringAngle(mavs::vehicle::Vehicle* veh, int i) {
		mavs::vehicle::Rp3dVehicle* vehicle = static_cast<mavs::vehicle::Rp3dVehicle *>(veh);
		float sa = vehicle->GetTireSteeringAngle(i);
		return sa;
	}

	EXPORT_CMD void SetRp3dTerrain(mavs::vehicle::Vehicle* veh, char *soil_type, float soil_strength, char *terrain_type, float terrain_param1, float terrain_param2) {
		mavs::vehicle::Rp3dVehicle* vehicle = static_cast<mavs::vehicle::Rp3dVehicle *>(veh);
		std::vector<float> tp;
		tp.resize(2, 0.0f);
		tp[0] = terrain_param1;
		tp[1] = terrain_param2;
		vehicle->SetTerrainProperties(std::string(soil_type), soil_strength, std::string(terrain_type), tp);
	}

	EXPORT_CMD void SetRp3dGravity(mavs::vehicle::Vehicle* veh, float gx, float gy, float gz) {
		mavs::vehicle::Rp3dVehicle* vehicle = static_cast<mavs::vehicle::Rp3dVehicle *>(veh);
		vehicle->SetGravity(gx, gy, gz);
	}

	EXPORT_CMD void SetRp3dExternalForce(mavs::vehicle::Vehicle* veh, float fx, float fy, float fz) {
		mavs::vehicle::Rp3dVehicle* vehicle = static_cast<mavs::vehicle::Rp3dVehicle *>(veh);
		vehicle->SetExternalForceOnCg(glm::vec3(fx, fy, fz));
	}

	EXPORT_CMD float * GetRp3dLookTo(mavs::vehicle::Vehicle* veh) {
		mavs::vehicle::Rp3dVehicle* vehicle = static_cast<mavs::vehicle::Rp3dVehicle *>(veh);
		glm::vec3 look_to = vehicle->GetLookTo();
		static float lt[3];
		lt[0] = look_to.x;
		lt[1] = look_to.y;
		lt[2] = look_to.z;
		return lt;
	}

	EXPORT_CMD float GetRp3dTireNormalForce(mavs::vehicle::Vehicle* veh, int tire_id) {
		mavs::vehicle::Rp3dVehicle* vehicle = static_cast<mavs::vehicle::Rp3dVehicle *>(veh);
		return vehicle->GetTireForces(tire_id).z;
	}

	EXPORT_CMD float * GetRp3dTireForces(mavs::vehicle::Vehicle* veh, int tire_id) {
		mavs::vehicle::Rp3dVehicle* vehicle = static_cast<mavs::vehicle::Rp3dVehicle *>(veh);
		glm::vec3 f = vehicle->GetTireForces(tire_id);
		static float forces[3];
		forces[0] = f.x;
		forces[1] = f.y;
		forces[2] = f.z;
		return forces;
	}

	EXPORT_CMD float GetRp3dTireSlip(mavs::vehicle::Vehicle* veh, int tire_id) {
		mavs::vehicle::Rp3dVehicle* vehicle = static_cast<mavs::vehicle::Rp3dVehicle *>(veh);
		return vehicle->GetTireSlip(tire_id);
	}

	EXPORT_CMD float GetRp3dTireAngularVelocity(mavs::vehicle::Vehicle* veh, int tire_id) {
		mavs::vehicle::Rp3dVehicle* vehicle = static_cast<mavs::vehicle::Rp3dVehicle *>(veh);
		return vehicle->GetTireAngularVelocity(tire_id);
	}

	EXPORT_CMD float GetRp3dLatAccel(mavs::vehicle::Vehicle* veh) {
		mavs::vehicle::Rp3dVehicle* vehicle = static_cast<mavs::vehicle::Rp3dVehicle *>(veh);
		return vehicle->GetLateralAcceleration();
	}

	EXPORT_CMD float GetRp3dLonAccel(mavs::vehicle::Vehicle* veh) {
		mavs::vehicle::Rp3dVehicle* vehicle = static_cast<mavs::vehicle::Rp3dVehicle *>(veh);
		return vehicle->GetLongitudinalAcceleration();
	}

	EXPORT_CMD mavs::vehicle::Vehicle* NewChronoVehicle() {
#ifdef USE_CHRONO
		mavs::vehicle::Vehicle* car = new mavs::vehicle::ChronoWheeledJson;
#else
		mavs::vehicle::Vehicle* car = NULL;
		std::cerr << "WARNING: PYTHON REQUESTED CHRONO VEHICLE, BUT CHRONO IS NOT BEING BUILT BY MAVS. CHECK CMAKE CONFIG" << std::endl;
#endif
		return car;
	}

	EXPORT_CMD void LoadMavsRp3dVehicle(mavs::vehicle::Vehicle* car, char* fname) {
		std::string veh_file(fname);
		mavs::vehicle::Rp3dVehicle* vehicle = static_cast<mavs::vehicle::Rp3dVehicle *>(car);
		vehicle->Load(veh_file);
	}

	EXPORT_CMD void SetMavsRp3dVehicleReloadVis(mavs::vehicle::Vehicle* car, bool reload) {
		mavs::vehicle::Rp3dVehicle* vehicle = static_cast<mavs::vehicle::Rp3dVehicle *>(car);
		vehicle->SetReloadVisualization(reload);
	}

	EXPORT_CMD void SetRp3dUseDrag(mavs::vehicle::Vehicle* veh, bool use_drag) {
		mavs::vehicle::Rp3dVehicle* vehicle = static_cast<mavs::vehicle::Rp3dVehicle *>(veh);
		vehicle->UseDragForces(use_drag);
	}

	EXPORT_CMD void LoadChronoVehicle(mavs::vehicle::Vehicle* car, char* fname) {
#ifdef USE_CHRONO
		std::string veh_file(fname);
		mavs::vehicle::ChronoWheeledJson* vehicle = static_cast<mavs::vehicle::ChronoWheeledJson *>(car);
		vehicle->Load(veh_file);
#endif
	}

	EXPORT_CMD float GetChronoTireNormalForce(mavs::vehicle::Vehicle* veh, int tire_id) {
#ifdef USE_CHRONO
		mavs::vehicle::ChronoWheeledJson* vehicle = static_cast<mavs::vehicle::ChronoWheeledJson *>(veh);
		return vehicle->GetTireTerrainForces(tire_id).z;
#else
		return 0.0;
#endif
	}


	EXPORT_CMD void DeleteMavsVehicle(mavs::vehicle::Vehicle* veh) {
		delete veh;
	}

	EXPORT_CMD void UpdateMavsVehicle(mavs::vehicle::Vehicle* veh, mavs::environment::Environment* env, float throttle, float steering, float brake, float dt) {
		veh->Update(env, throttle, steering, brake, dt);
	}

	EXPORT_CMD float * GetMavsVehicleFullState(mavs::vehicle::Vehicle* veh) {
		mavs::VehicleState vs = veh->GetState();
		// px,py,pz,qw,qx,qy,qz,vlx,vly,vlz,vax,vay,vaz,alx,aly,alz,aax,aay,aaz
		static float veh_state[19];
		veh_state[0] = vs.pose.position.x;
		veh_state[1] = vs.pose.position.y;
		veh_state[2] = vs.pose.position.z;
		veh_state[3] = vs.pose.quaternion.w;
		veh_state[4] = vs.pose.quaternion.x;
		veh_state[5] = vs.pose.quaternion.y;
		veh_state[6] = vs.pose.quaternion.z;
		veh_state[7] = vs.twist.linear.x;
		veh_state[8] = vs.twist.linear.y;
		veh_state[9] = vs.twist.linear.z;
		veh_state[10] = vs.twist.angular.x;
		veh_state[11] = vs.twist.angular.y;
		veh_state[12] = vs.twist.angular.z;
		veh_state[13] = vs.accel.linear.x;
		veh_state[14] = vs.accel.linear.y;
		veh_state[15] = vs.accel.linear.z;
		veh_state[16] = vs.accel.angular.x;
		veh_state[17] = vs.accel.angular.y;
		veh_state[18] = vs.accel.angular.z;
		return veh_state;
	}

	EXPORT_CMD float * GetMavsVehiclePosition(mavs::vehicle::Vehicle* veh) {
		glm::vec3 p = veh->GetPosition();
		static float position[3];
		position[0] = p.x;
		position[1] = p.y;
		position[2] = p.z;
		return position;
	}

	EXPORT_CMD float * GetMavsVehicleVelocity(mavs::vehicle::Vehicle* veh) {
		mavs::VehicleState state = veh->GetState();
		glm::vec3 v(state.twist.linear.x, state.twist.linear.y, state.twist.linear.z);
		static float velocity[3];
		velocity[0] = v.x;
		velocity[1] = v.y;
		velocity[2] = v.z;
		return velocity;
	}

	EXPORT_CMD float * GetMavsVehicleOrientation(mavs::vehicle::Vehicle* veh) {
		glm::quat q = veh->GetOrientation();
		static float orientation[4];
		orientation[0] = q.w;
		orientation[1] = q.x;
		orientation[2] = q.y;
		orientation[3] = q.z;
		return orientation;
	}

	EXPORT_CMD float GetMavsVehicleHeading(mavs::vehicle::Vehicle* veh) {
		glm::vec3 lt = veh->GetLookTo();
		float heading = (float)(atan2(lt.y, lt.x));
		return heading;
	}

	EXPORT_CMD float GetMavsVehicleSpeed(mavs::vehicle::Vehicle* veh) {
		mavs::VehicleState state = veh->GetState();
		float speed = veh->GetSpeed();
		//float speed = (float)(sqrt(state.twist.linear.x*state.twist.linear.x + state.twist.linear.y*state.twist.linear.y));
		return speed;
	}

	EXPORT_CMD void SetMavsVehiclePosition(mavs::vehicle::Vehicle* veh, float x, float y, float z) {
		veh->SetPosition(x, y, z);
	}

	EXPORT_CMD void SetMavsVehicleHeading(mavs::vehicle::Vehicle* veh, float theta) {
		float t = 0.5f*theta;
		veh->SetOrientation(cos(t), 0.0, 0.0, sin(t));
	}

	//--------------------- Vehicle controller methods -------------------------------------------------//
	EXPORT_CMD mavs::vehicle::PurePursuitController* NewVehicleController() {
		mavs::vehicle::PurePursuitController* controller = new mavs::vehicle::PurePursuitController;
		return controller;
	}

	EXPORT_CMD void DeleteVehicleController(mavs::vehicle::PurePursuitController* controller) {
		delete controller;
	}

	EXPORT_CMD void SetLooping(mavs::vehicle::PurePursuitController* controller) {
		controller->TurnOnLooping();
	}

	EXPORT_CMD void SetControllerDesiredSpeed(mavs::vehicle::PurePursuitController* controller, float speed) {
		controller->SetDesiredSpeed(speed);
	}

	EXPORT_CMD void SetControllerSpeedParams(mavs::vehicle::PurePursuitController* controller, float kp, float ki, float kd) {
		controller->SetSpeedControllerParams(kp,ki,kd);
	}

	EXPORT_CMD void SetControllerWheelbase(mavs::vehicle::PurePursuitController* controller, float wb) {
		controller->SetWheelbase(wb);
	}

	EXPORT_CMD void SetControllerMaxSteeringAngle(mavs::vehicle::PurePursuitController* controller, float max_angle) {
		controller->SetMaxSteering(max_angle);
	}

	EXPORT_CMD void SetControllerMinLookAhead(mavs::vehicle::PurePursuitController* controller, float min_la) {
		controller->SetMinLookAhead(min_la);
	}

	EXPORT_CMD void SetControllerMaxLookAhead(mavs::vehicle::PurePursuitController* controller, float max_la) {
		controller->SetMaxLookAhead(max_la);
	}

	EXPORT_CMD void SetControllerSteeringScale(mavs::vehicle::PurePursuitController* controller, float k) {
		controller->SetSteeringParam(k);
	}

	EXPORT_CMD float * GetControllerDrivingCommand(mavs::vehicle::PurePursuitController* controller, float dt) {
		static float driving_command[3];
		float throttle, steering, braking;
		controller->GetDrivingCommand(throttle, steering, braking, dt);
		driving_command[0] = throttle;
		driving_command[1] = steering;
		driving_command[2] = braking;
		return driving_command;
	}

	EXPORT_CMD void SetControllerDesiredPath(mavs::vehicle::PurePursuitController* controller, float *x, float *y, int np) {
		std::vector<glm::vec2> path;
		path.resize(np);
		for (int i = 0; i < np; i++) {
			path[i].x = x[i];
			path[i].y = y[i];
		}
		controller->SetDesiredPath(path);
	}

	EXPORT_CMD void SetControllerVehicleState(mavs::vehicle::PurePursuitController* controller, float px, float py, float speed, float heading) {
		controller->SetVehicleState(px, py, speed, heading);
	}

	EXPORT_CMD void ViewRp3dDebug(char* input_file_name){
		mavs::Rp3dVehicleViewer viewer;

		std::string fname(input_file_name);
		viewer.LoadVehicle(fname);

		while (viewer.IsOpen()) {
			viewer.Display(true);
		}
	}
	//--- Sensor base class functions ----------------------//
	EXPORT_CMD void SetMavsSensorPose(mavs::sensor::Sensor* sens, float position[3], float orientation[4]) {
		glm::vec3 p(position[0], position[1], position[2]);
		glm::quat q(orientation[0], orientation[1], orientation[2], orientation[3]);
		sens->SetPose(p, q);
	}

	EXPORT_CMD void SetMavsSensorRelativePose(mavs::sensor::Sensor* sens, float position[3], float orientation[4]) {
		glm::vec3 p(position[0], position[1], position[2]);
		glm::quat q(orientation[0], orientation[1], orientation[2], orientation[3]);
		sens->SetRelativePose(p, q);
	}

	EXPORT_CMD void DeleteMavsSensor(mavs::sensor::Sensor* sens) {
		delete sens;
	}

	EXPORT_CMD void UpdateMavsSensor(mavs::sensor::Sensor* sens, mavs::environment::Environment* env, float dt) {
		sens->Update(env, (double)dt);
	}

	EXPORT_CMD void DisplayMavsSensor(mavs::sensor::Sensor* sens) {
		sens->Display();
	}

	EXPORT_CMD float * GetSensorPose(mavs::sensor::Sensor* sens) {
		float * posebuff = new float[7];
		mavs::Pose pose = sens->GetPose();
		posebuff[0] = (float)pose.position.x;
		posebuff[1] = (float)pose.position.y;
		posebuff[2] = (float)pose.position.z;
		posebuff[3] = (float)pose.quaternion.w;
		posebuff[4] = (float)pose.quaternion.x;
		posebuff[5] = (float)pose.quaternion.y;
		posebuff[6] = (float)pose.quaternion.z;
		return posebuff;
	}

	EXPORT_CMD void DisplayMavsLidarPerspective(mavs::sensor::Sensor* sens, int im_width, int im_height) {
		mavs::sensor::lidar::Lidar* lidar = static_cast<mavs::sensor::lidar::Lidar *>(sens);
		lidar->DisplayPerspective(im_width, im_height);
	}

	EXPORT_CMD float GetChamferDistance(int npc1, float* pc1x, float* pc1y, float* pc1z, int npc2, float* pc2x, float* pc2y, float* pc2z) {
		std::vector<glm::vec3> pc1, pc2;
		pc1.resize(npc1);
		pc2.resize(npc2);
		for (int i = 0; i < npc1; i++) {
			pc1[i].x = pc1x[i];
			pc1[i].y = pc1y[i];
			pc1[i].z = pc1z[i];
		}
		for (int i = 0; i < npc2; i++) {
			pc2[i].x = pc2x[i];
			pc2[i].y = pc2y[i];
			pc2[i].z = pc2z[i];
		}
		float d = mavs::math::ChamferDistance(pc1, pc2);
		return d;
	}

	EXPORT_CMD void SetPointCloudColorType(mavs::sensor::Sensor* sens, char* type) {
		/// options are 'height', 'color', 'range', 'intensity', or 'white'
		mavs::sensor::lidar::Lidar* lidar = static_cast<mavs::sensor::lidar::Lidar *>(sens);
		std::string color_type(type);
		lidar->SetDisplayColorType(color_type);
	}

	EXPORT_CMD void SaveMavsSensorAnnotation(mavs::sensor::Sensor* sens, mavs::environment::Environment* env, char* ofname) {
		std::string fname(ofname);
		sens->AnnotateFrame(env, true);
		sens->SaveAnnotation(fname);
	}

	EXPORT_CMD void SaveMavsCameraAnnotationFull(mavs::sensor::Sensor* sens, mavs::environment::Environment* env, char* ofname) {
		std::string fname(ofname);
		mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(sens);
		camera->SetDrawAnnotations();
		//camera->AnnotateFrame(env, false);
		camera->AnnotateFull(env);
		camera->SaveAnnotation(fname);
		camera->SaveBoxAnnotationsCsv(fname+".csv");
	}

	EXPORT_CMD void AnnotateMavsSensorFrame(mavs::sensor::Sensor* sens, mavs::environment::Environment* env) {
		sens->AnnotateFrame(env, true);
	}

	EXPORT_CMD void SaveMavsSensorRaw(mavs::sensor::Sensor* sens) {
		sens->SaveRaw();
	}

	EXPORT_CMD void SaveMavsLidarImage(mavs::sensor::Sensor* sens, char* fname) {
		std::string outfile(fname);
		mavs::sensor::lidar::Lidar* lidar = static_cast<mavs::sensor::lidar::Lidar *>(sens);
		lidar->WritePointsToImage(outfile);
	}

	EXPORT_CMD void SaveProjectedMavsLidarImage(mavs::sensor::Sensor* sens, char* fname) {
		std::string outfile(fname);
		mavs::sensor::lidar::Lidar* lidar = static_cast<mavs::sensor::lidar::Lidar *>(sens);
		lidar->WriteProjectedLidarImage(outfile);
	}

	//--- RTK functions ---------------------------------------------------////
	EXPORT_CMD mavs::sensor::Sensor* NewMavsRtk() {
		mavs::sensor::rtk::Rtk* rtk = new mavs::sensor::rtk::Rtk;
		mavs::sensor::Sensor * sens = rtk;
		return sens;
	}

	EXPORT_CMD void SetRtkError(mavs::sensor::Sensor* sens, float error) {
		mavs::sensor::rtk::Rtk* rtk = static_cast<mavs::sensor::rtk::Rtk *>(sens);
		rtk->SetError(error);
	}

	EXPORT_CMD void SetRtkDroputRate(mavs::sensor::Sensor* sens, float droput_rate) {
		mavs::sensor::rtk::Rtk* rtk = static_cast<mavs::sensor::rtk::Rtk *>(sens);
		rtk->SetDropoutRate(droput_rate);
	}

	EXPORT_CMD void SetRtkWarmupTime(mavs::sensor::Sensor* sens, float warmup_time) {
		mavs::sensor::rtk::Rtk* rtk = static_cast<mavs::sensor::rtk::Rtk *>(sens);
		rtk->SetWarmupTime(warmup_time);
	}

	EXPORT_CMD float * GetRtkPosition(mavs::sensor::Sensor* sens) {
		mavs::sensor::rtk::Rtk* rtk = static_cast<mavs::sensor::rtk::Rtk *>(sens);
		static float position[3];
		mavs::Odometry odom = rtk->GetOdometryMessage();
		position[0] = (float)odom.pose.pose.position.x;
		position[1] = (float)odom.pose.pose.position.y;
		position[2] = (float)odom.pose.pose.position.z;
		return position;
	}

	EXPORT_CMD float * GetRtkOrientation(mavs::sensor::Sensor* sens) {
		mavs::sensor::rtk::Rtk* rtk = static_cast<mavs::sensor::rtk::Rtk *>(sens);
		static float orientation[4];
		mavs::Odometry odom = rtk->GetOdometryMessage();
		orientation[0] = (float)odom.pose.pose.quaternion.w;
		orientation[1] = (float)odom.pose.pose.quaternion.x;
		orientation[2] = (float)odom.pose.pose.quaternion.y;
		orientation[3] = (float)odom.pose.pose.quaternion.z;
		return orientation;
	}
	//--- Done with RTK functions ----------------------------------------///


	EXPORT_CMD void SaveMavsRadarImage(mavs::sensor::Sensor* sens, char* fname) {
		std::string outfile(fname);
		mavs::sensor::radar::Radar* radar = static_cast<mavs::sensor::radar::Radar *>(sens);
		radar->SaveImage(outfile);
	}

	EXPORT_CMD float * GetRadarReturnLocations(mavs::sensor::Sensor* sens) {
		mavs::sensor::radar::Radar* radar = static_cast<mavs::sensor::radar::Radar *>(sens);
		std::vector<mavs::RadarTarget> targets = radar->GetDetectedTargets();
		int ntargets = (int)targets.size();
		float * points = new float[2 * ntargets];
		for (int i = 0; i < ntargets; i++) {
			points[i] = targets[i].position_x;
			points[i + ntargets] = targets[i].position_y;
		}
		return points;
	}

	EXPORT_CMD int GetRadarNumTargets(mavs::sensor::Sensor* sens) {
		mavs::sensor::radar::Radar* radar = static_cast<mavs::sensor::radar::Radar *>(sens);
		std::vector<mavs::RadarTarget> targets = radar->GetDetectedTargets();
		int ntargets = (int)targets.size();
		return ntargets;
	}

	EXPORT_CMD float * GetRadarTargets(mavs::sensor::Sensor* sens) {
		mavs::sensor::radar::Radar* radar = static_cast<mavs::sensor::radar::Radar *>(sens);
		std::vector<mavs::RadarTarget> targets = radar->GetDetectedTargets();

		int np = (int)(10 * targets.size());	
		if (np <= 0)return NULL;
		//float * target_data = new float(np);
		std::vector<float> target_data;
		target_data.resize(np);
		int nn = 0;
		for (int i = 0; i < targets.size(); i++) {
			target_data[nn] = targets[i].id;
			nn++;
			target_data[nn] = targets[i].status;
			nn++;
			target_data[nn] = targets[i].range;
			nn++;
			target_data[nn] = targets[i].range_rate;
			nn++;
			target_data[nn] = targets[i].range_accleration;
			nn++;
			target_data[nn] = targets[i].angle;
			nn++;
			target_data[nn] = targets[i].width;
			nn++;
			target_data[nn] = targets[i].lateral_rate;
			nn++;
			target_data[nn] = targets[i].position_x;
			nn++;
			target_data[nn] = targets[i].position_y;
			nn++;
		}
		return target_data.data();
	}

	EXPORT_CMD void SaveMavsCameraImage(mavs::sensor::Sensor* sens, char* fname) {
		std::string outfile(fname);
		mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(sens);
		camera->SaveImage(outfile);
	}


	EXPORT_CMD void SetMavsPathTracerCameraNormalization(mavs::sensor::Sensor* sens, char* norm_type) {
		std::string nt(norm_type);
		mavs::sensor::camera::PathTracerCamera* camera = static_cast<mavs::sensor::camera::PathTracerCamera *>(sens);
		camera->SetNormalizationType(nt);
	}

	EXPORT_CMD void SetMavsPathTracerFixPixels(mavs::sensor::Sensor* sens, bool fix) {
		mavs::sensor::camera::PathTracerCamera* camera = static_cast<mavs::sensor::camera::PathTracerCamera *>(sens);
		if (fix) {
			camera->TurnOnPixelSmoothing();
		}
		else {
			camera->TurnOffPixelSmoothing();
		}
	}

	EXPORT_CMD void FreeCamera(mavs::sensor::Sensor* sens) {
		mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(sens);
		camera->FreePose();
	}

	EXPORT_CMD float * GetDrivingCommandFromCamera(mavs::sensor::Sensor* sens) {
		mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(sens);
		static float driving_command[3];
		std::vector<bool> dc = camera->GetKeyCommands();
		driving_command[0] = 0.0f; // throttle;
		driving_command[1] = 0.0f; // steering;
		driving_command[2] = 0.0f; // braking;
		if (dc[0]) driving_command[0] = 1.0f;
		if (dc[1]) driving_command[2] = 1.0f;
		if (dc[2]) driving_command[1] = 1.0f;
		if (dc[3]) driving_command[1] = -1.0f;
		return driving_command;
	}

	EXPORT_CMD mavs::sensor::Sensor* NewMavsRgbCameraDimensions(int num_hor,
		int num_vert, float focal_array_width, float focal_array_height,
		float focal_length) {
		mavs::sensor::camera::Camera* camera = new mavs::sensor::camera::RgbCamera;
		camera->Initialize(num_hor, num_vert, focal_array_width, focal_array_height,
			focal_length);
		camera->SetPixelSampleFactor(1);
		camera->SetAntiAliasing("oversampled");
		camera->SetRenderShadows(false);
		mavs::sensor::Sensor * sens = camera;
		return sens;
	}

EXPORT_CMD mavs::sensor::Sensor* NewMavsPathTraceCameraExplicit(int nx, int ny, float h_size, float v_size, float flen, float gamma, int num_rays, int max_depth, float rr) {
	mavs::sensor::camera::PathTracerCamera *cam = new mavs::sensor::camera::PathTracerCamera;
	cam->Initialize(nx, ny, h_size, v_size, flen);
	cam->SetGamma(gamma);
	cam->SetNumIterations(num_rays);
	cam->SetMaxDepth(max_depth);
	cam->SetRRVal(rr);
	return cam;
}

EXPORT_CMD mavs::sensor::Sensor* NewMavsPathTraceCamera(char* type, int num_rays, int max_depth, float rr) {
	std::string model(type);
	if (model == "low") {
		return new mavs::sensor::camera::MachineVisionPathTraced(num_rays, max_depth, rr);
	}
	else if (model == "high") {
		return new mavs::sensor::camera::HDPathTraced(num_rays, max_depth, rr);
	}
	else if (model == "half") {
		return new mavs::sensor::camera::HalfHDPathTraced(num_rays, max_depth, rr);
	}
	else if (model == "uav") {
		return new mavs::sensor::camera::UavCameraPathTraced(num_rays, max_depth, rr);
	}
	else if (model == "uavlow") {
		return new mavs::sensor::camera::UavCameraPathTracedLowRes(num_rays, max_depth, rr);
	}
	else if (model == "phantom4") {
		return new mavs::sensor::camera::Phantom4CameraPathTraced(num_rays, max_depth, rr);
	}
	else if (model == "phantom4low") {
		return new mavs::sensor::camera::Phantom4CameraPathTracedLowRes(num_rays, max_depth, rr);
	}
	else if (model == "sf3325") {
		return new mavs::sensor::camera::Sf3325PathTraced(num_rays, max_depth, rr);
	}
	else if (model == "sf3325low") {
		return new mavs::sensor::camera::Sf3325PathTracedLow(num_rays, max_depth, rr);
	}
	else {
		return new mavs::sensor::camera::MachineVisionPathTraced(num_rays, max_depth, rr);
	}
	
}
/*EXPORT_CMD mavs::sensor::Sensor* NewMavsPathTraceCameraLowRes(int num_rays, int max_depth, float rr) {
	return new mavs::sensor::camera::MachineVisionPathTraced(num_rays, max_depth, rr);
}

EXPORT_CMD mavs::sensor::Sensor* NewMavsPathTraceCameraHalfHighRes(int num_rays, int max_depth, float rr) {
	return new mavs::sensor::camera::HalfHDPathTraced(num_rays, max_depth, rr);
}

EXPORT_CMD mavs::sensor::Sensor* NewMavsPathTraceCameraHighRes(int num_rays, int max_depth, float rr) {
	return new mavs::sensor::camera::HDPathTraced(num_rays, max_depth, rr);
}*/

EXPORT_CMD mavs::sensor::Sensor* NewMavsRedEdge() {
	mavs::sensor::Sensor *camera = new mavs::sensor::camera::MicaSenseRedEdge;
	return camera;
}

EXPORT_CMD void SaveRedEdge(mavs::sensor::Sensor* sensor, char* fname) {
	mavs::sensor::camera::MicaSenseRedEdge* camera = static_cast<mavs::sensor::camera::MicaSenseRedEdge *>(sensor);
	std::string imname(fname);
	camera->SaveImage(imname);
}

EXPORT_CMD void SaveRedEdgeBands(mavs::sensor::Sensor* sensor, char* fname) {
	mavs::sensor::camera::MicaSenseRedEdge* camera = static_cast<mavs::sensor::camera::MicaSenseRedEdge *>(sensor);
	std::string imname(fname);
	camera->SaveBands(imname);
}

EXPORT_CMD void SaveRedEdgeFalseColor(mavs::sensor::Sensor* sensor, int band1, int band2, int band3, char* fname) {
	mavs::sensor::camera::MicaSenseRedEdge* camera = static_cast<mavs::sensor::camera::MicaSenseRedEdge *>(sensor);
	std::string imname(fname);
	camera->SaveFalseColor(band1-1, band2-1, band3-1, imname);
}

EXPORT_CMD void DisplayRedEdge(mavs::sensor::Sensor* sensor) {
	mavs::sensor::camera::MicaSenseRedEdge* camera = static_cast<mavs::sensor::camera::MicaSenseRedEdge *>(sensor);
	camera->Display();
}

EXPORT_CMD mavs::sensor::Sensor* NewMavsCameraModel(char* model_name) {
	std::string model(model_name);
	mavs::sensor::Sensor *camera;
	if (model == "XCD-V60") {
		camera = new mavs::sensor::camera::XCD_V60;
	}
	else if (model == "Flea") {
		camera = new mavs::sensor::camera::Flea3_4mm;
	}
	else if (model == "HD1080") {
		camera = new mavs::sensor::camera::HD1080;
	}
	else if (model == "MachineVision") {
		camera = new mavs::sensor::camera::MachineVision;
	}
	else if (model == "MachineVisionPathTraced") {
		camera = new mavs::sensor::camera::MachineVisionPathTraced(1000,10,0.55f);
	}
	else if (model == "Sf3325") {
		camera = new mavs::sensor::camera::Sf3325;
	}
	else if (model == "Sf3325PathTraced") {
		camera = new mavs::sensor::camera::Sf3325PathTraced(20, 10, 0.55f);
	}
	else if (model == "UavCamera") {
		camera = new mavs::sensor::camera::UavCameraLowRes;
	}
	else if (model == "UavCameraPathTraced") {
		camera = new mavs::sensor::camera::UavCameraPathTraced(20, 10, 0.55f);
	}
	else if (model == "UavCameraPathTracedLow") {
		camera = new mavs::sensor::camera::UavCameraPathTracedLowRes(10, 10, 0.55f);
	}
	else if (model == "HDPathTraced") {
		camera = new mavs::sensor::camera::HDPathTraced(1500, 15, 0.55f);
	}
	else if (model == "Phantom4Camera") {
		camera = new mavs::sensor::camera::Phantom4Camera;
	}
	else if (model == "Phantom4CameraPathTraced") {
		camera = new mavs::sensor::camera::Phantom4CameraPathTraced(20, 10, 0.55f);
	}
	else if (model == "Phantom4CameraPathTracedLow") {
		camera = new mavs::sensor::camera::Phantom4CameraPathTracedLowRes(10, 10, 0.55f);
	}
	else if (model == "HalfHDPathTraced") {
		camera = new mavs::sensor::camera::HalfHDPathTraced(1500, 15, 0.55f);
	}
	else {
		camera = new mavs::sensor::camera::LowRes;
	}
	return camera;
}

EXPORT_CMD void ConvertToRccb(mavs::sensor::Sensor* sens) {
	mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(sens);
	std::string cam_type = camera->GetCameraType();
	if (cam_type == "rgb" || cam_type == "rccb") {
		mavs::sensor::camera::RgbCamera* rgb_camera = static_cast<mavs::sensor::camera::RgbCamera *>(sens);
		rgb_camera->ConvertImageToRccb();
		sens = camera;
	}
}

EXPORT_CMD float * GetCameraBuffer(mavs::sensor::Sensor* sens) {
	mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(sens);
	float *buffer = camera->GetImageBuffer();
	return buffer;
}

EXPORT_CMD int GetCameraBufferSize(mavs::sensor::Sensor* sens) {
	mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(sens);
	int buff_size = camera->GetBufferSize(); 
	return buff_size;
}

EXPORT_CMD int GetCameraBufferWidth(mavs::sensor::Sensor* sens) {
	mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(sens);
	return camera->GetWidth();
}

EXPORT_CMD int GetCameraBufferHeight(mavs::sensor::Sensor* sens) {
	mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(sens);
	return camera->GetHeight();
}

EXPORT_CMD int GetCameraBufferDepth(mavs::sensor::Sensor* sens) {
	mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(sens);
	return camera->GetBufferSize() / (camera->GetHeight()*camera->GetWidth());
}

EXPORT_CMD void SetMavsCameraShadows(mavs::sensor::Sensor* sens, bool shadows) {
	mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(sens);
	camera->SetRenderShadows(shadows);
	sens = camera;
}

EXPORT_CMD void SetMavsCameraBlur(mavs::sensor::Sensor* sens, bool blur) {
	mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera*>(sens);
	camera->UseBlur(blur);
	sens = camera;
}

EXPORT_CMD void SetMavsCameraLensDrops(mavs::sensor::Sensor* sens, bool onlens) {
	mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(sens);
	camera->SetRaindropsOnLens(onlens);
	sens = camera;
}

EXPORT_CMD void SetMavsCameraEnvironmentProperties(mavs::sensor::Sensor* sens, mavs::environment::Environment* env) {
	mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(sens);
	std::string camera_type = camera->GetCameraType();
	if (camera_type == "rgb" || camera_type == "rccb") {
		mavs::sensor::camera::RgbCamera* rgb_camera = static_cast<mavs::sensor::camera::RgbCamera *>(sens);
		rgb_camera->SetEnvironmentProperties(env);
		sens = camera;
	}
}

EXPORT_CMD void SetMavsCameraAntiAliasingFactor(mavs::sensor::Sensor* sens, int fac) {
	mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(sens);
	camera->SetPixelSampleFactor(fac);
	sens = camera;
}

EXPORT_CMD void SetMavsCameraElectronics(mavs::sensor::Sensor* sens, float gamma, float gain) {
	mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(sens);
	camera->SetGamma(gamma);
	camera->SetGain(gain);
	sens = camera;
}

EXPORT_CMD void SetMavsCameraTempAndSaturation(mavs::sensor::Sensor* sens, float temp, float sat) {
	mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(sens);
	camera->SetSaturationAndTemperature(sat,temp);
	sens = camera;
}

EXPORT_CMD float GetCameraGamma(mavs::sensor::Sensor* sens) {
	mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(sens);
	float gamma = camera->GetGamma();
	return gamma;
}

EXPORT_CMD float GetCameraGain(mavs::sensor::Sensor* sens) {
	mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(sens);
	float gain = camera->GetGain();
	return gain;
}

EXPORT_CMD void AddRainToExistingImage(char* image_name, float rain_rate, bool drops_on_lens) {
	std::string imagefile(image_name);
	cimg_library::CImg<float> image;
	image.load(imagefile.c_str());

	mavs::sensor::camera::AddRainToImage(image, rain_rate,drops_on_lens);

	std::string outfile = "raining_";
	outfile.append(imagefile);
	image.save(outfile.c_str());
}

EXPORT_CMD void AddRainToExistingImageRho(char* image_name, float rain_rate, float rho, bool drops_on_lens) {
	std::string imagefile(image_name);
	cimg_library::CImg<float> image;
	image.load(imagefile.c_str());

	mavs::sensor::camera::AddRainToImage(image, rain_rate, rho, drops_on_lens);

	std::string outfile = "raining_";
	outfile.append(imagefile);
	image.save(outfile.c_str());
}

// ---- MEMS sensor functions ------------------------------------------//
EXPORT_CMD mavs::sensor::imu::MemsSensor* NewMavsMems(char* memstype) {
	mavs::sensor::imu::MemsSensor* mems;
	std::string type(memstype);
	if (type == "gyro") {
		mems = new mavs::sensor::imu::Gyro;
	}
	else if (type == "accelerometer") {
		mems = new mavs::sensor::imu::Accelerometer;
	}
	else if (type == "magnetometer") {
		mems = new mavs::sensor::imu::Magnetometer;
	}
	else {
		mems = new mavs::sensor::imu::Accelerometer;
	}
	return mems;
}

EXPORT_CMD void SetMemsMeasurmentRange(mavs::sensor::imu::MemsSensor* sens, float range) {
	sens->SetMeasurmentRange(range);
}

EXPORT_CMD void SetMemsResolution(mavs::sensor::imu::MemsSensor* sens, float res) {
	sens->SetResolution(res);
}

EXPORT_CMD void SetMemsConstantBias(mavs::sensor::imu::MemsSensor* sens, float bias_x, float bias_y, float bias_z) {
	sens->SetConstantBias(glm::vec3(bias_x, bias_y, bias_z));
}

EXPORT_CMD void SetMemsNoiseDensity(mavs::sensor::imu::MemsSensor* sens, float nd_x, float nd_y, float nd_z) {
	sens->SetNoiseDensity(glm::vec3(nd_x, nd_y, nd_z));
}

EXPORT_CMD void SetMemsBiasInstability(mavs::sensor::imu::MemsSensor* sens, float bi_x, float bi_y, float bi_z) {
	sens->SetBiasInstability(glm::vec3(bi_x, bi_y, bi_z));
}

EXPORT_CMD void SetMemsAxisMisalignment(mavs::sensor::imu::MemsSensor* sens, float misalign_x, float misalign_y, float misalign_z) {
	sens->SetAxisMisalignment(glm::vec3(misalign_x, misalign_y, misalign_z));
}

EXPORT_CMD void SetMemsRandomWalk(mavs::sensor::imu::MemsSensor* sens, float rw_x, float rw_y, float rw_z) {
	sens->SetRandomWalk(glm::vec3(rw_x, rw_y, rw_z));
}

EXPORT_CMD void SetMemsTemperatureBias(mavs::sensor::imu::MemsSensor* sens, float tb_x, float tb_y, float tb_z) {
	sens->SetTemperatureBias(glm::vec3(tb_x, tb_y, tb_z));
}

EXPORT_CMD void SetMemsTemperatureScaleFactor(mavs::sensor::imu::MemsSensor* sens, float tsf_x, float tsf_y, float tsf_z) {
	sens->SetTemperatureScaleFactor(glm::vec3(tsf_x, tsf_y, tsf_z));
}

EXPORT_CMD void SetMemsAccelerationBias(mavs::sensor::imu::MemsSensor* sens, float ab_x, float ab_y, float ab_z) {
	sens->SetAccelerationBias(glm::vec3(ab_x, ab_y, ab_z));
}

EXPORT_CMD float * MemsUpdate(mavs::sensor::imu::MemsSensor* sens, float input_x, float input_y, float input_z, float temperature, float sample_rate) {
	float * returned_accel = new float[3];
	glm::vec3 a = sens->Update(glm::vec3(input_x, input_y, input_z), temperature, sample_rate);
	for (int i = 0; i < 3; i++)returned_accel[i] = a[i];
	return returned_accel;
}
// ---------------- Done with MEMS functions -----------------------------------------------------------------------------------------------

EXPORT_CMD bool CameraDisplayOpen(mavs::sensor::Sensor* sens) {
	mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera*>(sens);
	bool is_open = camera->DisplayOpen();
	return is_open;
}

EXPORT_CMD void SetOakDCameraDisplayType(mavs::sensor::Sensor* sens, char* display_type) {
	std::string disp_type(display_type);
	mavs::sensor::camera::OakDCamera* camera = static_cast<mavs::sensor::camera::OakDCamera*>(sens);
	camera->SetDisplayType(disp_type);
}

EXPORT_CMD mavs::sensor::Sensor* NewMavsOakDCamera() {
	mavs::sensor::Sensor* cam = new mavs::sensor::camera::OakDCamera();
	return cam;
}

EXPORT_CMD float* GetOakDDepthBuffer(mavs::sensor::Sensor* sens) {
	mavs::sensor::camera::OakDCamera* camera = static_cast<mavs::sensor::camera::OakDCamera*>(sens);
	float* buffer = camera->GetRangeBuffer();
	return buffer;
}

EXPORT_CMD float* GetOakDImageBuffer(mavs::sensor::Sensor* sens) {
	mavs::sensor::camera::OakDCamera* camera = static_cast<mavs::sensor::camera::OakDCamera*>(sens);
	float* buffer = camera->GetImageBuffer();
	return buffer;
}

EXPORT_CMD mavs::sensor::Sensor* GetOakDCamera(mavs::sensor::Sensor* sens) {
	mavs::sensor::camera::OakDCamera* oakd = static_cast<mavs::sensor::camera::OakDCamera*>(sens);
	mavs::sensor::Sensor* cam = oakd->GetCamera();
	return cam;
}

EXPORT_CMD int GetOakDDepthBufferSize(mavs::sensor::Sensor* sens) {
	mavs::sensor::camera::OakDCamera* camera = static_cast<mavs::sensor::camera::OakDCamera*>(sens);
	return camera->GetRangeBufferSize();
}

EXPORT_CMD int GetOakDImageBufferSize(mavs::sensor::Sensor* sens) {
	mavs::sensor::camera::OakDCamera* camera = static_cast<mavs::sensor::camera::OakDCamera*>(sens);
	return camera->GetBufferSize();
}

EXPORT_CMD float GetOakDMaxRangeCm(mavs::sensor::Sensor* sens) {
	mavs::sensor::camera::OakDCamera* camera = static_cast<mavs::sensor::camera::OakDCamera*>(sens);
	float max_range_cm = camera->GetMaxRangeCm();
	return max_range_cm;
}

EXPORT_CMD void SetOakDMaxRangeCm(mavs::sensor::Sensor* sens, float max_range_cm) {
	mavs::sensor::camera::OakDCamera* camera = static_cast<mavs::sensor::camera::OakDCamera*>(sens);
	camera->SetMaxDepth(max_range_cm / 100.0f);
}

//--- Constructors for specific sensors ------------------------------//
EXPORT_CMD mavs::sensor::Sensor* NewMavsRgbCamera() {
	mavs::sensor::Sensor* cam = new mavs::sensor::camera::RgbCamera();
	return cam;
}

EXPORT_CMD mavs::sensor::Sensor* NewMavsLwirCamera(int nx, int ny, float dx, float dy, float flen) {
	mavs::sensor::Sensor* cam = new mavs::sensor::camera::LwirCamera(nx, ny, dx, dy, flen);
	return cam;
}

EXPORT_CMD void LoadLwirThermalData(mavs::sensor::Sensor *sens, char* fname) {
	mavs::sensor::camera::LwirCamera* cam = static_cast<mavs::sensor::camera::LwirCamera *>(sens);
	std::string to_load(fname);
	cam->LoadThermalData(to_load);
}

EXPORT_CMD mavs::sensor::Sensor* NewMavsRadar() {
	mavs::sensor::Sensor* radar = new mavs::sensor::radar::Radar();
	return radar;
}

EXPORT_CMD void SetRadarMaxRange(mavs::sensor::Sensor* sens, float max_range) {
	mavs::sensor::radar::Radar* radar = static_cast<mavs::sensor::radar::Radar *>(sens);
	radar->SetMaxRange(max_range);
	sens = radar;
}

EXPORT_CMD void SetRadarFieldOfView(mavs::sensor::Sensor* sens, float fov, float res_degrees) {
	mavs::sensor::radar::Radar* radar = static_cast<mavs::sensor::radar::Radar *>(sens);
	radar->Initialize(fov, 1.0f, res_degrees);
	sens = radar;
}

EXPORT_CMD void SetRadarSampleResolution(mavs::sensor::Sensor* sens, float res_degrees) {
	mavs::sensor::radar::Radar* radar = static_cast<mavs::sensor::radar::Radar *>(sens);
	radar->SetSampleResolution(res_degrees);
	sens = radar;
}

mavs::sensor::lidar::LidarTools pc_analyzer;

EXPORT_CMD void AnalyzeCloud(mavs::sensor::Sensor* sens, char* fname, int frame_num, bool display) {
	std::string outfile(fname);
	pc_analyzer.DisplayLidarImage(display);
	mavs::sensor::lidar::Lidar* lidar = static_cast<mavs::sensor::lidar::Lidar *>(sens);
	std::vector<mavs::sensor::lidar::labeled_point> points = lidar->GetLabeledPoints();
	pc_analyzer.AnalyzeCloud(points, outfile, frame_num, lidar);
}

EXPORT_CMD void SetMavsSensorVelocity(mavs::sensor::Sensor* sens, float vx, float vy, float vz) {
	sens->SetVelocity(vx, vy, vz);
}

EXPORT_CMD void WriteMavsLidarToColorizedCloud(mavs::sensor::Sensor* sens, char* fname) {
	std::string outfile(fname);
	mavs::sensor::lidar::Lidar* lidar = static_cast<mavs::sensor::lidar::Lidar *>(sens);
	lidar->WriteColorizedRegisteredPointsToText(outfile);
}

EXPORT_CMD void WriteMavsLidarToLabeledPcdWithNormals(mavs::sensor::Sensor* sens, char* fname) {
	std::string outfile(fname);
	mavs::sensor::lidar::Lidar* lidar = static_cast<mavs::sensor::lidar::Lidar *>(sens);
	lidar->WritePcdWithNormalsAndLabels(outfile);
}

EXPORT_CMD void WriteMavsLidarToLabeledPcd(mavs::sensor::Sensor* sens, char* fname) {
	std::string outfile(fname);
	mavs::sensor::lidar::Lidar* lidar = static_cast<mavs::sensor::lidar::Lidar *>(sens);
	lidar->WritePcdWithLabels(outfile);
}

EXPORT_CMD void WriteMavsLidarToPcd(mavs::sensor::Sensor* sens, char* fname) {
	std::string outfile(fname);
	mavs::sensor::lidar::Lidar* lidar = static_cast<mavs::sensor::lidar::Lidar *>(sens);
	lidar->WritePcd(outfile);
}


EXPORT_CMD void WriteMavsLidarToLabeledCloud(mavs::sensor::Sensor* sens, char* fname) {
	std::string outfile(fname);
	mavs::sensor::lidar::Lidar* lidar = static_cast<mavs::sensor::lidar::Lidar *>(sens);
	lidar->WriteLabeledRegisteredPointsToText(outfile);
}

EXPORT_CMD void AddPointsToImage(mavs::sensor::Sensor* lidar_in, mavs::sensor::Sensor* camera_in) {
	mavs::sensor::lidar::Lidar* lidar = static_cast<mavs::sensor::lidar::Lidar *>(lidar_in);
	mavs::sensor::camera::Camera* camera = static_cast<mavs::sensor::camera::Camera *>(camera_in);
	std::vector<glm::vec4> points = lidar->GetRegisteredPointsXYZI();
	camera->AddLidarPointsToImage(points);
}

EXPORT_CMD int GetMavsLidarNumberPoints(mavs::sensor::Sensor* sens) {
	mavs::sensor::lidar::Lidar* lidar = static_cast<mavs::sensor::lidar::Lidar *>(sens);
	int np = (int)lidar->GetNumPoints();
	return np;
}

EXPORT_CMD float * GetMavsLidarRegisteredPoints(mavs::sensor::Sensor* sens) {
	//size of retured points should be 3*npoints
	mavs::sensor::lidar::Lidar* lidar = static_cast<mavs::sensor::lidar::Lidar *>(sens);
	std::vector<glm::vec4> points = lidar->GetRegisteredPointsXYZI();
	int npoints = (int)points.size();
	//std::vector<float> returned_points;
	//returned_points.resize(npoints * 3, 0.0f);
	float * returned_points = new float[npoints * 3];
	int np = npoints * 3;
	for (int i = 0; i < (int)points.size(); i++) {
		int n = i * 3;
		if ((n + 2) < np) {
			if (points[i].w > 0.0) {
				returned_points[n] = points[i].x;
				returned_points[n + 1] = points[i].y;
				returned_points[n + 2] = points[i].z;
			}
			else {
				returned_points[n] = 0.0f;
				returned_points[n + 1] = 0.0f;
				returned_points[n + 2] = 0.0f;
			}
		}
	}

	return returned_points; 
}

EXPORT_CMD float * GetMavsLidarUnRegisteredPointsXYZIL(mavs::sensor::Sensor* sens) {
	mavs::sensor::lidar::Lidar* lidar = static_cast<mavs::sensor::lidar::Lidar *>(sens);
	//std::vector<glm::vec4> points = lidar->GetPointsXYZI();
	std::vector<mavs::sensor::lidar::labeled_point> points = lidar->GetLabeledPoints();
	int npoints = (int)points.size();
	int np = npoints*5;
	float * returned_points = new float[np];
	//std::vector<float> returned_points;
	//returned_points.resize(npoints * 5, 0.0f);
	for (int i = 0; i < (int)points.size(); i++) {
		int n = i * 5;
		if (n + 4 < np) {
			if (points[i].intensity > 0.0) {
				returned_points[n] = points[i].x;
				returned_points[n + 1] = points[i].y;
				returned_points[n + 2] = points[i].z;
				returned_points[n + 3] = points[i].intensity;
				returned_points[n + 4] = (float)points[i].label;
			}
			else {
				returned_points[n] = 0.0f;
				returned_points[n + 1] = 0.0f;
				returned_points[n + 2] = 0.0f;
				returned_points[n + 3] = 0.0f;
				returned_points[n + 4] = 0.0f;
			}
		}
	}
	return returned_points;
}

EXPORT_CMD float * GetMavsLidarUnRegisteredPointsXYZI(mavs::sensor::Sensor* sens) {
	//size of retured points should be 3*npoints
	mavs::sensor::lidar::Lidar* lidar = static_cast<mavs::sensor::lidar::Lidar *>(sens);
	std::vector<glm::vec4> points = lidar->GetPointsXYZI();
	int npoints = (int)points.size();
	//int n = 0;
	// * returned_points = new float[npoints * 4];
	std::vector<float> returned_points;
	returned_points.resize(npoints * 4, 0.0f);
	for (int i = 0; i < (int)points.size(); i++) {
		int n = i * 4;
		if (n + 3 < returned_points.size()) {
			if (points[i].w > 0.0) {
				returned_points[n] = points[i].x;
				returned_points[n + 1] = points[i].y;
				returned_points[n + 2] = points[i].z;
				returned_points[n + 3] = points[i].w;
			}
			else {
				returned_points[n] = 0.0f;
				returned_points[n + 1] = 0.0f;
				returned_points[n + 2] = 0.0f;
				returned_points[n + 3] = 0.0f;
			}
		}
	}
	return returned_points.data();
}

EXPORT_CMD mavs::sensor::Sensor* MavsLidarSetScanPattern(float horiz_fov_low, float horiz_fov_high,float horiz_resolution,
	float vert_fov_low, float vert_fov_high,float vert_resolution) {
	mavs::sensor::lidar::Lidar *lidar = new mavs::sensor::lidar::Lidar;
	lidar->SetScanPattern(horiz_fov_low, horiz_fov_high, horiz_resolution, vert_fov_low, vert_fov_high, vert_resolution);
	mavs::sensor::Sensor* sens = lidar;
	return sens;
}

EXPORT_CMD mavs::sensor::Sensor* NewMavsLidar(char* senstype) {
	std::string model(senstype);
	mavs::sensor::Sensor* lidar;
	float rep_rate = 10.0;
	if (model == "M8") {
		lidar = new mavs::sensor::lidar::MEight(rep_rate);
	}
	else if (model == "HDL-64E") {
		lidar = new mavs::sensor::lidar::Hdl64ESimple(rep_rate);
	}
	else if (model == "HDL-32E") {
		lidar = new mavs::sensor::lidar::Hdl32E;
	}
	else if (model == "VLP-16") {
		lidar = new mavs::sensor::lidar::Vlp16(rep_rate);
	}
	else if (model == "AnvelApiLidar") {
		lidar = new mavs::sensor::lidar::AnvelApiLidar(rep_rate);
	}
	else if (model == "FourPi") {
		lidar = new mavs::sensor::lidar::FourPiLidar;
	}
	else if (model == "LMS-291") {
		lidar = new mavs::sensor::lidar::Lms291_S05;
	}
	else if (model == "OS1") {
		lidar = new mavs::sensor::lidar::OusterOS1;
	}
	else if (model == "OS1-16") {
		lidar = new mavs::sensor::lidar::OusterOS1_16;
	}
	else if (model == "OS2") {
		lidar = new mavs::sensor::lidar::OusterOS2;
	}
	else if (model == "RS32") {
		lidar = new mavs::sensor::lidar::Rs32;
	}
	else if (model == "OS0") {
		mavs::sensor::lidar::Lidar* new_lidar = new mavs::sensor::lidar::Lidar;
		new_lidar->SetBeamSpotCircular((float)(0.09f*mavs::kDegToRad));
		new_lidar->SetMaxRange(50.0f);
		new_lidar->SetRotationRate(10.0f);
		float res = 360.0f / (1024.0f);
		float vlo = -45.0f;
		float vhi = 45.0f;
		float vres = (vhi-vlo) / 31.0f;
		new_lidar->SetScanPattern(-180.0f, 180.0f - res, res, vlo, vhi, vres);
		new_lidar->SetTimeStep(0.1f);
		lidar = new_lidar;
	}
	else if (model == "BPearl") {
		mavs::sensor::lidar::Lidar* new_lidar = new mavs::sensor::lidar::Lidar;
		new_lidar->SetBeamSpotCircular((float)(0.09f*mavs::kDegToRad));
		new_lidar->SetMaxRange(100.0f);
		new_lidar->SetRotationRate(10.0f);
		float res = 360.0f / (1800.0f);
		float vlo = 2.31f;
		float vhi = 89.5f;
		float vres = (vhi-vlo) / 31.0f;
		new_lidar->SetScanPattern(-180.0f, 180.0f - res, res, vlo, vhi, vres);
		new_lidar->SetTimeStep(0.1f);
		lidar = new_lidar;
	}
	else {
		lidar = new mavs::sensor::lidar::Hdl32E;
	}
	return lidar;
}

EXPORT_CMD mavs::Waypoints * LoadAnvelReplayFile(char* fname) {
	std::string infile(fname);
	mavs::Waypoints * wp = new mavs::Waypoints;
	wp->CreateFromAnvelVprp(infile);
	return wp;
}

EXPORT_CMD mavs::Waypoints * LoadWaypointsFromJson(char* fname) {
	std::string infile(fname);
	mavs::Waypoints * wp = new mavs::Waypoints;
	wp->Load(infile);
	return wp;
}

EXPORT_CMD void DeleteMavsWaypoints(mavs::Waypoints * wp) {
	delete wp;
}

EXPORT_CMD void SaveWaypointsAsJson(mavs::Waypoints *wp, char* fname) {
	std::string json_name(fname);
	wp->SaveAsJson(json_name);
}

EXPORT_CMD int GetNumWaypoints(mavs::Waypoints * wp) {
	return (int)wp->NumWaypoints();
}

EXPORT_CMD float * GetWaypoint(mavs::Waypoints * wp, int wpnum) {
	glm::vec2 p = wp->GetWaypoint(wpnum);
	static float current_gotten_waypoint[3];
	current_gotten_waypoint[0] = p.x;
	current_gotten_waypoint[1] = p.y;
	current_gotten_waypoint[2] = 0.0f;
	return current_gotten_waypoint;
}

EXPORT_CMD mavs::OrthoViewer * CreateOrthoViewer() {
	mavs::OrthoViewer * viewer = new mavs::OrthoViewer;
	return viewer;
}

EXPORT_CMD void DeleteOrthoViewer(mavs::OrthoViewer *viewer) {
	delete viewer;
}

EXPORT_CMD void UpdateOrthoViewerWaypoints(mavs::OrthoViewer *viewer, mavs::environment::Environment* env, mavs::Waypoints *wp) {
	std::vector<glm::vec2> waypoints;
	for (int i = 0; i < wp->NumWaypoints(); i++) {
		waypoints.push_back(wp->GetWaypoint(i));
	}
	viewer->Update(env, waypoints);
}

EXPORT_CMD void UpdateOrthoViewer(mavs::OrthoViewer *viewer, mavs::environment::Environment* env) {
	viewer->Update(env);
}

EXPORT_CMD void DisplayOrtho(mavs::OrthoViewer *viewer) {
	viewer->Display();
}

EXPORT_CMD void SaveOrtho(mavs::OrthoViewer *viewer, char *fname) {
	std::string sname(fname);
	viewer->SaveImage(sname);
}

EXPORT_CMD float * GetOrthoBuffer(mavs::OrthoViewer *viewer) {
	float *buffer = viewer->GetImageBuffer();
	return buffer;
}

EXPORT_CMD int GetOrthoBufferSize(mavs::OrthoViewer *viewer) {
	int sz = viewer->GetImageBufferSize();
	return sz;
}

// -------------- Rp3d Vehicle Viewer ---------------------------------------------------//
EXPORT_CMD mavs::Rp3dVehicleViewer * CreateRp3dViewer() {
	mavs::Rp3dVehicleViewer * viewer = new mavs::Rp3dVehicleViewer;
	return viewer;
}

EXPORT_CMD void DeleteRp3dViewer(mavs::Rp3dVehicleViewer *viewer) {
	delete viewer;
}

EXPORT_CMD void Rp3dViewerLoadVehicle(mavs::Rp3dVehicleViewer *viewer, char *vehfile) {
	std::string vf(vehfile);
	viewer->LoadVehicle(vf);
}

EXPORT_CMD void Rp3dViewerDisplay(mavs::Rp3dVehicleViewer *viewer, bool show_debug) {
	viewer->Display(show_debug);
}

EXPORT_CMD void Rp3dViewerUpdate(mavs::Rp3dVehicleViewer *viewer, bool show_debug) {
	viewer->Update(show_debug);
}

EXPORT_CMD float * Rp3dViewerGetSideImage(mavs::Rp3dVehicleViewer *viewer) {
	return viewer->GetSideBuffer();
}

EXPORT_CMD float * Rp3dViewerGetFrontImage(mavs::Rp3dVehicleViewer *viewer) {
	return viewer->GetFrontBuffer();
}

EXPORT_CMD void Rp3dViewerSaveSideImage(mavs::Rp3dVehicleViewer *viewer, char *fname) {
	std::string fn(fname);
	viewer->WriteSideImage(fn);
}

EXPORT_CMD void Rp3dViewerSaveFrontImage(mavs::Rp3dVehicleViewer *viewer, char *fname) {
	std::string fn(fname);
	viewer->WriteFrontImage(fn);
}

EXPORT_CMD int Rp3dViewerGetSideImageSize(mavs::Rp3dVehicleViewer *viewer) {
	return viewer->GetSideBufferSize();
}

EXPORT_CMD int Rp3dViewerGetFrontImageSize(mavs::Rp3dVehicleViewer *viewer) {
	return viewer->GetFrontBufferSize();
}

//-------------- Material viewer functions ---------------------------------------------//
EXPORT_CMD mavs::raytracer::MtlViewer * CreateMaterialViewer() {
	mavs::raytracer::MtlViewer * viewer = new mavs::raytracer::MtlViewer;
	return viewer;
}

EXPORT_CMD void DeleteMaterialViewer(mavs::raytracer::MtlViewer *viewer) {
	delete viewer;
}

EXPORT_CMD void UpdateMaterialViewer(mavs::raytracer::MtlViewer *viewer) {
	viewer->Update();
}

EXPORT_CMD void ResetMaterialViewer(mavs::raytracer::MtlViewer *viewer, float kd_r, float kd_g, float kd_b) {
	mavs::Material material;
	material.kd.r = kd_r;
	material.kd.g = kd_g;
	material.kd.b = kd_b;
	viewer->SetMaterial(material);
}

EXPORT_CMD void LoadMaterialViewerMesh(mavs::raytracer::MtlViewer *viewer, char* meshfile) {
	std::string mf(meshfile);
	viewer->LoadMesh(mf);
}

EXPORT_CMD int GetMaterialViewerNumMats(mavs::raytracer::MtlViewer *viewer) {
	return viewer->GetNumMats();
}

EXPORT_CMD char * GetMaterialViewerMatName(mavs::raytracer::MtlViewer *viewer, int id) {
	return viewer->GetMatName(id);
}
EXPORT_CMD char * GetMaterialViewerSpectrumName(mavs::raytracer::MtlViewer *viewer, int id) {
	return viewer->GetSpectrumName(id);
}

EXPORT_CMD float * GetMaterialViewerMaterial(mavs::raytracer::MtlViewer *viewer, int id) {
	mavs::Material mat = viewer->GetMaterial(id);
	static float material[7];
	glm::vec3 color = mat.kd;
	if (color.r == 1.0f && color.g == 1.0f && color.b == 1.0f &&
		  !(mat.ka.r==0.0f && mat.ka.g==0.0f && mat.ka.b==0.0f)) {
		color = mat.ka;
	}
	material[0] = color.r;
	material[1] = color.g;
	material[2] = color.b;
	material[3] = mat.ks.r;
	material[4] = mat.ks.g;
	material[5] = mat.ks.b;
	material[6] = mat.ns;
	return material;
}

#ifdef __cplusplus
}
#endif


#endif
