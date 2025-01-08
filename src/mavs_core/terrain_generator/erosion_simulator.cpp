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
#include <mavs_core/terrain_generator/erosion_simulator.h>

#include <iostream>
#include <fstream>
#include <algorithm>

#include <mavs_core/math/constants.h>
#include <mavs_core/math/utils.h>

namespace mavs {
namespace terraingen {

ErosionSimulator::ErosionSimulator() {

}

ErosionSimulator::~ErosionSimulator() {

}

ErosionSimulator::ErosionSimulator(RandomSurface &surface) {
	Initialize(surface);
}

void ErosionSimulator::Initialize(RandomSurface &surface) {
	/*
	save_debug_ = false;
	nsteps_ = 0;
	dt_ = 0.0001f;
	elapsed_time_ = 0.0f;
	rainfall_rate_ = 1.0f;
	sed_capacity_const_ = 0.25f;
	sed_deposition_const_ = 0.25f;
	sed_dissolve_const_ = 0.25f;
	water_evap_constant_ = 0.25f;

	glm::ivec2 dim = surface.GetMapDimensions();
	nx_ = dim.x;
	ny_ = dim.y;
	std::vector<ErosionCell> row;
	row.resize(ny_);
	terrain_.resize(nx_, row);
	for (int i = 0; i < nx_; i++) {
		for (int j = 0; j < ny_; j++) {
			terrain_[i][j].terrain_height = surface.GetCellHeight(i, j);
			terrain_[i][j].water_height = 0.0f;
			terrain_[i][j].flux_bottom = 0.0f;
			terrain_[i][j].flux_top = 0.0f;
			terrain_[i][j].flux_left = 0.0f;
			terrain_[i][j].flux_right = 0.0f;
			terrain_[i][j].slope = 0.0f;
			terrain_[i][j].susp_sediment = 0.0f;
			terrain_[i][j].water_source = 0.0f;
			terrain_[i][j].water_velocity = glm::vec2(0.0f, 0.0f);
		}
	}
	pipe_length_ = 2.0*surface.GetResolution();
	CalculateSlopes();
	*/
}

void ErosionSimulator::CalculateSlopes() {
	glm::vec3 z_hat(0.0f, 0.0f, 1.0f);
	for (int i = 0; i < nx_; i++) {
		for (int j = 0; j < ny_; j++) {
			int i_lo = std::max(i - 1, 0);
			int i_hi = std::min(i + 1, nx_ - 1);
			int j_lo = std::max(j - 1, 0);
			int j_hi = std::min(j + 1, ny_ - 1);
			float dx = (i_hi - i_lo)*pipe_length_;
			float dy = (j_hi - j_lo)*pipe_length_;
			float dh_x = terrain_[i_hi][j].terrain_height - terrain_[i_lo][j].terrain_height;
			float dh_y = terrain_[i][j_hi].terrain_height - terrain_[i][j_lo].terrain_height;
			float dhdx = dh_x / dx;
			float dhdy = dh_y / dy;
			glm::vec3 normal(-dhdx, -dhdy, 1.0f);
			normal = glm::normalize(normal);
			float costheta = glm::dot(normal, z_hat);
			terrain_[i][j].slope = acos(costheta); // sqrt(dhdx*dhdx + dhdy * dhdy);
		}
	}
}

std::vector<std::vector< float> > ErosionSimulator::GetHeightMap() {
	std::vector<float> row;
	row.resize(ny_, 0.0f);
	std::vector< std::vector<float> > hm;
	hm.resize(nx_, row);
	for (int i = 0; i < nx_; i++) {
		for (int j = 0; j < ny_; j++) {
			hm[i][j] = terrain_[i][j].terrain_height;
		}
	}
	return hm;
}

void ErosionSimulator::Step() {

	if (save_debug_) {
		std::cout << "Simulating erosion step " << nsteps_ << " "<<elapsed_time_<<std::endl;
		WriteCellData();
	}
	elapsed_time_ += dt_;

	//create intermediate variables
	std::vector<float> row;
	row.resize(ny_, 0.0f);
	std::vector< std::vector<float> > d1,d2,s1;
	d1.resize(nx_, row);
	d2.resize(nx_, row);
	s1.resize(nx_, row);

	//increment water sources (rain and rivers)
	for (int i = 0; i < nx_; i++) {
		for (int j = 0; j < ny_; j++) {
			d1[i][j] = terrain_[i][j].water_height + terrain_[i][j].water_source + rainfall_rate_*dt_;
		}
	}

	//calculate outflow fluxes
	float flux_const = (float)(dt_ * pipe_length_*kGravity);
	float l2 = pipe_length_ * pipe_length_;
	for (int i = 0; i < nx_; i++) {
		for (int j = 0; j < ny_; j++) {
			//get height differences in each direction
			float dij = terrain_[i][j].terrain_height + d1[i][j];
			float dh_left = 0.0f;
			if (i > 0)dh_left =  dij - terrain_[i - 1][j].terrain_height - d1[i - 1][j];
			float dh_right = 0.0f;
			if (i < nx_-1)dh_right = dij - terrain_[i + 1][j].terrain_height - d1[i + 1][j];
			float dh_bottom = 0.0f;
			if (j > 0)dh_bottom = dij - terrain_[i][j-1].terrain_height - d1[i][j-1];
			float dh_top = 0.0f;
			if (j < ny_-1)dh_top = dij - terrain_[i][j + 1].terrain_height - d1[i][j + 1];

			//calculate fluxes
			float fl = std::max(0.0f, terrain_[i][j].flux_left + flux_const * dh_left);
			float fr = std::max(0.0f, terrain_[i][j].flux_right + flux_const * dh_right);
			float fb = std::max(0.0f, terrain_[i][j].flux_bottom + flux_const * dh_bottom);
			float ft = std::max(0.0f, terrain_[i][j].flux_top + flux_const * dh_top);

			//scale the fluxes
			float k = std::min(1.0f, d1[i][j] * l2 / (dt_*(fl + fr + fb + ft)));
			terrain_[i][j].flux_left = k * fl;
			terrain_[i][j].flux_right = k * fr;
			terrain_[i][j].flux_bottom = k * fb;
			terrain_[i][j].flux_top = k * ft;

			//boundary conditions
			if (i == 0 ) terrain_[i][j].flux_left = 0.0f;
			if (i == nx_ - 1) terrain_[i][j].flux_right = 0.0f;
			if (j == 0 ) terrain_[i][j].flux_bottom = 0.0f;
			if (j == ny_ - 1) terrain_[i][j].flux_top = 0.0f;
		}
	}

	float vx_max = 0.0f;
	float vy_max = 0.0f;
	//water surface and velocity update due to fluxes
	for (int i = 0; i < nx_; i++) {
		for (int j = 0; j < ny_; j++) {
			int i_lo = std::max(0, i - 1);
			int i_hi = std::min(nx_ - 1, i + 1);
			int j_lo = std::max(0, j - 1);
			int j_hi = std::min(ny_ - 1, j + 1);
			float dv = dt_ * (terrain_[i_lo][j].flux_right + terrain_[i_hi][j].flux_left +
				terrain_[i][j_lo].flux_top + terrain_[i][j_hi].flux_bottom -
				terrain_[i][j].flux_right - terrain_[i][j].flux_left -
				terrain_[i][j].flux_top - terrain_[i][j].flux_bottom);
			d2[i][j] = d1[i][j] + dv / l2;
			float dbar = 0.5f*(d1[i][j] + d2[i][j]);
			float dwx = terrain_[i_lo][j].flux_right - terrain_[i][j].flux_left + terrain_[i][j].flux_right - terrain_[i_hi][j].flux_left;
			float dwy = terrain_[i][j_lo].flux_top - terrain_[i][j].flux_bottom + terrain_[i][j].flux_top - terrain_[i][j_hi].flux_bottom;
			terrain_[i][j].water_velocity.x = 0.5f*dwx / (pipe_length_*dbar);
			terrain_[i][j].water_velocity.y = 0.5f*dwy / (pipe_length_*dbar);
			//boundary conditions
			if (i == 0) terrain_[i][j].water_velocity.x = terrain_[1][j].water_velocity.x;
			if (i == nx_ - 1) terrain_[i][j].water_velocity.x= terrain_[nx_ - 2][j].water_velocity.x;
			if (j == 0) terrain_[i][j].water_velocity.y = terrain_[i][1].water_velocity.y;
			if (j == ny_ - 1) terrain_[i][j].water_velocity.y = terrain_[i][ny_ - 2].water_velocity.y;
			//max velocities for CFL calculation
			float vx = fabs(terrain_[i][j].water_velocity.x);
			float vy = fabs(terrain_[i][j].water_velocity.y);
			if (vx > vx_max)vx_max = vx;
			if (vy > vy_max)vy_max = vy;
		}
	}

	//update erosion and deposition
	float ks = sed_dissolve_const_;
	float kd = sed_deposition_const_;
	for (int i = 0; i < nx_; i++) {
		for (int j = 0; j < ny_; j++) {
			float vmag = glm::length(terrain_[i][j].water_velocity);
			float slopefac = (float)std::max((float)sin(terrain_[i][j].slope), 0.001f);
			float c = sed_capacity_const_ *slopefac*vmag;
			float st = terrain_[i][j].susp_sediment;
			if (c > st) {
				float fac = ks * (c - st);
				terrain_[i][j].terrain_height = terrain_[i][j].terrain_height - fac;
				s1[i][j] = st + fac;
			}
			else {
				float fac = kd * (st - c);
				terrain_[i][j].terrain_height = terrain_[i][j].terrain_height + fac;
				s1[i][j] = st - fac;
			}
		}
	}

	//transport sediment
	float dtdl = dt_ / pipe_length_;
	float dl = 0.00001f;
	for (int i = 0; i < nx_; i++) {
		for (int j = 0; j < ny_; j++) {
			float di = (terrain_[i][j].water_velocity.x*dtdl);
			int di_lo = (int)floor(di);
			int di_hi = (int)ceil(di);
			float wi_lo = 1.0f / (di - floor(di) + dl);
			float wi_hi = 1.0f / (ceil(di)-di + dl);
			int i_lo = std::min(std::max(0,i - di_lo),nx_-1);
			int i_hi = std::min(std::max(0, i - di_hi), nx_ - 1);
			float dj = (terrain_[i][j].water_velocity.y*dtdl);
			int dj_lo = (int)floor(dj);
			int dj_hi = (int)ceil(dj);
			float wj_lo = 1.0f / (dj - floor(dj) + dl);
			float wj_hi = 1.0f / (ceil(dj) - dj + dl);
			int j_lo = std::min(std::max(0, j - dj_lo), ny_ - 1);
			int j_hi = std::min(std::max(0, j - dj_hi), ny_ - 1);
			terrain_[i][j].susp_sediment = (s1[i_lo][j_lo]*wi_lo*wj_lo + s1[i_lo][j_hi]*wi_lo*wj_hi + 
				s1[i_hi][j_lo]*wi_hi*wj_lo + s1[i_hi][j_hi]*wi_hi*wj_hi)/(wi_lo*wj_lo + wi_lo*wj_hi +
					wi_hi*wj_lo + wi_hi*wj_hi) ;
		}
	}

	//perform evaportation
	float evap_const = (1.0f - water_evap_constant_ * dt_);
	for (int i = 0; i < nx_; i++) {
		for (int j = 0; j < ny_; j++) {
			terrain_[i][j].water_height = d2[i][j] * evap_const;
		}
	}
	for (int i = 0; i < nx_; i++) {
		for (int j = 0; j < ny_; j++) {
			if (i == 0) terrain_[i][j].water_height = terrain_[1][j].water_height;
			if (i == nx_ - 1) terrain_[i][j].water_height = terrain_[nx_-2][j].water_height;
			if (j == 0) terrain_[i][j].water_height = terrain_[i][1].water_height;
			if (j == ny_ - 1) terrain_[i][j].water_height = terrain_[i][ny_-2].water_height;
		}
	}


	//recalculate the slopes for the next step
	CalculateSlopes();

	//determine the time step for the next step
	// based on CFL condition
	float dtx = 0.5f*pipe_length_ / vx_max;
	float dty = 0.5f*pipe_length_ / vy_max;
	dt_ = std::min(1.0f,std::min(dtx, dty));

	nsteps_++;
}

void ErosionSimulator::Erode(int num_years) {
	while (elapsed_time_ < num_years) {
		Step();
	}
}

void ErosionSimulator::WriteCellData() {
	std::string debug_data("erode_data_");
	debug_data.append(utils::ToString(nsteps_));
	debug_data.append(".txt");
	std::ofstream fout(debug_data.c_str());
	for (int i = 0; i < nx_; i++) {
		for (int j = 0; j < ny_; j++) {
			ErosionCell c = terrain_[i][j];
			fout << i << " " << j << " " << 
				c.terrain_height << " " << //3
				c.water_height << " " << //4 
				c.susp_sediment << " " << //5 
				c.flux_top << " " << //6 
				c.flux_bottom << " " << //7
				c.flux_right << " " << //8 
				c.flux_left << " " << //9
				c.water_velocity.x << " " << //10 
				c.water_velocity.y << " " << //11
				c.slope << " " <<  //12
				c.water_source << //13
				std::endl;
		}
	}
	fout.close();
}


} //namespace terraingen
} //namespace mavs