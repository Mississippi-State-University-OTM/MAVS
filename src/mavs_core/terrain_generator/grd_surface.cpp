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
#include <mavs_core/terrain_generator/grd_surface.h>
#include <string>
#include <limits>
#include <iostream>
#include <fstream>

namespace mavs {
namespace terraingen {

GridSurface::GridSurface(){
	nodata_value_ = -9999;
}

GridSurface::GridSurface(HeightMap &hm) {
	nodata_value_ = -9999;
	heightmap_ = hm;
}

GridSurface::~GridSurface(){}

void GridSurface::LoadAscFile(std::string grdName){
	std::ifstream fid;
	fid.open(grdName.c_str());

	int ncols, nrows;
	float xllcorner, yllcorner, cellsize;

	std::string key;
	fid >> key >> ncols;
	fid >> key >> nrows;
	fid >> key >> xllcorner;
	fid >> key >> yllcorner;
	fid >> key >> cellsize;
	fid >> key >> nodata_value_;

	glm::vec2 ll_corner(xllcorner, yllcorner);
	glm::vec2 ur_corner = ll_corner; 
	ur_corner.x += float(nrows) * cellsize;
	ur_corner.y += float(ncols) * cellsize;

	heightmap_.SetCorners(ll_corner.x, ll_corner.y, ur_corner.x, ur_corner.y);
	heightmap_.SetResolution(cellsize);
	heightmap_.Resize(nrows, ncols);

	if (heightmap_.GetHorizontalDim() != nrows || heightmap_.GetVerticalDim() != ncols) {
		std::cerr << "WARNING, the .asc file dimensions were " << nrows << " x " << ncols << ", but " <<
			heightmap_.GetHorizontalDim() << " x " << heightmap_.GetVerticalDim() << " were allocated." << std::endl;
	}

	for (int i=0;i<nrows;i++){
		for (int j=0;j<ncols;j++){
			float z;
			fid >> z;
			heightmap_.SetHeight(i, j, z);
		}
	}
}

void GridSurface::WriteElevToGrd(std::string output_file){
	std::ofstream fout;
	fout.open(output_file.c_str());
	fout<<"ncols "<<heightmap_.GetHorizontalDim()<<std::endl;
	fout<<"nrows "<<heightmap_.GetVerticalDim()<< std::endl;
	fout<<"xllcorner "<<heightmap_.GetLLCorner().x<< std::endl;
	fout<<"yllcorner "<<heightmap_.GetLLCorner().y<< std::endl;
	fout<<"cellsize "<<heightmap_.GetResolution()<< std::endl;
	fout<<"NODATA_value "<<nodata_value_<<std::endl;
	for (int i=0;i<(int)heightmap_.GetHorizontalDim();i++){
		for (int j=0;j<(int)heightmap_.GetVerticalDim();j++){
			fout << heightmap_.GetCellHeight(i, j) << " ";
		}
		fout<< std::endl;
	}
	fout.close();
}

void GridSurface::WriteToPointsFile(std::string filename){
	std::ofstream fout;
	fout.open(filename.c_str());

	float xllcorner = heightmap_.GetLLCorner().x;
	float yllcorner = heightmap_.GetLLCorner().y;
	float cellsize = heightmap_.GetResolution();
	int nx = (int)heightmap_.GetHorizontalDim();
	for (int i=0;i<(int)heightmap_.GetHorizontalDim();i++){
		for (int j=0;j<(int)heightmap_.GetVerticalDim();j++){
			fout<<i<<" "<<j<<" "<<yllcorner + j*cellsize<<" "<<xllcorner + (nx-i)*cellsize<<" "<<heightmap_.GetCellHeight(i,j)<<std::endl;
		}
	}
	fout.close();
}

GridSurface GridSurface::TrimCell(int trim) {
	GridSurface cell;
	cell = GetSubCell((int)heightmap_.GetHorizontalDim()-2*trim,(int)heightmap_.GetVerticalDim()-2*trim,trim,trim);
	return cell;
}

void GridSurface::Smooth(float thresh) {
	heightmap_.CleanHeights(thresh);
}

void GridSurface::RemoveNoDataCells() {
	heightmap_.ReplaceCellsByValue(nodata_value_);
}

GridSurface GridSurface::GetSubCell(int xsize, int ysize, int LLx, int LLy){
	int nx = (int)heightmap_.GetHorizontalDim();
	int ny = (int)heightmap_.GetVerticalDim();

	if ( (LLx + xsize)>nx || (LLy+ysize)>ny || LLx<0 || LLy<0 || xsize<0 || ysize<0){
		std::cerr<<"ERROR: the trim value is too large or negative."<<std::endl;
		exit(1);
	}
	
	GridSurface cell;
	float res = heightmap_.GetResolution();
	glm::vec2 ll_old = heightmap_.GetLLCorner();
	glm::vec2 ll_new(ll_old.x + LLx * res, ll_old.y + LLy * res);
	glm::vec2 ur_new(ll_new.x + xsize * res, ll_new.y + ysize * res);
	cell.SetDimensions(ll_new.x, ll_new.y, ur_new.x, ur_new.y, res);
	cell.SetNoDataValue(nodata_value_);
	cell.Resize(xsize, ysize);
	int ii = 0;
	for (int i = LLx; i<(LLx+xsize); i++){
		int jj = 0;
		for (int j=LLy; j<(LLy+ysize); j++) {
			cell.SetHeightAtPoint(ii, jj, heightmap_.GetCellHeight(i, j));
			jj++;
		}
		ii++;
	}

	return cell;
}

void GridSurface::AddPoint(float x, float y, float z){
	heightmap_.AddPoint(x, y, z);
}

} //namespace terraingen
} //namespace mavs