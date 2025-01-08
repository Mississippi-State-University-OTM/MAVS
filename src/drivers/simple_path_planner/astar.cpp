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
#include <drivers/simple_path_planner/astar.h>

#include <queue>
#include <limits>
#include <cmath>
#include <fstream>
#include <iostream>

#include <simple_path_planner/astar_node.h>

namespace mavs{
namespace driver{

Astar::Astar(){

}

Astar::~Astar(){

}

MapIndex Astar::FoldIndex(int n){
  MapIndex c;
  c.i = n%width_;
  c.j = (int)(floor( (1.0*n)/(1.0*width_) ));
  return c;
}

void Astar::AllocateMap(int h, int w, int init_val){
  height_ = h;
  width_ = w;
  weights_.clear();
  weights_ .resize(height_*width_,init_val);
  paths_ .clear();
  paths_ .resize(height_*width_,-1);
}

void Astar::SetMapValue(int i, int j, int val){
  weights_[FlattenIndex(i,j)]=val;
}

// manhattan distance: requires each move to cost >= 1
float Astar::Heuristic(int i0, int j0, int i1, int j1) {
  //manhattan distance
  //return std::abs(i0 - i1) + std::abs(j0 - j1);
  //straight line distance
  int x = i1-i0;
  int y = j1-j0;
  return (float)sqrt(x*x+y*y);
}

void Astar::WriteMap(){
  std::ofstream fout;
  fout.open("map.txt");
  for (int i=0;i<width_;i++){
    for (int j=0;j<height_;j++){
      int n = FlattenIndex(i,j);
      fout<<i<<" "<<j<<" "<<weights_[n]<<std::endl;
    }
  }
  fout.close();
}

void Astar::WritePath(){
  std::ofstream fout;
  fout.open("path.txt");
  for (int n =0; n<(int)path_.size(); n++){
    fout<<path_[n].i<<" "<<path_[n].j<<std::endl;
  }
  fout.close();
}

bool Astar::SolvePotential(){
  int window = 4;
  std::vector<int> potential;
  potential.resize(weights_.size(),0);
  for (int i=0;i<width_;i++){
    for (int j=0;j<height_;j++){
      double summ = 0;
      for (int ii=i-window;ii<i+window;ii++){
	for (int jj=j-window;jj<j+window;jj++){
	  if (ii>=0 && ii<width_ && jj>=0 && jj<height_){
	      double w = (weights_[FlattenIndex(ii,jj)])/
		(1.0+sqrt((i-ii)*(i-ii)+(j-jj)*(j-jj)));
	      if (w>summ)summ=w;
	    }
	}
      }
      if (summ>100) summ =100;
      potential[FlattenIndex(i,j)] = (int)summ;
    }
  }
  weights_ = potential;
  bool solved = Solve();
  return solved;
}


bool Astar::Solve() {
  std::fill(paths_.begin(), paths_.end(),-1);
  
  const float INF = std::numeric_limits<float>::infinity();

  Node start_node(start_, 0.);
  Node goal_node(goal_, 0.);

  float* costs = new float[height_ * width_];
  for (int i = 0; i < height_ * width_; ++i)
    costs[i] = INF;
  costs[start_] = 0.;

  std::priority_queue<Node> nodes_to_visit;
  nodes_to_visit.push(start_node);

  int* nbrs = new int[4];

  bool solution_found = false;
  while (!nodes_to_visit.empty()) {
    // .top() doesn't actually remove the node
    Node cur = nodes_to_visit.top();

    if (cur == goal_node) {
      solution_found = true;
      break;
    }

    nodes_to_visit.pop();

    // check bounds and find up to four neighbors
    nbrs[0] = (cur.idx / width_ > 0) ? (cur.idx - width_) : -1;
    nbrs[1] = (cur.idx % width_ > 0) ? (cur.idx - 1) : -1;
    nbrs[2] = (cur.idx / width_ + 1 < height_) ? (cur.idx + width_) : -1;
    nbrs[3] = (cur.idx % width_ + 1 < width_) ? (cur.idx + 1) : -1;
    for (int i = 0; i < 4; ++i) {
      if (nbrs[i] >= 0) {
        // the sum of the cost so far and the cost of this move
        float new_cost = costs[cur.idx] + weights_[nbrs[i]];
        if (new_cost < costs[nbrs[i]]) {
          costs[nbrs[i]] = new_cost;
          float priority = new_cost + Heuristic(nbrs[i] / width_,
                                                nbrs[i] % width_,
                                                goal_ / width_,
                                                goal_ % width_);
          // paths with lower expected cost are explored first
          nodes_to_visit.push(Node(nbrs[i], priority));
          paths_[nbrs[i]] = cur.idx;
        }
      }
    }
  }

  delete[] costs;
  delete[] nbrs;

  if (solution_found)ExtractPath();
  
  return solution_found;
}

void Astar::ExtractPath(){
  path_.clear();
  int path_idx = goal_;
  while (path_idx!=start_){
    MapIndex c = FoldIndex(path_idx);
    path_.push_back(c);
    path_idx = paths_[path_idx];
  }
}

} //namespace driver
} //namespace mavs
