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
/**
 * \file astar.h
 * C++ API based on the simple A* algorithm from GitHub
 * https://github.com/hjweide/a-star
 * Original License: MIT license
 */

#ifndef ASTAR_H
#define ASTAR_H

#include <vector>

#include <glm/glm.hpp>

namespace mavs{
namespace driver{

///Index into the A* map
struct MapIndex{
  int i;
  int j;
};

/**
 * Astar map class with solve functions and map accessor methods.
 * The map holds obstacle values from 0 to 100. 0=no obstacle, 100=impassable.
 */
class Astar{
 public:
  /// Constructor
  Astar();

  /// Destructor
  ~Astar();
  
  /**
   * Allocate memory for the map and initialize
   * \param height Height of the map, in cells
   * \param width Width of the map, in cells
   * \param init_val Initial value for all cells, from 0 to 100
   */
  void AllocateMap(int height, int width, int init_val);

  /**
   * Set the value of cell (i,j).
   * \param i Vertical index of the cell to set
   * \param j Horizontal index of the cell to set
   * \param val Value to set, [0,100]
   */
  void SetMapValue(int i, int j, int val);

  /**
   * Returns the map value of cell (i,j).
   * \param i Vertical index of cell to get
   * \param j Horizontal index of cell to get
   */
  int GetMapValue(int i, int j){return weights_[FlattenIndex(i,j)];}

  /**
   * Sets cell (i,j) as the goal point.
   * \param i Vertical index of goal cell
   * \param j Horizontal index of goal cell
   */
  void SetGoal(int i, int j){goal_ = FlattenIndex(i,j);}

  /**
   * Sets cell (i,j) as the current location of the vehicle
   * \param i Vertical index of the vehicle location
   * \param j Horizontal index of the vehicle location
   */
  void SetStart(int i, int j){start_ = FlattenIndex(i,j);}

  /**
   * Solve the A* map. Returns true if a path was found.
   */
  bool Solve();

  /**
   * Solve the A* map, buffering obstacles with a "potential" like field.
   * Makes algorithm slower, but prevents vehicle from driving too close to 
   * obstacles. Returns true if a path was found.
   */
  bool SolvePotential();

  /// Write the map to a text file.
  void WriteMap();

  /// Write the calculated path to a text file.
  void WritePath();
  
  /// Return a list of indices specifying the current path.
  std::vector<MapIndex> GetPath(){return path_;}
  
  /// Returns the number of horizontal cells.
  int GetMapWidth(){return width_;}

  /// Returns the number of vertical cells.
  int GetMapHeight(){return height_;}

 private:
  MapIndex FoldIndex(int n);
  
  int FlattenIndex(int i, int j){return j*width_+i;}
  
  /// Heuristic
  float Heuristic(int i0, int j0, int i1, int j1);

  /// Flattened occupancy grid
  std::vector<int> weights_;

  ///height of the grid
  int height_;

  ///width of the grid
  int width_;

  ///flattened index of the goal point
  int goal_;

  ///flattened index of the start point
  int start_;

  /// calculated path
  std::vector<int> paths_;

  std::vector<MapIndex> path_;

  void ExtractPath();
};

} //namespace driver
} //namespace mavs

#endif
