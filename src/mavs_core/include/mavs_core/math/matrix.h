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
 * \class Matrix
 * 
 * A matrix class that supplies what glm was lacking, namely arbitrarily sized
 * matrices with operations such as transpose and inverse.
 *
 * \author Chris Goodin
 *
 * \date 12/13/2017
 */
#ifndef MATRIX_H
#define MATRIX_H

#include <vector>
#include <iostream>

namespace mavs{
namespace math{

class Matrix {
 public: 
  ///Declare an empty matrix.
  Matrix();
  
  ///Declare an nrows x ncols matrix
  Matrix(int nrows, int ncols);

  ~Matrix();

  Matrix (const Matrix &m);
  
  /// Return the transpose of a matrix.
  Matrix Transpose();

  /// Return the minor of element ij.
  Matrix GetMinor(int i, int j);

  /**
   * Returns the inverse of a matrix. If the inverse does not exist, returns
   * a matrix of zeros with the same dimensions as the original.
   */
  Matrix Inverse();

  /// Returns the determinant of a matrix.
  double Determinant();
 
  /// Resizes a matrix to nrows x ncols and sets all elements to zero.
  void Resize(int nrows, int ncols);

  /// Returns the number of columns in the matrix.
  int GetNumCols()const {return ncols_;}

  /// Returns the number of rows in the matrix.
  int GetNumRows()const {return nrows_;}

  /// Returns the value of element ij.
  double GetElement(int i, int j)const {return elements_[GetIndex(i,j)];}

  /// Prints the matrix to stdout
  void Print();

  /// Access the elements with parentheses. The first element is (0,0)
  double& operator() (int i, int j){
    return elements_[GetIndex(i,j)];
  }
  double operator() (int i, int j) const {
    return elements_[GetIndex(i,j)];
  }

  /// Compares the dimensions of matrices, returns true if they are the same.
  bool CompareSize(const Matrix& a)const;

 private:
  std::vector<double> elements_;
  int GetIndex(int i, int j) const;
  
  int PermuteSign(int i);
  int nrows_;
  int ncols_;
};

//----- Matrix operators --------------------------------------//
inline Matrix operator+(const Matrix& a, const Matrix& b) {
  Matrix c(a.GetNumRows(), a.GetNumCols());
  if (a.CompareSize(b)){
    for (int i = 0; i<a.GetNumRows(); i++){
      for (int j = 0; j<a.GetNumCols(); j++){
	c(i,j) = a(i,j) + b(i,j);
      }
    }
  }
  else {
    std::cerr<<"Warning, attempted to add "<<a.GetNumRows()<<"x"<<
      a.GetNumCols()<<" matrix and "<<b.GetNumRows()<<"x"<<b.GetNumCols()
	     <<"matrix."<<std::endl;
  }
  return c; 
}

inline Matrix operator-(const Matrix& a, const Matrix& b) {
  Matrix c(a.GetNumRows(), a.GetNumCols());
  if (a.CompareSize(b)){
    for (int i = 0; i<a.GetNumRows(); i++){
      for (int j = 0; j<a.GetNumCols(); j++){
	c(i,j) = a(i,j) - b(i,j);
      }
    }
  }
  else {
    std::cerr<<"Warning, attempted to subtract "<<a.GetNumRows()<<"x"<<
      a.GetNumCols()<<" matrix and "<<b.GetNumRows()<<"x"<<b.GetNumCols()
	     <<"matrix."<<std::endl;
  }
  return c; 
}

inline Matrix operator*(const Matrix& m, const double s) {
  Matrix b(m.GetNumRows(), m.GetNumCols());
  for (int i = 0; i<m.GetNumRows(); i++){
    for (int j = 0; j<m.GetNumCols(); j++){
      b(i,j) = s*m(i,j);
    }
  }
  return b; 
}

inline Matrix operator*(const double s, const Matrix& m){
  return m*s; 
}

inline Matrix operator/(const Matrix& m, const double s) {
  Matrix b(m.GetNumRows(), m.GetNumCols());
  for (int i = 0; i<m.GetNumRows(); i++){
    for (int j = 0; j<m.GetNumCols(); j++){
      b(i,j) = m(i,j)/s;
    }
  }
  return b; 
}

inline Matrix operator*(const Matrix& a, const Matrix& b) {
  Matrix c(a.GetNumRows(), b.GetNumCols());
  if (a.GetNumCols()==b.GetNumRows()){
    for (int i = 0; i<a.GetNumRows(); i++){
      for (int j = 0; j<b.GetNumCols(); j++){
	for (int k=0; k<a.GetNumCols();k++){
	  c(i,j) = c(i,j) + a(i,k)*b(k,j);
	}
      }
    }
  }
  else{
    std::cerr<<"Warning: Attempted to multiply matrix with inner dimensions "
	     <<a.GetNumCols()<<" and "<<b.GetNumRows()<<std::endl;
  }   
  return c; 
}
//--- Done with operators-----------------------------------------//

} //namespace math
} //namespace mavs

#endif
