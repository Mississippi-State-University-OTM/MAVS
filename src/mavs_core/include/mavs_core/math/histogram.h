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
#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <vector>
#include <string>
#include <fstream>

namespace mavs{
namespace math{

/**
 * \class Histogram
 *
 * A histogram class.
 *
 * \author Chris Goodin
 *
 * \date 2/1/2018
 */
template <class T>
class Histogram{
 public:
	 /// Create an empty histogram
	 Histogram() { }

  /**
   * Create the histogram
   * \param min The minimum x-value of the histogram
   * \param max The maximum x-value of the histogram
   * \param res The resolution of the histogram
   */
  Histogram(T min, T max, T res){
    min_ = min;
    max_ = max;
    res_ = res;
    AllocateHist(min_, max_, res_);
  }
  
  /**
   * Construct a histogram from a vector of values
	 * \param data The data to build the histogram
	 * \param res The binning resolution of the data
   */
	Histogram(std::vector<T> data, T res) {
		if (data.size() > 0) {
			res_ = res;
			min_ = (T)(1.0E6);
			max_ = (T)(-1.0E6);
			for (int i = 0; i < data.size(); i++) {
				if (data[i] < min_)min_ = data[i];
				if (data[i] > max_)max_ = data[i];
			}
			min_ = min_ - 0.5f*res_;
			AllocateHist(min_, max_, res_);
			for (int i = 0; i < data.size(); i++) {
				AddToHistogram(data[i]);
			}
		}
	}

  /**
   * Add a point at value "x" to the histogram counter
   */
	void AddToHistogram(T x) {
		if (x >= min_ && x <= max_) {
			count_[GetIndex(x)] += 1;
		}
		else if (x < min_) {
			std::vector<double> old_count = count_;
			T old_min = min_;
			min_ = x;
			AllocateHist(min_, max_, res_);
			AddToHistogram(x);
			for (int i = 0; i < old_count.size(); i++) {
				T x2 = min_ + res_ * i;
				for (int j = 0; j < old_count[i]; j++)AddToHistogram(x2);
			}
		}
		else if (x > max_) {
			std::vector<double> old_count = count_;
			T old_max = max_;
			max_ = x;
			AllocateHist(min_, max_, res_);
			AddToHistogram(x);
			for (int i = 0; i < old_count.size(); i++) {
				T x2 = min_ + res_ * i;
				for (int j = 0; j < old_count[i]; j++)AddToHistogram(x2);
			}
		}
	}

  ///Normalize the histogram
  void Normalize(){
    double sum = 0.0;
    for (int i = 0; i<count_.size();i++){
      sum += (double) count_[i];
    }
    sum = sum*res_;
    for (int i=0; i<count_.size();i++){
      count_[i] = (T)(count_[i]/sum);
    }
  }

  double GetMean(){
    double num = 0.0;
    double sum = 0.0;
    for (int i=0;i<count_.size();i++){
      num += (min_+ (i+0.5)*res_)*count_[i];
      sum += count_[i];
    }
    return num/sum;
  }

	double GetMode() {
		double mode = (double)min_;
		T maxval = (T) 0.0;
		for (int i = 0; i < count_.size(); i++) {
			if (count_[i] > maxval) {
				maxval = count_[i];
				mode = (double)(min_ + (i + 0.5)*res_);
			}
		}
		return mode;
	}

  /// Write histogram to file "out_name"
  void WriteToFile(std::string out_name){
    std::ofstream fout;
    fout.open(out_name.c_str());
    for (int i=0;i<count_.size();i++){
      fout<<(i+0.5)*res_ + min_<<" "<<count_[i]<<std::endl;;
    }
    fout.close();
  }

 private:
  int GetIndex(T x){
    int n = (int) ((x-min_)/res_);
    return n;
  }

  void AllocateHist(T min, T max, T res){
    int n = (int)((max-min)/res) + 1;
    count_.resize(n,0);
  }

  std::vector<double> count_;
  T res_;
  T min_;
  T max_;
};

} // namespace math
} // namespace mavs
#endif
