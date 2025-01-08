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
* \file quick_sort.h
*
* Template functions for quick sort algorithm
* See http://web.engr.oregonstate.edu/~budd/Books/stl/info/sources/quickSort.cpp
*
* \author Chris Goodin
*
* \date 6/27/2018
*/

#ifndef QUICK_SORT_H
#define QUICK_SORT_H

namespace mavs {
namespace utils {

template <class T>
unsigned int Pivot(std::vector<T> & v, unsigned int start,
	unsigned int stop, unsigned int position)
	// partition vector into two groups
	// values smaller than or equal to pivot
	// values larger than pivot
	// return location of pivot element
{
	// swap pivot into starting position
	std::swap(v[start], v[position]);

	// partition values
	unsigned int low = start + 1;
	unsigned int high = stop;
	while (low < high)
		if (v[low] < v[start])
			low++;
		else if (v[--high] < v[start])
			std::swap(v[low], v[high]);

	// then swap pivot back into place
	std::swap(v[start], v[--low]);
	return low;
}

template <class T>
void QuickSort(std::vector<T> & v, unsigned int low, unsigned int high)
{
	// no need to sort a vector of zero or one elements
	if (low >= high)
		return;

	// select the pivot value
	unsigned int pivotIndex = (low + high) / 2;

	// partition the vector
	pivotIndex = Pivot(v, low, high, pivotIndex);

	// sort the two sub vectors
	if (low < pivotIndex)
		QuickSort(v, low, pivotIndex);
	if (pivotIndex < high)
		QuickSort(v, pivotIndex + 1, high);
}

template <class T> void QuickSort(std::vector<T> & v)
{
	unsigned int numberElements = (unsigned int)v.size();
	if (numberElements > 1)
		QuickSort(v, 0, numberElements - 1);
}

}//namespace utils
} //namespace math

#endif