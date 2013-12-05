/*
 * windowMeanFilter.cpp
 *
 *  Created on: Jan 16, 2012
 *      Author: tlinder
 */

#include "WindowMeanFilter.h"


WindowMeanFilter::WindowMeanFilter(unsigned int windowSize, double threshold) {
	_maxWindowSize = windowSize;
	_threshold = threshold;
	sum = 0.0;

}

WindowMeanFilter::~WindowMeanFilter() {
	while (!rawData.empty()){
	  rawData.pop();
	}
}

double WindowMeanFilter::run(double newValue) {

	if ((abs(newValue - sum / rawData.size()) > _threshold) && (rawData.size() > 0)) {
		//if the new value is to different then the mean the replace it with the mean
		newValue = sum / rawData.size();
	}
	if (rawData.size() < _maxWindowSize) {
		//In this case the window size is not reached, so just fill the queue and do not drop any element
		sum += newValue;
		rawData.push(newValue);

	} else {
		//In this case the window size is reached, drop the first element of the queue and recomputed the sum
		sum -= rawData.front();
		sum += newValue;
		rawData.pop();
		rawData.push(newValue);
	}

	//return the mean
	return sum / rawData.size();
}

unsigned int WindowMeanFilter::windowSize() {
	return rawData.size();
}
