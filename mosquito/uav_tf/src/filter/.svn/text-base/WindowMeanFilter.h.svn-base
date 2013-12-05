/*
 * windowMeanFilter.h
 *
 *  Created on: Jan 16, 2012
 *      Author: tlinder
 */

#ifndef WINDOWMEANFILTER_H_
#define WINDOWMEANFILTER_H_
#include <stdlib.h>
#include <queue>

class WindowMeanFilter {
public:
	WindowMeanFilter(unsigned int windowSize, double threshold);
	virtual ~WindowMeanFilter();
	double run(double newValue);
	unsigned int windowSize();

private:
	unsigned int _maxWindowSize;
	double sum;
	std::queue<double> rawData;
	double _threshold;

};

#endif /* WINDOWMEANFILTER_H_ */
