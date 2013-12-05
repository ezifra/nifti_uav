/*
 * ContingencyAnalysisWindowMeanFilter.h
 *
 *  Created on: Jan 23, 2012
 *      Author: tlinder
 */

#ifndef CONTINGENCYANALYSISWINDOWMEANFILTER_H_
#define CONTINGENCYANALYSISWINDOWMEANFILTER_H_

#include "WindowMeanFilter.h"
#include <cmath>

class ContingencyAnalysisWindowMeanFilter
{
public:
  ContingencyAnalysisWindowMeanFilter(unsigned int windowSize, double sigma);
  virtual ~ContingencyAnalysisWindowMeanFilter();
  double run(double newValue);
  unsigned int windowSize();
private:
  double NormalDist(double mean, double varianze, double value);
  double UniformDist(unsigned int cardinality);
  std::vector<double> rawData;
  unsigned int _oldestValuePos;
  double _sigma;
};

#endif /* CONTINGENCYANALYSISWINDOWMEANFILTER_H_ */
