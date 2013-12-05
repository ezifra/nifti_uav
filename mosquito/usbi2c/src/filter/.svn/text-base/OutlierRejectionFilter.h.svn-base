/*
 * OutlierRejectionFilter.h
 *
 *  Created on: Jan 24, 2012
 *      Author: tlinder
 */

#ifndef OUTLIERREJECTIONFILTER_H_
#define OUTLIERREJECTIONFILTER_H_

class OutlierRejectionFilter
{
public:
  OutlierRejectionFilter(double min, double max);
  virtual ~OutlierRejectionFilter();
  double run(double newValue);
private:
  double _min,_max;
  double _oldValues[2];
  int _count;

};

#endif /* OUTLIERREJECTIONFILTER_H_ */
