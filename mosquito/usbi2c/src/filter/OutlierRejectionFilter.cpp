/*
 * OutlierRejectionFilter.cpp
 *
 *  Created on: Jan 24, 2012
 *      Author: tlinder
 */

#include "OutlierRejectionFilter.h"
#include <stdio.h>
#include <iostream>

OutlierRejectionFilter::OutlierRejectionFilter(double min, double max)
{
  _min = min;
  _max = max;
  _oldValues[0] = 0.0;
  _oldValues[1] = 0.0;
  _count = 0;
}

OutlierRejectionFilter::~OutlierRejectionFilter()
{
}

double OutlierRejectionFilter::run(double newValue)
{
  //if newValue if not in between the MIN MAX zone reject and send MIN or MAX as response
  //std::cout << "newValue " << newValue << " _oldValue " << _oldValue << std::endl;

  if ((_min < (newValue - _oldValues[1])) && (_max > (newValue - _oldValues[1])))
  {

  }
  else
  {
    //std::cout << _count << "+++++" << std::endl;

    _count++;
    if (_count <= 2)
    {

      if (_min > (newValue - _oldValues[1]))
      {
        double slope = _oldValues[1] - _oldValues[0];
        //      slope*=k;
        double result = _oldValues[1] + slope;
        _oldValues[0] = _oldValues[1];
        _oldValues[1] = result;
        return result;

      }
      else if (_max < (newValue - _oldValues[1]))
      {
        //      double tmp = _oldValue + _max;
        //      _oldValue = tmp;
        //      return tmp;
        double slope = _oldValues[1] - _oldValues[0];
        //      slope*=k;
        double result = _oldValues[1] + slope;
        _oldValues[0] = _oldValues[1];
        _oldValues[1] = result;
        return result;
      }
    }
  }
  //_count = _count > 0 ? _count-- : 0;
  if (_count > 0)
  {
    _count--;
  }
  else
  {
    _count = 0;
  }
  //std::cout << _count << "-----" << std::endl;
  _oldValues[0] = _oldValues[1];
  _oldValues[1] = newValue;
  return newValue;

}
