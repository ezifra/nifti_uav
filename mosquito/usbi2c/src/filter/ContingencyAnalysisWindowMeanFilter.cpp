/*
 * ContingencyAnalysisWindowMeanFilter.cpp
 *
 *  Created on: Jan 23, 2012
 *      Author: tlinder
 */

#include "ContingencyAnalysisWindowMeanFilter.h"
#include <assert.h>
#include <stdio.h>
#include <iostream>

using namespace std;

ContingencyAnalysisWindowMeanFilter::ContingencyAnalysisWindowMeanFilter(unsigned int windowSize, double sigma)
{
  rawData.resize(windowSize);
  _oldestValuePos = 0;
  _sigma = sigma;
}

ContingencyAnalysisWindowMeanFilter::~ContingencyAnalysisWindowMeanFilter()
{
  rawData.resize(0);
}

unsigned int ContingencyAnalysisWindowMeanFilter::windowSize()
{
  return rawData.size();
}

double ContingencyAnalysisWindowMeanFilter::run(double newValue)
{
  //Compute the mean of rawData
  double mean = 0.0;
  for (unsigned int i = 0; i < rawData.size(); i++)
  {
    mean += rawData[i];
  }
  mean = mean / rawData.size();

  //Compute the variance of rawData
  double standardDeviation = 0.0;
  for (unsigned int i = 0; i < rawData.size(); i++)
  {
    standardDeviation += pow(rawData[i] - mean, 2);
  }
  standardDeviation = sqrt(standardDeviation / rawData.size());
  standardDeviation *= _sigma;

  //Computed the weighted mean (including newvalue) as return value
  double result = 0.0;
  double normalizer = 0.0;
  double likelihood = 0.0;
  if (abs(standardDeviation) > 0.0)
  {
    //Assume Gaussian Distribution
    for (unsigned int i = 0; i < rawData.size(); i++)
    {
      likelihood = NormalDist(mean, standardDeviation, rawData[i]);

      //      std::cout << "likelihood "<<likelihood <<" of rawData[i] "<< rawData[i] <<std::endl;
      result += likelihood * rawData[i];
      normalizer += likelihood;
    }
    likelihood = NormalDist(mean, standardDeviation, newValue);
    result += likelihood * newValue;
    normalizer += likelihood;
    result = result / normalizer;
  }
  else
  {
    //Assume Uniform Distribution
    likelihood = UniformDist(rawData.size());
    for (unsigned int i = 0; i < rawData.size(); i++)
    {
      result += rawData[i];
//      std::cout << "likelihood " << likelihood << " of rawData[i] " << rawData[i] << std::endl;
    }
    result += newValue;
    result *= likelihood;
  }

  //store the newValue
  rawData[_oldestValuePos] = newValue;
  _oldestValuePos = ++_oldestValuePos % rawData.size();
  return result;
}

double ContingencyAnalysisWindowMeanFilter::NormalDist(double mean, double standardDeviation, double value)
{
  assert(0!= standardDeviation);
  double normaliser = 1.0 / (standardDeviation * sqrt(2.0 * M_PI));
  double expo = exp(-0.5 * pow((value - mean) / standardDeviation, 2));
  //std::cout << "       normaliser "<< normaliser << " expo " << expo << std::endl;
  return normaliser * expo;
}

double ContingencyAnalysisWindowMeanFilter::UniformDist(unsigned int cardinality)
{
  assert(0!=cardinality);

  return 1.0 / (cardinality+1.0);
}
