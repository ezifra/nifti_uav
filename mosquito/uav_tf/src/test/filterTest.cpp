/*
 * filterTest.cpp
 *
 *  Created on: Jan 16, 2012
 *      Author: tlinder
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "../filter/WindowMeanFilter.h"
#include "../filter/ContingencyAnalysisWindowMeanFilter.h"

using namespace std;

int main(int argc, char** argv)
{
  std::cout << "-----------------WindowMeanFilter----------------------" << std::endl;
  WindowMeanFilter filter(5, 2);
  std::cout << "size is " << filter.windowSize() << std::endl;

  for (int i = 1; i <= 10; i++)
  {
    double temp = filter.run(i);
    std::cout << i << " - size is " << filter.windowSize() << " mean is " << temp << std::endl;
  }

  std::cout << std::endl;

  double temp = filter.run(100);
  std::cout << "size is " << filter.windowSize() << " mean is " << temp << std::endl;

  std::cout << "-----------------ContingencyAnalysisWindowMeanFilter----------------------" << std::endl;

  ContingencyAnalysisWindowMeanFilter cAFilter(5 ,0.5);
  std::cout << "size is " << cAFilter.windowSize() << std::endl;

  for (int i = 1; i <= 10; i++)
  {
    double temp = cAFilter.run(i);
    std::cout << i << " - size is " << cAFilter.windowSize() << " mean is " << temp << std::endl;
  }

  temp = cAFilter.run(100);
  std::cout << "After inserting 1x outlier's (Value = 100) size is " << cAFilter.windowSize() << " mean is " << temp << std::endl;
  temp = cAFilter.run(100);
  std::cout << "After inserting 1x outlier's (Value = 100) size is " << cAFilter.windowSize() << " mean is " << temp << std::endl;

  for (int i = 10; i > 0; i--)
  {
    double temp = cAFilter.run(i);
    std::cout << i << " - size is " << cAFilter.windowSize() << " mean is " << temp << std::endl;
  }

  std::cout << "-----------------ContingencyAnalysisWindowMeanFilter-2--------------------" << std::endl;

   ContingencyAnalysisWindowMeanFilter cAFilter2(5,0.5);
   std::cout << "size is " << cAFilter2.windowSize() << std::endl;

   for (int i = 1; i <= 10; i++)
   {

     double temp = cAFilter2.run(i%5);
     std::cout << i%5 << " - size is " << cAFilter2.windowSize() << " mean is " << temp << std::endl;
   }

//   std::cout << std::endl;
//
//   temp = cAFilter2.run(100);
//   std::cout << "size is " << cAFilter2.windowSize() << " mean is " << temp << std::endl;
//
//   for (int i = 20; i > 0; i--)
//   {
//     double temp = cAFilter2(i);
//     std::cout << i << " - size is " << cAFilter2.windowSize() << " mean is " << temp << std::endl;
//   }

   std::cout << "-----------------ContingencyAnalysisWindowMeanFilter-3--------------------" << std::endl;

    ContingencyAnalysisWindowMeanFilter cAFilter3(3,0.5);
    std::cout << "size is " << cAFilter3.windowSize() << std::endl;

    for (int i = 1; i <= 10; i++)
    {

      double temp = cAFilter3.run(1);
      std::cout << 1 << " - size is " << cAFilter3.windowSize() << " mean is " << temp << std::endl;
    }

 //   std::cout << std::endl;
 //
 //   temp = cAFilter2.run(100);
 //   std::cout << "size is " << cAFilter2.windowSize() << " mean is " << temp << std::endl;
 //
 //   for (int i = 20; i > 0; i--)
 //   {
 //     double temp = cAFilter2(i);
 //     std::cout << i << " - size is " << cAFilter2.windowSize() << " mean is " << temp << std::endl;
 //   }

}
