/*
 * MK_Serial_Test.cpp
 *
 *  Created on: Sep 26, 2011
 *      Author: tlinder
 */

#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <string>

#include "mk/MkFlightControl.h"

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "FlightControl");

  MkFlightControl* uav = new MkFlightControl();
  uav->run();

  delete (uav);
  std::cout<<"FlightControl was shutdown"<<std::endl;
  return 0;

}

