/*
 *
 *  Created on: Feb 7, 2011
 *      Author: Thorsten Linder
 */

#include <ros/ros.h>
#include <rescuebot_sensing/Trafic.h>
#include <iostream>
#include <iomanip>
#include <fstream>
using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "networkLinkTest");
    ros::NodeHandle nodeHandle;
    ros::Publisher wifi_pub = nodeHandle.advertise<rescuebot_sensing::Trafic> ("networkLinkTest", 1);

    string hz = "100.0";
    if (nodeHandle.getParam("hz", hz)) {
	ROS_INFO("Use update frequency \"%s\" " , hz.c_str());
    }
    else {
	ROS_WARN("Using default update frequency \"%s\"",hz.c_str());
    }
    ros::Rate loop_time(atof(hz.c_str()));


    string volume = "100000";
        if (nodeHandle.getParam("/rescuebot_sensing_networkLinkTest/networkLinkTest/volume", volume)) {
    	ROS_INFO("Use trafic volume of \"%s\" x 64-Bit " , volume.c_str());
        }
        else {
    	ROS_WARN("Use trafic volume of \"%s\" x 64-Bit ",volume.c_str());
        }



    rescuebot_sensing::Trafic trafic;

    int size = atoi(volume.c_str());

    trafic.length = size;
    trafic.trafic.resize(size);

    for(unsigned int i=0; i<trafic.trafic.size() ;i++)
    {
	trafic.trafic[i]=i;
    }

    while (nodeHandle.ok()) {


	wifi_pub.publish(trafic);
	ros::spinOnce();
	loop_time.sleep();

    }

}
