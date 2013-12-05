/*
 * memoryInfo.cpp
 *
 *  Created on: Feb 20, 2011
 *      Author: Thorsten Linder
 */

#include <ros/ros.h>
#include <monitoring_msgs/Memory.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <iostream>
#include <iomanip>
#include <fstream>
using namespace std;

monitoring_msgs::Memory memory;
diagnostic_msgs::DiagnosticArray diagArray;
std::stringstream floatStringHelper;

diagnostic_msgs::DiagnosticStatus getMemoryStatus() {
    diagnostic_msgs::DiagnosticStatus memoryStatus;

    memoryStatus.name = "memoryStatus";
    memoryStatus.message = "The current memory status is ";
    float percentage = (memory.memFree + memory.memCached + memory.memBuffers) * 1.0 / memory.memTotal;
    if (0.3 <= percentage) {
	memoryStatus.level = 0;
	memoryStatus.message += " OK";
    }
    else if (0.2 <= percentage) {
	memoryStatus.level = 1;
	memoryStatus.message += " moderate";
    }
    else {
	memoryStatus.level = 2;
	memoryStatus.message += " weak";
    }

    memoryStatus.hardware_id = "memory";
    memoryStatus.values.resize(5);

    floatStringHelper.clear();
    floatStringHelper << percentage;
    memoryStatus.values[0].key = "free";
    floatStringHelper >> memoryStatus.values[0].value;

    floatStringHelper.clear();
    floatStringHelper << memory.memTotal;
    memoryStatus.values[1].key = "memTotal";
    floatStringHelper >> memoryStatus.values[1].value;

    floatStringHelper.clear();
    floatStringHelper << memory.memFree;
    memoryStatus.values[2].key = "memFree";
    floatStringHelper >> memoryStatus.values[2].value;

    floatStringHelper.clear();
    floatStringHelper << memory.memCached;
    memoryStatus.values[3].key = "memCached";
    floatStringHelper >> memoryStatus.values[3].value;

    floatStringHelper.clear();
    floatStringHelper << memory.memBuffers;
    memoryStatus.values[4].key = "memBuffers";
    floatStringHelper >> memoryStatus.values[4].value;

    return memoryStatus;
}

diagnostic_msgs::DiagnosticStatus getSwapStatus() {
    diagnostic_msgs::DiagnosticStatus swapStatus;

    swapStatus.name = "swapStatus";
    swapStatus.message = "The current swap space status is ";
    float percentage = (memory.swapFree + memory.swapCached) * 1.0 / memory.swapTotal;
    if (0.5 <= percentage) {
	swapStatus.level = 0;
	swapStatus.message += " OK";
    }
    else if (0.25 <= percentage) {
	swapStatus.level = 1;
	swapStatus.message += " moderate";
    }
    else {
	swapStatus.level = 2;
	swapStatus.message += " weak";
    }

    swapStatus.hardware_id = "swap_space";
    swapStatus.values.resize(4);

    floatStringHelper.clear();
    floatStringHelper << percentage;
    swapStatus.values[0].key = "free";
    floatStringHelper >> swapStatus.values[0].value;

    floatStringHelper.clear();
    floatStringHelper << memory.swapTotal;
    swapStatus.values[1].key = "swapTotal";
    floatStringHelper >> swapStatus.values[1].value;

    floatStringHelper.clear();
    floatStringHelper << memory.swapFree;
    swapStatus.values[2].key = "swapFree";
    floatStringHelper >> swapStatus.values[2].value;

    floatStringHelper.clear();
    floatStringHelper << memory.swapCached;
    swapStatus.values[3].key = "swapCached";
    floatStringHelper >> swapStatus.values[3].value;

    return swapStatus;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "memoryInfo");
    ros::NodeHandle nodeHandle;
    ros::Publisher memory_pub = nodeHandle.advertise<monitoring_msgs::Memory> ("/monitoring/memoryInfo", 10);
    ros::Publisher diagnostic_pub = nodeHandle.advertise<diagnostic_msgs::DiagnosticArray> ("diagnostics", 100);

    string hz = "0.5";
    if (nodeHandle.getParam("/monitoring_memoryInfo/memoryInfo/hz", hz)) {

	ROS_INFO("Use memory info update frequency \"%s\" " , hz.c_str());
    }
    else {

	ROS_WARN("Using default memory info update frequency \"%s\"",hz.c_str());
    }
    ros::Rate loop_time(atof(hz.c_str()));

    std::string lastReadWord;

    while (nodeHandle.ok()) {
	lastReadWord.clear();
	ifstream memoryFile;
	memoryFile.open("/proc/meminfo");
	if (!memoryFile) {
	    ROS_ERROR("Unable to open file batteryInfo.txt");
		loop_time.sleep();
	}
	else {
	    while (memoryFile >> lastReadWord) {
		if (0 == lastReadWord.compare("MemTotal:")) {
		    memoryFile >> lastReadWord;
		    memory.memTotal = atoi(lastReadWord.c_str());
		    continue;
		}
		else if (0 == lastReadWord.compare("MemFree:")) {
		    memoryFile >> lastReadWord;
		    memory.memFree = atoi(lastReadWord.c_str());
		    continue;
		}
		else if (0 == lastReadWord.compare("Cached:")) {
		    memoryFile >> lastReadWord;
		    memory.memCached = atoi(lastReadWord.c_str());
		    continue;
		}
		else if (0 == lastReadWord.compare("Buffers:")) {
				    memoryFile >> lastReadWord;
				    memory.memBuffers = atoi(lastReadWord.c_str());
				    continue;
				}
		else if (0 == lastReadWord.compare("SwapTotal:")) {
		    memoryFile >> lastReadWord;
		    memory.swapTotal = atoi(lastReadWord.c_str());
		    continue;
		}
		else if (0 == lastReadWord.compare("SwapFree:")) {
		    memoryFile >> lastReadWord;
		    memory.swapFree = atoi(lastReadWord.c_str());
		    continue;
		}
		else if (0 == lastReadWord.compare("SwapCached:")) {
		    memoryFile >> lastReadWord;
		    memory.swapCached = atoi(lastReadWord.c_str());
		    continue;
		}
	    }

	    memoryFile.close();
	    memory.header.stamp = ros::Time::now();
	    memory.header.frame_id = "base_link";
	    memory_pub.publish(memory);
	    diagArray.header.stamp = ros::Time::now();
	    diagArray.header.frame_id = "base_link";
	    diagArray.status.clear();
	    diagArray.status.push_back(getMemoryStatus());
	    diagArray.status.push_back(getSwapStatus());
	    diagnostic_pub.publish(diagArray);
	    ros::spinOnce();
	    loop_time.sleep();

	}
    }
}
