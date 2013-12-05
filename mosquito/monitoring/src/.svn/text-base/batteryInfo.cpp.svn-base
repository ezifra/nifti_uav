/*
 * battery.cpp
 *
 *  Created on: Jan 22, 2011
 *      Author: Thorsten Linder
 *      note: requires installed "acpi" package
 */

#include <ros/ros.h>
#include <monitoring_msgs/Battery.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <iostream>
#include <iomanip>
#include <fstream>
using namespace std;

monitoring_msgs::Battery bat;
diagnostic_msgs::DiagnosticArray diagArray;
std::stringstream floatStringHelper;

diagnostic_msgs::DiagnosticStatus getBatteryStatus() {
    diagnostic_msgs::DiagnosticStatus batteryStatus;

    batteryStatus.name = "currentBatteryLevel";
    batteryStatus.message = "The current battery level is ";
    if (0.5 <= bat.currentBatteryLevel) {
	batteryStatus.level = 0;
	batteryStatus.message += " OK";
    }
    else if (0.25 <= bat.currentBatteryLevel) {
	batteryStatus.level = 1;
	batteryStatus.message += " moderate";
    }
    else {
	batteryStatus.level = 2;
	batteryStatus.message += " weak";
    }

    batteryStatus.hardware_id = "battery";
    batteryStatus.values.resize(3);

    floatStringHelper.clear();
    floatStringHelper << bat.currentBatteryLevel;
    batteryStatus.values[0].key = "currentBatteryLevel";
    floatStringHelper >> batteryStatus.values[0].value;

    batteryStatus.values[1].key = "status";
    batteryStatus.values[1].value = bat.status;

    floatStringHelper.clear();
    floatStringHelper << bat.batteryTimeLeft;
    batteryStatus.values[2].key = "batteryTimeLeft";
    floatStringHelper >> batteryStatus.values[2].value;

    return batteryStatus;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "batteryInfo");
    ros::NodeHandle nodeHandle;
    ros::Publisher battery_pub = nodeHandle.advertise<monitoring_msgs::Battery> ("/monitoring/batteryInfo", 10);
    ros::Publisher diagnostic_pub = nodeHandle.advertise<diagnostic_msgs::DiagnosticArray> ("diagnostics", 100);

    string hz = "0.5";
    if (nodeHandle.getParam("/monitoring_batteryInfo/batteryInfo/hz", hz)) {

	ROS_INFO("Use battery info update frequency \"%s\" " , hz.c_str());
    }
    else {

	ROS_WARN("Using default battery info update frequency \"%s\"",hz.c_str());
    }
    ros::Rate loop_time(atof(hz.c_str()));

    std::string lastReadWord;
    int posFirstColon = 0;
    int posSecondeColon = 0;

    while (nodeHandle.ok()) {
	lastReadWord.clear();

	if (system("acpi -b > batteryInfo.txt")) {
	    ROS_ERROR("Could not recieve batteryInfo info by calling acpi!");
		loop_time.sleep();
	}
	else {
	    ifstream batteryFile;
	    batteryFile.open("batteryInfo.txt");
	    if (!batteryFile) {
		ROS_ERROR("Unable to open file batteryInfo.txt");
	    }
	    else {
		while (batteryFile >> lastReadWord) {
		    if ((0 == lastReadWord.compare("Charging,")) || (0 == lastReadWord.compare("Discharging,"))) {
			bat.status = lastReadWord.substr(0, lastReadWord.size() - 1);
			batteryFile >> lastReadWord;
			int end_pos = lastReadWord.find("%", 0);
			lastReadWord = lastReadWord.substr(0, end_pos);
			bat.currentBatteryLevel = atof(lastReadWord.c_str()) / 100.0;
			batteryFile >> lastReadWord;

			posFirstColon = lastReadWord.find(":", 0);
			posSecondeColon = lastReadWord.find(":", posFirstColon + 1);

			bat.batteryTimeLeft = atoi(lastReadWord.substr(0, posFirstColon).c_str()) * 3600;
			bat.batteryTimeLeft += atoi(lastReadWord.substr(posFirstColon + 1, posSecondeColon - posFirstColon - 1).c_str()) * 60;
			bat.batteryTimeLeft += atoi(lastReadWord.substr(posSecondeColon + 1, lastReadWord.length() - 1).c_str());
		    }
		    if ((0 == lastReadWord.compare("Unknown,")) || (0 == lastReadWord.compare("Full,"))) {
			bat.status = "Full";
			bat.currentBatteryLevel = 1.0;
		    }
		}

		batteryFile.close();
	    }

	    if (remove("batteryInfo.txt") != 0) {
		ROS_ERROR( "Error deleting batteryInfo.txt file" );
	    }

	}

	bat.header.stamp = ros::Time::now();
	bat.header.frame_id = "base_link";
	battery_pub.publish(bat);
	diagArray.header.stamp = ros::Time::now();
	diagArray.header.frame_id = "base_link";
	diagArray.status.clear();
	diagArray.status.push_back(getBatteryStatus());
	diagnostic_pub.publish(diagArray);
	ros::spinOnce();
	loop_time.sleep();

    }

}
