/*
 * thermalInfo.cpp
 *
 *  Created on: Feb 19, 2011
 *      Author: Thorsten Linder
 *      note: requires installed "acpi" package
 */

#include <ros/ros.h>
#include <monitoring_msgs/Temperature.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <iostream>
#include <iomanip>
#include <fstream>
using namespace std;

monitoring_msgs::Temperature temperatureContainer;
diagnostic_msgs::DiagnosticArray diagArray;
std::stringstream floatStringHelper;

diagnostic_msgs::DiagnosticStatus getTemperatureStatus() {
    diagnostic_msgs::DiagnosticStatus temperatureStatus;

    temperatureStatus.name = "temperature";
    temperatureStatus.message = "The current temperature is ";
    bool statusOK = true;
    for (unsigned int i = 0; i < temperatureContainer.values.size(); i++) {
	if (temperatureContainer.values[i].status.compare("ok") != 0) {
	    statusOK = statusOK && false;
	}
    }

    if (true == statusOK) {
	temperatureStatus.level = 0;
	temperatureStatus.message += " OK";
    }
    else {
	temperatureStatus.level = 2;
	temperatureStatus.message += " HOT";
    }

    temperatureStatus.hardware_id = "";

    temperatureStatus.values.resize(temperatureContainer.values.size() * 2);

    for (unsigned int i = 0; i < temperatureStatus.values.size(); i++) {

	if (0 == i % 2) {
	    temperatureStatus.values[i].key = temperatureContainer.values[i].key;
	    temperatureStatus.values[i].value = temperatureContainer.values[i].status;
	}
	else {
	    floatStringHelper.clear();
	    floatStringHelper << temperatureContainer.values[i / 2].temperature;

	    temperatureStatus.values[i].key = temperatureContainer.values[i / 2].key;
	    floatStringHelper >> temperatureStatus.values[i].value;

	}
    }

    return temperatureStatus;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "batteryInfo");
    ros::NodeHandle nodeHandle;
    ros::Publisher thermo_pub = nodeHandle.advertise<monitoring_msgs::Temperature> ("/monitoring/thermalInfo", 10);
    ros::Publisher diagnostic_pub = nodeHandle.advertise<diagnostic_msgs::DiagnosticArray> ("diagnostics", 100);

    string hz = "0.5";
    if (nodeHandle.getParam("/monitoring_thermalInfo/batteryInfo/hz", hz)) {

	ROS_INFO("Use thermal info info update frequency \"%s\" " , hz.c_str());
    }
    else {

	ROS_WARN("Using default thermal info info update frequency \"%s\"",hz.c_str());
    }
    ros::Rate loop_time(atof(hz.c_str()));

    std::string lastReadWord;

    while (nodeHandle.ok()) {
	lastReadWord.clear();

	if (system("acpi -t > thermalInfo.txt")) {
	    ROS_ERROR("Could not recieve batteryInfo info by calling acpi!");
		loop_time.sleep();
	}
	else {
	    ifstream thermalFile;
	    thermalFile.open("thermalInfo.txt");
	    if (!thermalFile) {
		ROS_ERROR("Unable to open file thermalInfo.txt");
	    }
	    else {
		temperatureContainer.name = "Temperature";
		temperatureContainer.values.clear();

		while (thermalFile >> lastReadWord) {
		    if (0 == lastReadWord.compare("Thermal")) {
			monitoring_msgs::ThermoValues thermo;
			thermo.key = lastReadWord;
			thermalFile >> lastReadWord;
			int pos = lastReadWord.find(":", 0);
			thermo.key += " " + lastReadWord.substr(0, pos);
			thermalFile >> lastReadWord;
			pos = lastReadWord.find(",", 0);
			thermo.status = lastReadWord.substr(0, pos);
			thermalFile >> lastReadWord;
			thermo.temperature = atof(lastReadWord.c_str());
			thermalFile >> lastReadWord;
			thermalFile >> lastReadWord;
			thermo.unit = lastReadWord;
			temperatureContainer.values.push_back(thermo);
		    }

		    thermalFile.close();
		}

		if (remove("thermalInfo.txt") != 0) {
		    ROS_ERROR( "Error deleting thermalInfo.txt file" );
		}

	    }

	    temperatureContainer.header.stamp = ros::Time::now();
	    temperatureContainer.header.frame_id = "base_link";
	    thermo_pub.publish(temperatureContainer);
	    diagArray.header.stamp = ros::Time::now();
	    diagArray.header.frame_id = "base_link";
	    diagArray.status.clear();
	    diagArray.status.push_back(getTemperatureStatus());
	    diagnostic_pub.publish(diagArray);
	    ros::spinOnce();
	    loop_time.sleep();

	}
    }

}
