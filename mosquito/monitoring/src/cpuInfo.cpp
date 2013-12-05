/*
 * cpuInfo.cpp
 *
 *  Created on: Feb 21, 2011
 *      Author: Thorsten Linder
 */

#include <ros/ros.h>
#include <monitoring_msgs/CPU.h>
#include <monitoring_msgs/CPUValues.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <iostream>
#include <iomanip>
#include <fstream>
using namespace std;

monitoring_msgs::CPU cpu;
diagnostic_msgs::DiagnosticArray diagArray;
std::stringstream floatStringHelper;
unsigned int cpu_time_total_after;
unsigned int cpu_time_total_before;
unsigned int cpu_idel_time_after;
unsigned int cpu_idel_time_before;

vector<int> sibling_time;

diagnostic_msgs::DiagnosticStatus getCPUStatus() {
    diagnostic_msgs::DiagnosticStatus cpuStatus;

    cpuStatus.name = "cpuStatus";
    cpuStatus.message = "The current cpu load is ";

    if (0.8 >= cpu.load) {
	cpuStatus.level = 0;
	cpuStatus.message += " OK";
    }
    else if (0.9 >= cpu.load) {
	cpuStatus.level = 1;
	cpuStatus.message += " moderate";
    }
    else {
	cpuStatus.level = 2;
	cpuStatus.message += " loaded";
    }

    cpuStatus.hardware_id = "memory";
    cpuStatus.values.resize(3 + cpu.cpu_siblings * 2);

    floatStringHelper.clear();
    floatStringHelper.str("");
    floatStringHelper << cpu.load;
    cpuStatus.values[0].key = "loadTotal";
    floatStringHelper >> cpuStatus.values[0].value;

    floatStringHelper.clear();
    floatStringHelper.str("");
    floatStringHelper << cpu.cpu_cores;
    cpuStatus.values[1].key = "cpuCores";
    floatStringHelper >> cpuStatus.values[1].value;

    floatStringHelper.clear();
    floatStringHelper.str("");
    floatStringHelper << cpu.cpu_siblings;
    cpuStatus.values[2].key = "cpuSiblings";
    floatStringHelper >> cpuStatus.values[2].value;

    for (unsigned int i = 0; i < cpu.siblings.size(); i++) {

	floatStringHelper.clear();
	floatStringHelper.str("");
	floatStringHelper << "loadCPU" << i;
	floatStringHelper >> cpuStatus.values[i + 3].key;
	floatStringHelper.clear();
	floatStringHelper.str("");
	floatStringHelper << cpu.siblings[i].load;
	floatStringHelper >> cpuStatus.values[i + 3].value;
    }

    for (unsigned int i = 0; i < cpu.siblings.size(); i++) {

	floatStringHelper.clear();
	floatStringHelper.str("");
	floatStringHelper << "speedCPU" << i;
	floatStringHelper >> cpuStatus.values[i + 3 + cpu.siblings.size()].key;
	floatStringHelper.clear();
	floatStringHelper.str("");
	floatStringHelper << cpu.siblings[i].cpuMhz;
	floatStringHelper >> cpuStatus.values[i + 3 + cpu.siblings.size()].value;
    }

    return cpuStatus;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cpuInfo");
    ros::NodeHandle nodeHandle;
    ros::Publisher cpu_pub = nodeHandle.advertise<monitoring_msgs::CPU> ("/monitoring/cpuInfo", 10);
    ros::Publisher diagnostic_pub = nodeHandle.advertise<diagnostic_msgs::DiagnosticArray> ("diagnostics", 100);

    string hz = "0.5";
    if (nodeHandle.getParam("/monitoring_cpuInfo/cpuInfo/hz", hz)) {

	ROS_INFO("Use cpu info update frequency \"%s\" " , hz.c_str());
    }
    else {

	ROS_WARN("Using default cpu info update frequency \"%s\"",hz.c_str());
    }
    ros::Rate loop_time(atof(hz.c_str()));

    std::string lastReadWord;

    while (nodeHandle.ok()) {
	lastReadWord.clear();
	ifstream cpuInfoFile;
	cpuInfoFile.open("/proc/cpuinfo");
	if (!cpuInfoFile) {
	    ROS_ERROR("Unable to open /proc/state");
	    loop_time.sleep();
	}
	else {
	    while (cpuInfoFile >> lastReadWord) {
		if (0 == lastReadWord.compare("siblings")) {
		    cpuInfoFile >> lastReadWord;
		    cpuInfoFile >> lastReadWord;
		    cpu.cpu_siblings = atoi(lastReadWord.c_str());
		}
		else if (0 == lastReadWord.compare("cores")) {
		    cpuInfoFile >> lastReadWord;
		    cpuInfoFile >> lastReadWord;
		    cpu.cpu_cores = atoi(lastReadWord.c_str());
		}
	    }
	    cpuInfoFile.close();
	}

	cpu.siblings.clear();
	cpu.siblings.resize(cpu.cpu_siblings);
	sibling_time.resize(cpu.cpu_siblings * 4);

	cpuInfoFile.open("/proc/cpuinfo");
	if (!cpuInfoFile) {
	    ROS_ERROR("Unable to open /proc/state");
	}
	else {
	    int current_procs = 0;
	    while (cpuInfoFile >> lastReadWord) {
		if (0 == lastReadWord.compare("processor")) {
		    cpuInfoFile >> lastReadWord;
		    cpuInfoFile >> lastReadWord;
		    current_procs = atoi(lastReadWord.c_str());
		    cpu.siblings[current_procs].key = current_procs;
		}
		else if (0 == lastReadWord.compare("vendor_id")) {
		    cpuInfoFile >> lastReadWord;
		    cpuInfoFile >> lastReadWord;
		    cpu.siblings[current_procs].vendorId = lastReadWord;
		}
		else if (0 == lastReadWord.compare("model")) {
		    cpuInfoFile >> lastReadWord;
		    if (0 == lastReadWord.compare("name")) {
			cpuInfoFile >> lastReadWord;
			cpuInfoFile >> lastReadWord;
			while (0 != lastReadWord.compare("stepping")) {
			    cpu.siblings[current_procs].modelName += lastReadWord + " ";
			    cpuInfoFile >> lastReadWord;
			}
		    }
		}
		else if (0 == lastReadWord.compare("cpu")) {
		    cpuInfoFile >> lastReadWord;
		    if (0 == lastReadWord.compare("MHz")) {
			cpuInfoFile >> lastReadWord;
			cpuInfoFile >> lastReadWord;
			floatStringHelper.clear();
			floatStringHelper << lastReadWord;
			floatStringHelper >> cpu.siblings[current_procs].cpuMhz;
		    }
		}

	    }
	    cpuInfoFile.close();
	}

	lastReadWord.clear();
	ifstream cpuStateFile;
	cpuStateFile.open("/proc/stat");
	if (!cpuStateFile) {
	    ROS_ERROR("Unable to open /proc/state");
	}
	else {
	    int current_procs = 0;
	    stringstream sstream;
	    floatStringHelper.clear();
	    sstream << "cpu" << current_procs;
	    while (cpuStateFile >> lastReadWord) {
		//ROS_INFO("%s",lastReadWord.c_str());

		if (0 == lastReadWord.compare("cpu")) {
		    cpuStateFile >> lastReadWord;
		    cpu_time_total_after = atoi(lastReadWord.c_str());//user mode utime
		    cpuStateFile >> lastReadWord;
		    cpu_time_total_after += atoi(lastReadWord.c_str());//user mode with low priority (nice) ntime
		    cpuStateFile >> lastReadWord;
		    cpu_time_total_after += atoi(lastReadWord.c_str());//system mode stime
		    cpuStateFile >> lastReadWord;
		    cpu_idel_time_after = atoi(lastReadWord.c_str());//idle task itime;
		    cpu_time_total_after += atoi(lastReadWord.c_str());
		    cpuStateFile >> lastReadWord;
		    cpu_time_total_after += atoi(lastReadWord.c_str());//time waiting for I/O to complete iowait
		    cpuStateFile >> lastReadWord;
		    cpu_time_total_after += atoi(lastReadWord.c_str());//time servicing interrupts irq
		    cpuStateFile >> lastReadWord;
		    cpu_time_total_after += atoi(lastReadWord.c_str());//time servicing softirqs softirq
		    cpuStateFile >> lastReadWord;
		    cpu_time_total_after += atoi(lastReadWord.c_str());
		    cpuStateFile >> lastReadWord;
		    cpu_time_total_after += atoi(lastReadWord.c_str());
		    cpuStateFile >> lastReadWord;
		    cpu_time_total_after += atoi(lastReadWord.c_str());

		    cpu.load = 1 - ((cpu_idel_time_after - cpu_idel_time_before) * 1.0 / (cpu_time_total_after
			    - cpu_time_total_before));
		    cpu_time_total_before = cpu_time_total_after;
		    cpu_idel_time_before = cpu_idel_time_after;

		}
		else if (0 == lastReadWord.compare(sstream.str().c_str())) {//cpuX
		    cpuStateFile >> lastReadWord;
		    sibling_time[(current_procs * 4)] = atoi(lastReadWord.c_str());//user mode utime
		    cpuStateFile >> lastReadWord;
		    sibling_time[(current_procs * 4)] += atoi(lastReadWord.c_str());//user mode with low priority (nice) ntime
		    cpuStateFile >> lastReadWord;
		    sibling_time[(current_procs * 4)] += atoi(lastReadWord.c_str());//system mode stime
		    cpuStateFile >> lastReadWord;
		    sibling_time[(current_procs * 4) + 2] = atoi(lastReadWord.c_str());//idle task itime;
		    sibling_time[(current_procs * 4)] += atoi(lastReadWord.c_str());
		    cpuStateFile >> lastReadWord;
		    sibling_time[(current_procs * 4)] += atoi(lastReadWord.c_str());//time waiting for I/O to complete iowait
		    cpuStateFile >> lastReadWord;
		    sibling_time[(current_procs * 4)] += atoi(lastReadWord.c_str());//time servicing interrupts irq
		    cpuStateFile >> lastReadWord;
		    sibling_time[(current_procs * 4)] += atoi(lastReadWord.c_str());//time servicing softirqs softirq
		    cpuStateFile >> lastReadWord;
		    sibling_time[(current_procs * 4)] += atoi(lastReadWord.c_str());
		    cpuStateFile >> lastReadWord;
		    sibling_time[(current_procs * 4)] += atoi(lastReadWord.c_str());
		    cpuStateFile >> lastReadWord;
		    sibling_time[(current_procs * 4)] += atoi(lastReadWord.c_str());

		    cpu.siblings[current_procs].load = 1 - ((sibling_time[(current_procs * 4) + 2]
			    - sibling_time[(current_procs * 4) + 3]) * 1.0 / (sibling_time[(current_procs * 4)]
			    - sibling_time[(current_procs * 4) + 1]));
		    sibling_time[(current_procs * 4) + 1] = sibling_time[(current_procs * 4)];
		    sibling_time[(current_procs * 4) + 3] = sibling_time[(current_procs * 4) + 2];
		    current_procs++;
		    sstream.str("");
		    sstream << "cpu" << current_procs;
		}
	    }
	    cpuStateFile.close();

	    cpu.header.stamp = ros::Time::now();
	    cpu.header.frame_id = "base_link";
	    cpu_pub.publish(cpu);
	    diagArray.header.stamp = ros::Time::now();
	    diagArray.header.frame_id = "base_link";
	    diagArray.status.clear();
	    diagArray.status.push_back(getCPUStatus());
	    //diagArray.status.push_back(getSwapStatus());
	    diagnostic_pub.publish(diagArray);
	    ros::spinOnce();
	    loop_time.sleep();

	}

    }
}
