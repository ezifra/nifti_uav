The monitoring package is meant to support remotely monitoring computers.
It includes battery level, CPU load, memory load, system temperature and WiFi strength.

Nodes:
- monitoring_batteryInfo: 	provides the current status of the battery uses the acpi, Topics: /monitoring/batteryInfo(Battery.msg) /diagnostics
- monitoring_batteryInfo2: 	provides the current status of the battery uses the /diagnostics, Topics: /monitoring/batteryInfo(Battery.msg) /diagnostics
- monitoring_cpuInfo:		provides the current status of all CPU and the total system load, Topics: /monitoring/cpuInfo(CPU.msg) /diagnostics 
- monitoring_memoryInfo:	provides information about the used/free memory, Topics: /monitoring/memoryInfo(Memory.msg) /diagnostics
- monitoring_thermalInfo:	provides the system temperature, Topics: /monitoring/thermalInfo(Temperature.msg) /diagnostics
- monitoring_wifiLink:		provides information about the current WiF states, Topics: /monitoring/wifiLink(WiFi.msg) /diagnostics

USAGE:
The idea is to use this package to remotely monitor a PC/computer/laptop.
To do so, the mentioned nodes must be executed on the PC which you would like to monitor.
For example, if we would like to monitor the CPUs of the robot-computer with an operator PC,
we would start on side of the robot the monitoring_cpuInfo node (rosrun monitoring monitoring_cpuInfo).
On side of the operator we can now read the CPU states, either by reading the corresponding 
topic (/monitoring/cpuInfo (CPU.msg)) (e.g. rostopic echo -c /monitoring/cpuInfo) 
or by using the /diagnostics messages. To use the diagnostic_msg just start runtime_monitore node 
(rosrun runtime_monitor monitor).

For easy testing the package includes a launch files (monitoring.launch) to demonstrate the functionality (only local). 
