/*
 * This node publishes the /monitoring/batteryInfo.
 * To do so it parses the messages from the /diagnostics_agg topic. The problem is
 * that not all the data in the message are now available. Therefore the
 * status will be always set to 'Unknown' and the batteryTimeLeft will be
 * always 0.
 *
 * @author Jan Brabec
 */

#include <ros/ros.h>
#include <monitoring_msgs/Battery.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <boost/lexical_cast.hpp>

typedef std::vector<diagnostic_msgs::DiagnosticStatus>::const_iterator DiagnosticArrayConstIterator;

std::string namePrefix;
ros::NodeHandlePtr nodeHandle;
size_t correctIndexInArray = std::string::npos;


/**
 * Compares the string with the expected name of the battery status in the diagnostics array.
 */
bool isBatteryStatus(const std::string& statusName) {
    return statusName == namePrefix + ": Battery";
}


void getNamePrefix() {
    if (!nodeHandle->getParam("batteryDiagnosticsPublisherNode", namePrefix)) {
        ROS_INFO_STREAM("No prefix parameter specified. using: nifti_robot_driver");
        namePrefix = "nifti_robot_driver";
    }
}


/**
 * Tries to find the index of the battery status in the diagnostics array. The problem
 * is that there can be different types of arrays published and some might not contain the
 * battery information. Therefore this function returns true if the battery status has been
 * successfully found and false otherwise.
 */
bool tryToFindIndexInArray(const diagnostic_msgs::DiagnosticArrayConstPtr& msg) {
    DiagnosticArrayConstIterator it = msg->status.begin();
    size_t index = 0;
    for (;it < msg->status.end(); ++it, ++index) {
        if (isBatteryStatus(it->name)) {
            correctIndexInArray = index;
            ROS_INFO_STREAM("Correct status in the DiagnosticArray has been found.");
            return true;
        }
    }
    ROS_WARN_STREAM("Couldn't find the status in the DiagnosticArray. This may be normal in the" <<
                    " first few seconds.");
    return false;
}


/**
 * Gets the battery status from the diagnostics array. Returns the end iterator if the
 * status is not in the array.
 */
DiagnosticArrayConstIterator getBatteryStatus(const diagnostic_msgs::DiagnosticArrayConstPtr& msg) {
    if (correctIndexInArray == std::string::npos && !tryToFindIndexInArray(msg)) {
        return msg->status.end(); //Couldn't find the correct index.
    }
    if (msg->status.size() <= correctIndexInArray) {
        return msg->status.end(); //This array is smaller than the correct one.
    }
    DiagnosticArrayConstIterator it = msg->status.begin() + correctIndexInArray;
    if (isBatteryStatus(it->name)) {
        return it;
    } else {
        return msg->status.end(); //Different array than the one we are expecting.
    }
}


/**
 * Publishes the /monitoring/batteryInfo topic. Takes care of the whole creation of the message.
 */
void publishBatteryInfo(double batteryLevel) {
    static ros::Publisher batteryPub =
            nodeHandle->advertise<monitoring_msgs::Battery> ("/monitoring/batteryInfo", 10);
    monitoring_msgs::Battery bat;
    bat.header.stamp = ros::Time::now();
    bat.header.frame_id = "base_link";
    bat.batteryTimeLeft = 0;
    bat.currentBatteryLevel = batteryLevel;
    bat.status = "Unknown";
    batteryPub.publish(bat);
}


/**
 * This callback is invoked every time a new diagnostics message arrives on the /diagnostics
 * topic.
 */
void diagnosticsCallback(const diagnostic_msgs::DiagnosticArrayConstPtr& msg) {
    DiagnosticArrayConstIterator status = getBatteryStatus(msg);
    if (status != msg->status.end()) {
        publishBatteryInfo(boost::lexical_cast<double>(status->values[0].value));
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "batteryInfo");
    nodeHandle = ros::NodeHandlePtr(new ros::NodeHandle("~"));
    getNamePrefix();
    ROS_INFO("Subscribing to /diagnostics ...");
    ros::Subscriber diagnosticsSub = nodeHandle->subscribe("/diagnostics", 1, diagnosticsCallback);
    ros::spin();
    return 0;
}

