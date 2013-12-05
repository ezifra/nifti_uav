/*
 * Just mocks the hardware diagnostics for battery monitoring debugging.
 */

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>


void batteryDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    stat.summary(2, "No battery information.");
    stat.add("battery level", 0.93);
}

void uselessNoiseDiag1(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    stat.add("something1", "darkside1");
}

void uselessNoiseDiag2(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    stat.add("something2", "darkside2");
}

void uselessNoiseDiag3(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    stat.add("something3", "darkside3");
}

void uselessNoiseDiag4(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    stat.add("something4", "darkside4");
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "diagnosticsMock");
    ros::NodeHandle nodeHandle;
    diagnostic_updater::Updater diagnostic_pub;
    diagnostic_pub.setHardwareID("none");
    diagnostic_pub.add("Noise1", uselessNoiseDiag1);
    diagnostic_pub.add("Noise2", uselessNoiseDiag2);
    diagnostic_pub.add("Battery", batteryDiagnostic);
    diagnostic_pub.add("Noise3", uselessNoiseDiag3);
    diagnostic_pub.add("Noise4", uselessNoiseDiag4);

    std::string hz = "0.5";
    ros::Rate loop_time(atof(hz.c_str()));

    while (nodeHandle.ok()) {
        diagnostic_pub.update();
        ros::spinOnce();
    	loop_time.sleep();
    }

    return EXIT_SUCCESS;
}
