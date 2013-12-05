#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include "flight_control/UAV_Manager.h"

//double ang = 0.0, lin = 0.0;

double gas = 0.0;
double lin_x = 0.0;
double yaw = 0.0;

ros::Publisher uav_manager_pub;
flight_control::UAV_Manager uav_man;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  gas = joy->axes[1] * 1.0;
  lin_x = joy->axes[3] * 1.0;
  yaw = 0.0;
  if (joy->buttons[0])
  {
    yaw = -1.0;
  }
  if (joy->buttons[2])
  {
    yaw = 1.0;
  }

  if (joy->buttons[8] || joy->buttons[9])
  {
    if (joy->buttons[8])
    {
      //start
      uav_man.motorsOn = true;
    }
    if (joy->buttons[9])
    {
      //stop
      uav_man.motorsOn = false;
    }
    uav_manager_pub.publish(uav_man);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_control");
  ros::NodeHandle nh_;
  ros::Subscriber information_joy;

  ros::Publisher vel_pub_;
  geometry_msgs::Twist drive;

  vel_pub_ = nh_.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
  uav_manager_pub = nh_.advertise<flight_control::UAV_Manager> ("/uav_manager", 1);
  information_joy = nh_.subscribe<sensor_msgs::Joy> ("/joy", 1, joyCallback);
  ros::Rate loop_rate(10);
  while (nh_.ok())
  {
    drive.angular.z = yaw;
    drive.linear.x = lin_x;
    drive.linear.z = gas;
    ROS_INFO_STREAM(drive);
    vel_pub_.publish(drive);
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
