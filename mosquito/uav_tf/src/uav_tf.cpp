/*
 * uav_tf.cpp
 *
 *  Created on: Oct 26, 2011
 *      Author: tlinder
 */
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "usbi2c/sonar_params.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "flight_control/UAV_Status.h"
#include "filter/WindowMeanFilter.h"
//#include "advanced_filters/WindowMeanFilter.h"

double frequency;
int windowSizeSonar, windowSizeNick, windowSizeRoll;
double thresholdSonar, thresholdNick, thresholdRoll;
sensor_msgs::Range _range;
sensor_msgs::Range _sonar_height;
sensor_msgs::Imu _imu;
flight_control::UAV_Status _uav_status;
tf::Transform transform_uav_base_link;
tf::Transform transform_uav_cam_front;
tf::Transform transform_uav_cam_down;
usbi2c::sonar_params _sonar;

WindowMeanFilter* nickFilter;
WindowMeanFilter* rollFilter;

void heightCallback(const sensor_msgs::Range::ConstPtr& height)
{
  _range = *height;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  _imu = *imu;
}

void uavStatusCallback(const flight_control::UAV_Status::ConstPtr& msg)
{
  _uav_status = *msg;
}

void sonarCallback(const sensor_msgs::Range::ConstPtr& msg)
{
  _sonar_height = *msg;
}

void update_all(tf::TransformBroadcaster& tf_broadcaster)
{

  transform_uav_base_link.setOrigin(tf::Vector3(0.0, 0.0, _sonar_height.range));
  transform_uav_base_link.setRotation(tf::createQuaternionFromRPY(rollFilter->run(_uav_status.roll) * -M_PI / 180.0,
                                                                  nickFilter->run(_uav_status.nick) * M_PI / 180.0, 0.0));//uav_status.yaw*M_PI/180.0));
  tf_broadcaster.sendTransform(tf::StampedTransform(transform_uav_base_link, ros::Time::now(), "/base_stabilized",
                                                    "/uav_base_link"));

  transform_uav_cam_front.setOrigin(tf::Vector3(0.05, 0, -0.1));
  transform_uav_cam_front.setRotation(tf::createQuaternionFromRPY(-110 * M_PI / 180.0, 0 * M_PI / 180.0, -90 * M_PI / 180.0));//(-60 * M_PI / 180.0, 180 * M_PI / 180.0, -90 * M_PI / 180.0)
  tf_broadcaster.sendTransform(tf::StampedTransform(transform_uav_cam_front, ros::Time::now(), "/uav_base_link", "/uav_cam_front"));

  transform_uav_cam_down.setOrigin(tf::Vector3(0, -0.1, -0.05));
  transform_uav_cam_down.setRotation(tf::createQuaternionFromRPY(0, 180 * M_PI / 180.0, 90 * M_PI / 180.0));
  tf_broadcaster.sendTransform(tf::StampedTransform(transform_uav_cam_down, ros::Time::now(), "/uav_base_link", "/uav_cam_down"));
  //ROS_INFO("Send Transform");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_tf");
  ros::NodeHandle nh_;
  ros::Subscriber height_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber uav_status_sub;
  ros::Subscriber sonar_sub = nh_.subscribe("/sonar_height", 1, sonarCallback);
  tf::TransformBroadcaster tf_broadcaster;

  height_sub = nh_.subscribe<sensor_msgs::Range> ("/pressure_height", 1, heightCallback);
  imu_sub = nh_.subscribe<sensor_msgs::Imu> ("/imu", 1, imuCallback);
  uav_status_sub = nh_.subscribe<flight_control::UAV_Status> ("/uav_status", 1, uavStatusCallback);

  // getting frequency
  nh_.param<double> ("loop_rate", frequency, 40.0);
  ros::Rate loop_rate(frequency);
  ROS_INFO("Looping at %fHz.", frequency);

  nh_.param<int> ("window_size_nick", windowSizeNick, 3);
  nh_.param<double> ("threshold_nick", thresholdNick, 15.0);
  nickFilter = new WindowMeanFilter(windowSizeNick, thresholdNick);

  nh_.param<int> ("window_size_roll", windowSizeRoll, 3);
  nh_.param<double> ("threshold_roll", thresholdRoll, 15.0);
  rollFilter = new WindowMeanFilter(windowSizeRoll, thresholdRoll);

  ROS_INFO_STREAM("Running!");

  // loop
  while (nh_.ok())
  {
    update_all(tf_broadcaster);
    loop_rate.sleep();
    ros::spinOnce();
  }
}
