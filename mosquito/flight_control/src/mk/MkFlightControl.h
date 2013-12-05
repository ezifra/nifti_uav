/*
 * MkFlightControl.h
 *
 *  Created on: Oct 17, 2011
 *      Author: tlinder
 *  Edited on: Jul 12, 2012 by: Finley
 *    for Saitek Joystick
 *
 */

#ifndef MKFLIGHTCONTROL_H_
#define MKFLIGHTCONTROL_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/tf.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Range.h"
#include "flight_control/UAV_Manager.h"
#include "flight_control/UAV_Status.h"
#include "MkSerial.h"
#include "MKHandler.h"
#include "MKDefines.h"
#include "monitoring_msgs/Battery.h"
#include <string>

using namespace std;

class MkFlightControl
{
public:
  /**
   * Default constructor of the MkFlightControl
   */
  MkFlightControl();

  /**
   * Destructor of the MkFlightControl
   */
  ~MkFlightControl();

  /**
   * Main execution loop
   */
  void run();

  /**
   * Truns the motors on or off in respect to the argument.
   * enable == true turns motors on, else motors are off
   */
  void motors(bool enable);
  void calibration1(bool enable);
  void calibration2(bool enable);

  /**
   *  Internal update function which is updating and publishing the uav_topics
   *  [e.g. IMU, GPS, Barometric pressure, UAV Status]
   */
  void update_all();

protected:
  double MIN_LINEAR_VEL_X;
  double MAX_LINEAR_VEL_X;
  double MIN_LINEAR_VEL_Y;
  double MAX_LINEAR_VEL_Y;
  double MIN_LINEAR_VEL_Z;
  double MAX_LINEAR_VEL_Z;
  double MIN_ANGULAR_VEL_Z;
  double MAX_ANGULAR_VEL_Z;

  /**
   * MicroKopter Serial handler
   */
  MkSerial mkSerial;

  /**
   *  Loop frequency for run() and updated_all() functions. Default is 40.0 Hz.
   */
  double frequency;

  /**
   * Reading speed for flight control board messages (pseudo-ms till next sensor messages)
   */
  int RAWmsgSpeed;

  /**
   * Received "RAW" message from the Flight Controller
   */
  std::string receivedMsg;

  /**
   *  Parameter for the uav_manager topic. Setup with default value or by parameter server
   */
  string _uav_manager;

  /**
   *  Parameter for the imu topic. Setup with default value or by parameter server
   */
  string _imu;

  /**
   *  Parameter for the gps_fix topic. Setup with default value or by parameter server
   */
  string _gps_fix;

  /**
   *  Parameter for the pressure_height topic. Setup with default value or by parameter server
   */
  string _pressure_height;

  /**
   *  Parameter for the battery topic. Setup with default value or by parameter server
   */
  string _battery;

  /**
   * Parameter for the command velocity topic. Setup with default value or by parameter server
   */
  string _cmd_vel;

  /**
   * Parameter for the selected serial port which is in use to comunicated to the flight controller board.
   */
  string _serialport_name;

  /**
   * Private initialization function, fetching the arguments from the parameter server
   */
  bool init();

  /**
   * Sets up the reading/pulling speed to the flight controller. Argument is in pseudo ms.
   */
  void SetUpReadRAWSpeed(unsigned int readSpeed);

  /**
   * For Flight controller boards with activated watchdog behavior.
   * Each call of this function is sending once a watchdog signal to the Flight Controller Board.
   */
  void watchdog();


  float getPitch(geometry_msgs::Quaternion quat);

  float getYaw(geometry_msgs::Quaternion quat);

  float getRoll(geometry_msgs::Quaternion quat);

  // ============= ROS stuff =============

  //! IMU values
  sensor_msgs::Imu imu;

  //! GPS Values
  sensor_msgs::NavSatFix navFix;;

  //! public NodeHandle
  ros::NodeHandle nodeHandle;

  //!Barometric pressure height Value
  sensor_msgs::Range pressure_height;

  //! UAV Status msg
  flight_control::UAV_Status status;
  monitoring_msgs::Battery battery;

  // publishers
  //! IMU publisher
  ros::Publisher imu_pub;
  //! GPS FIX publisher
  ros::Publisher navFix_pub;
  //! Barometric pressure height Value publisher
  ros::Publisher pressure_height_pub;
  //! UAV status publisher
  ros::Publisher status_pub;
  ros::Publisher battery_pub;

  // subscribers
  //! Subscriber to uav manager messages
  ros::Subscriber uav_manager_sub;

  //! Subscriber to geometry_msgs::Twist& cmd_vel
  ros::Subscriber cmd_LINEAR_VEL_sub;

  // callbacks
  //! Callback for UAV Manager msg
  void uav_manager_cb(const flight_control::UAV_Manager& uav_manager_msg);

  //! Callback for const geometry_msgs::Twist& cmd_vel
  void cmd_vel_cb(const geometry_msgs::Twist& cmd_vel);

};

#endif /* MKFLIGHTCONTROL_H_ */
