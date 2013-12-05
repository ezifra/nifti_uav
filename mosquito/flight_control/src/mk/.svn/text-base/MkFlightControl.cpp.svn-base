/*
 * MkFlightControl.cpp
 *
 *  Created on: Oct 17, 2011
 *      Author: tlinder
 *  Modified on: Jul 12, 2012 by: Finley
 *    for Saitek Joystick
 *
 */

#include "MkFlightControl.h"

using namespace std;

MkFlightControl::MkFlightControl() :
    MIN_LINEAR_VEL_X(-1.0), MAX_LINEAR_VEL_X(1.0), MIN_LINEAR_VEL_Y(0.0), MAX_LINEAR_VEL_Y(0.0), MIN_LINEAR_VEL_Z(0.0), MAX_LINEAR_VEL_Z(
        1.0), MIN_ANGULAR_VEL_Z(-1.0), MAX_ANGULAR_VEL_Z(1.0), RAWmsgSpeed(2), imu_pub(
        nodeHandle.advertise<sensor_msgs::Imu>("/imu", 2, true)), navFix_pub(
        nodeHandle.advertise<sensor_msgs::NavSatFix>("/gps_fix", 2, true)), pressure_height_pub(
        nodeHandle.advertise<sensor_msgs::Range>("/pressure_height", 2, true)), status_pub(
        nodeHandle.advertise<flight_control::UAV_Status>("/uav_status", 2, true)), battery_pub(
        nodeHandle.advertise<monitoring_msgs::Battery>("/monitoring/batteryInfo", 2, true))
{
  init();
}

MkFlightControl::~MkFlightControl()
{
  if (mkSerial.isOpen())
  {

    //motors(false);

    mkSerial.ExternControl.free = 0;
    mkSerial.ExternControl.Digital[0] = 0;
    mkSerial.ExternControl.Gas = 0;
    mkSerial.ExternControl.Height = 0;
    mkSerial.ExternControl.RemoteButtons = 0;
    mkSerial.ExternControl.Config = 0x00;
    mkSerial.ExternControl.Nick = 0;
    mkSerial.ExternControl.Frame = 0;
    mkSerial.ExternControl.Yaw = 0;
    mkSerial.ExternControl.Roll = 0;

    memcpy((char *)&mkSerial.c_Data, (char *)&mkSerial.ExternControl, sizeof(mkSerial.ExternControl));
    string tempString = MKHandler::make_Frame( FC_ADDRESS,'b', mkSerial.c_Data, sizeof(mkSerial.ExternControl));
    mkSerial.writeData(tempString);

    if (mkSerial.closePort())
    {
      ROS_INFO("Close serial communication was succesfull");
    }
    else
    {
      ROS_INFO("Close serial communication failed");
    }
  }
}

bool MkFlightControl::init()
{
//pressure_height_pub(
//                  nodeHandle.advertise<sensor_msgs::Range> (
//                                  "/pressure_height", 2, true)), status_pub(
//                  nodeHandle.advertise<flight_control::UAV_Status> (
//                                  "/uav_status", 2, true)) {
  nodeHandle.param<double>("FlightControl/MIN_LINEAR_VEL_X", MIN_LINEAR_VEL_X, -1.0);
  nodeHandle.param<double>("FlightControl/MAX_LINEAR_VEL_X", MAX_LINEAR_VEL_X, 1.0);
  nodeHandle.param<double>("FlightControl/MIN_LINEAR_VEL_Y", MIN_LINEAR_VEL_Y, -1.0);
  nodeHandle.param<double>("FlightControl/MAX_LINEAR_VEL_Y", MAX_LINEAR_VEL_Y, 1.0);
  nodeHandle.param<double>("FlightControl/MIN_LINEAR_VEL_Z", MIN_LINEAR_VEL_Z, -1.0);
  nodeHandle.param<double>("FlightControl/MAX_LINEAR_VEL_Z", MAX_LINEAR_VEL_Z, 1.0);
  nodeHandle.param<double>("FlightControl/MIN_ANGULAR_VEL_Z", MIN_ANGULAR_VEL_Z, -1.0);
  nodeHandle.param<double>("FlightControl/MAX_ANGULAR_VEL_Z", MAX_ANGULAR_VEL_Z, 1.0);
  nodeHandle.param<int>("FlightControl/RAWmsgSpeed", RAWmsgSpeed, 2);

  nodeHandle.param<string>("FlightControl/imu", _imu, "/imu");
  imu_pub = nodeHandle.advertise<sensor_msgs::Imu>(_imu, 2, true);

  nodeHandle.param<string>("FlightControl/gps_fix", _gps_fix, "/gps_fix");
  navFix_pub = nodeHandle.advertise<sensor_msgs::NavSatFix>(_gps_fix, 2, true);

  nodeHandle.param<string>("FlightControl/pressure_height", _pressure_height, "/pressure_height");
  pressure_height_pub = nodeHandle.advertise<sensor_msgs::Range>("/pressure_height", 2, true);

  nodeHandle.param<string>("FlightControl/batteryInfo", _battery, "/monitoring/batteryInfo");
  battery_pub = nodeHandle.advertise<monitoring_msgs::Battery>("/monitoring/batteryInfo", 2, true);

  nodeHandle.param<string>("FlightControl/uav_manager", _uav_manager, "/uav_manager");
  uav_manager_sub = nodeHandle.subscribe(_uav_manager, 1, &MkFlightControl::uav_manager_cb, this);

  nodeHandle.param<string>("FlightControl/cmd_vel", _cmd_vel, "/cmd_vel");
  cmd_LINEAR_VEL_sub = nodeHandle.subscribe("/cmd_vel", 1, &MkFlightControl::cmd_vel_cb, this);

  nodeHandle.param<string>("FlightControl/serial_port_name", _serialport_name, "/dev/communication/flightcontrol");
  mkSerial.setSerialDev(_serialport_name);

  mkSerial.openPort();
  string msg = MKHandler::get_SelectRedirect2FC();
  mkSerial.writeData(msg);
  usleep(10000);

  return true;
}

float MkFlightControl::getPitch(geometry_msgs::Quaternion quat)
{
  return atan2(2 * (quat.y * quat.z + quat.w * quat.x),
               quat.w * quat.w - quat.x * quat.x - quat.y * quat.y + quat.z * quat.z);
}

float MkFlightControl::getYaw(geometry_msgs::Quaternion quat)
{
  return asin(-2 * (quat.x * quat.z - quat.w * quat.y));
}

float MkFlightControl::getRoll(geometry_msgs::Quaternion quat)
{
  return atan2(2 * (quat.x * quat.y + quat.w * quat.z),
               quat.w * quat.w + quat.x * quat.x - quat.y * quat.y - quat.z * quat.z);
}

void MkFlightControl::uav_manager_cb(const flight_control::UAV_Manager& uav_manager_msg)
{
  ROS_INFO_STREAM("Received UAV_Manager msg. " << uav_manager_msg);

  switch (uav_manager_msg.motorsOn)
  {
    case true:
      ROS_INFO("Enable Motors");
      motors(true);
      break;
    case false:
      if (uav_manager_msg.calib1)
      {
        //ROS_INFO("Calibration 1");
        calibration1(true);
        break;
      }

      if (uav_manager_msg.calib2)
      {
        //ROS_INFO("Calibration 2");
        calibration2(true);
        break;
      }

      ROS_INFO("Disable Motors");
      motors(false);
  }

  /** Check if the time stamp of the message is up-to-date.
   *  If yes perform the commands from the message,
   *  if not execute only those which are not time critical.
   *
   *  Hint: This assumes of course synchronized clocks on all computers!!!
   */
  if ((uav_manager_msg.header.stamp.sec - ros::Time::now().sec) <= 2.0)
  {

  }
  else
  {

  }

}

void MkFlightControl::cmd_vel_cb(const geometry_msgs::Twist& cmd_vel)
{

  ROS_INFO_STREAM("Received geometry_msgs::Twist msg. " << endl << cmd_vel);
  //Compute NIck
  //x= 1.0[m/s] => Nick 10 [deg]
  //x= -1.0[m/s] => Nick -10 [deg]
  double temp_x = cmd_vel.linear.x < MAX_LINEAR_VEL_X ? cmd_vel.linear.x : MAX_LINEAR_VEL_X;
  temp_x = cmd_vel.linear.x < MIN_LINEAR_VEL_X ? MIN_LINEAR_VEL_X : cmd_vel.linear.x;

  double temp_y = cmd_vel.linear.y < MAX_LINEAR_VEL_Y ? cmd_vel.linear.y : MAX_LINEAR_VEL_Y;
  temp_y = cmd_vel.linear.y < MIN_LINEAR_VEL_Y ? MIN_LINEAR_VEL_Y : cmd_vel.linear.y;

  double temp_z = cmd_vel.linear.z < MAX_LINEAR_VEL_Z ? cmd_vel.linear.z : MAX_LINEAR_VEL_Z;
  temp_z = cmd_vel.linear.z < MIN_LINEAR_VEL_Z ? MIN_LINEAR_VEL_Z : cmd_vel.linear.z;
  temp_z = 0.5 * temp_z + 0.5; //mapping [-1,1] to [0,1]

  double temp_yaw = cmd_vel.angular.z < MAX_ANGULAR_VEL_Z ? cmd_vel.angular.z : MAX_ANGULAR_VEL_Z;
  temp_yaw = cmd_vel.angular.z < MIN_ANGULAR_VEL_Z ? MIN_ANGULAR_VEL_Z : cmd_vel.angular.z;

  int nick = floor(temp_x * 20);
  int roll = floor(temp_y * 20);
  int yaw = floor(temp_yaw * 10);
  int gas = floor(temp_z * 100 * 2.5);

  mkSerial.ExternControl.free = 2;
  //  mk.ExternControl.Digital[0] = 0x55;
  mkSerial.ExternControl.Gas = (uint8_t)gas;
  //  mk.ExternControl.Height = 0;
  //  mk.ExternControl.RemoteButtons = 0;
  //  mk.ExternControl.Config = 0x00;
  mkSerial.ExternControl.Nick = (int8_t)nick;
  mkSerial.ExternControl.Roll = (int8_t)roll;
  //  mk.ExternControl.Frame = 1;
  mkSerial.ExternControl.Yaw = (int8_t)yaw;

  ROS_INFO_STREAM("Nick " << (int)(mkSerial.ExternControl.Nick));
  ROS_INFO_STREAM("GAS " << (int)(mkSerial.ExternControl.Gas));
  ROS_INFO_STREAM("Roll " << (int)(mkSerial.ExternControl.Roll));
  ROS_INFO_STREAM("YAW " << (int)(mkSerial.ExternControl.Yaw));

  memcpy((char *)&mkSerial.c_Data, (char *)&mkSerial.ExternControl, sizeof(mkSerial.ExternControl));
  string tempString = MKHandler::make_Frame( FC_ADDRESS,'b', mkSerial.c_Data, sizeof(mkSerial.ExternControl));
  //ROS_INFO_STREAM(tempString);
  mkSerial.writeData(tempString);

}

void MkFlightControl::motors(bool enable)
{
  if (enable)
  {
    ROS_INFO("Enable Motors");
    mkSerial.ExternControl.free = 21;
    //    mk.ExternControl.Digital[0] = 0x55;
    //    mk.ExternControl.Gas = 0;
    //    mk.ExternControl.Height = 0;
    //    mk.ExternControl.RemoteButtons = 0;
    //    mk.ExternControl.Config = 0x00;
    //    mk.ExternControl.Nick = 0;
    mkSerial.ExternControl.Frame = 1;
    //    mk.ExternControl.Yaw = 0;
    //    mk.ExternControl.Roll = 0;

    memcpy((char *)&mkSerial.c_Data, (char *)&mkSerial.ExternControl, sizeof(mkSerial.ExternControl));
    string tempString = MKHandler::make_Frame( FC_ADDRESS,'b', mkSerial.c_Data, sizeof(mkSerial.ExternControl));

    mkSerial.writeData(tempString);
    sleep(1);

    /*	mk.ExternControl.free = 2;
     //    mk.ExternControl.Digital[0] = 0x55;
     //    mk.ExternControl.Gas = 0;
     //    mk.ExternControl.Height = 0;
     //    mk.ExternControl.RemoteButtons = 0;
     //    mk.ExternControl.Config = 0x00;
     //    mk.ExternControl.Nick = 0;
     mk.ExternControl.Frame = 1;
     //    mk.ExternControl.Yaw = 0;
     //    mk.ExternControl.Roll = 0;

     memcpy((char *) &mk.c_Data, (char *) &mk.ExternControl,
     sizeof(mk.ExternControl));
     tempString = MKHandler::make_Frame('b', FC_ADDRESS, mk.c_Data,
     sizeof(mk.ExternControl));

     sleep(1);

     mk.writeData(tempString);*/
  }
  else
  {
    ROS_INFO("Disable Motors");
    mkSerial.ExternControl.free = 42;
//		mk.ExternControl.Digital[0] = 0;
//		mk.ExternControl.Gas = 0;
//		mk.ExternControl.Height = 0;
//		mk.ExternControl.RemoteButtons = 0;
//		mk.ExternControl.Config = 0x00;
//		mk.ExternControl.Nick = 0;
//		mk.ExternControl.Frame = 0;
//		mk.ExternControl.Yaw = 0;
//		mk.ExternControl.Roll = 0;

    memcpy((char *)&mkSerial.c_Data, (char *)&mkSerial.ExternControl, sizeof(mkSerial.ExternControl));
    string tempString = MKHandler::make_Frame( FC_ADDRESS,'b', mkSerial.c_Data, sizeof(mkSerial.ExternControl));

    mkSerial.writeData(tempString);
    mkSerial.flushData();
    sleep(2);
  }

}

void MkFlightControl::calibration1(bool enable)
{
  if (enable)
  {
    ROS_INFO("Calibration 1");
    mkSerial.ExternControl.free = 50; // 50 is for activating the zero value calibration
    mkSerial.ExternControl.Frame = 1;

    memcpy((char *)&mkSerial.c_Data, (char *)&mkSerial.ExternControl, sizeof(mkSerial.ExternControl));
    string tempString = MKHandler::make_Frame( FC_ADDRESS,'b', mkSerial.c_Data, sizeof(mkSerial.ExternControl));
    mkSerial.writeData(tempString);

    sleep(1);
    mkSerial.ExternControl.free = 0;
    mkSerial.ExternControl.Frame = 1;
    memcpy((char *)&mkSerial.c_Data, (char *)&mkSerial.ExternControl, sizeof(mkSerial.ExternControl));
    tempString = MKHandler::make_Frame( FC_ADDRESS,'b', mkSerial.c_Data, sizeof(mkSerial.ExternControl));

    mkSerial.writeData(tempString);

  }
}

void MkFlightControl::calibration2(bool enable)
{
  if (enable)
  {
    ROS_INFO("Calibration 2");
    mkSerial.ExternControl.free = 51; // 51 is for saving the ACC's zero value
    mkSerial.ExternControl.Frame = 1;

    memcpy((char *)&mkSerial.c_Data, (char *)&mkSerial.ExternControl, sizeof(mkSerial.ExternControl));
    string tempString = MKHandler::make_Frame( FC_ADDRESS,'b', mkSerial.c_Data, sizeof(mkSerial.ExternControl));
    mkSerial.writeData(tempString);

    sleep(1);
    mkSerial.ExternControl.free = 0;
    mkSerial.ExternControl.Frame = 1;
    memcpy((char *)&mkSerial.c_Data, (char *)&mkSerial.ExternControl, sizeof(mkSerial.ExternControl));
    tempString = MKHandler::make_Frame( FC_ADDRESS,'b', mkSerial.c_Data, sizeof(mkSerial.ExternControl));
    mkSerial.writeData(tempString);

  }
}

void MkFlightControl::SetUpReadRAWSpeed(unsigned int readSpeed)
{
  mkSerial.ExternControl.free = 2; //enables uninterrupted DebugData sending
  memcpy((char *)&mkSerial.c_Data, (char *)&mkSerial.ExternControl, sizeof(mkSerial.ExternControl));
  string tempString = MKHandler::make_Frame( FC_ADDRESS,'b', mkSerial.c_Data, sizeof(mkSerial.ExternControl));
  mkSerial.writeData(tempString);
  mkSerial.flushData();

  mkSerial.c_Data[0] = readSpeed; //for how long the data is sent in peusdo-ms
  mkSerial.writeData(MKHandler::make_Frame( FC_ADDRESS,'d', mkSerial.c_Data, 1));
}

void MkFlightControl::watchdog()
{
  ROS_DEBUG_STREAM("[MkFlightControl::watchdog] Watchdog message. Reseting WD-Timer.");
  mkSerial.c_Data[0] = 0; //for how long the data is sent in peusdo-ms
  if (mkSerial.ExternControl.Config != 1)
  {
    //mk.ExternControl.Config != 1;
    memcpy((char *)&mkSerial.c_Data, (char *)&mkSerial.ExternControl, sizeof(mkSerial.ExternControl));
    string tempString = MKHandler::make_Frame( FC_ADDRESS,'b', mkSerial.c_Data, sizeof(mkSerial.ExternControl));
    mkSerial.writeData(tempString);
    mkSerial.flushData();

  }
  mkSerial.writeData(MKHandler::make_Frame( ALL_ADDRESS,'@', mkSerial.c_Data, 1));
}

int counter;
void MkFlightControl::update_all()
{
//  string tempString = MKHandler::make_Frame( FC_ADDRESS,'v', mkSerial.c_Data, sizeof(mkSerial.ExternControl));
//  ROS_INFO("--> %s", tempString.c_str());

  counter++;
  if (0 == (counter % (int)floor(frequency)))
  {
    //Watchdog
    ROS_DEBUG_STREAM("[MkFlightControl::update_all] Setup: Requesting RAW sensor messages with speed = "<<RAWmsgSpeed);
    SetUpReadRAWSpeed(RAWmsgSpeed);
    watchdog();
  }
  //Request the Sensor data from UAV

  receivedMsg = mkSerial.readData();

  //Transform the RAW Sensor Data
  if (receivedMsg.size() > 0)
  {
    mkSerial.parse_MK_Data(receivedMsg);
    //		ROS_INFO_STREAM("Voltage " << mk.rawSensorData.Voltage);
    //		ROS_INFO_STREAM("YawGyro "<<mk.rawSensorData.YawGyro);

    //Publish ROS like
    //!IMU
    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = "uav_base_link";
    imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(mkSerial.rawSensorData.MagneticX,
                                                              mkSerial.rawSensorData.MagneticY,
                                                              mkSerial.rawSensorData.MagneticZ);
    imu.angular_velocity.x = mkSerial.rawSensorData.RollGryoRate;
    imu.angular_velocity.y = mkSerial.rawSensorData.NickGyroRate;
    imu.angular_velocity.z = mkSerial.rawSensorData.YawGyroRate;
    imu.linear_acceleration.x = mkSerial.rawSensorData.AccX;
    imu.linear_acceleration.y = mkSerial.rawSensorData.AccY;
    imu.linear_acceleration.z = mkSerial.rawSensorData.AccZ;
    imu_pub.publish(imu);

    //!GPS
    navFix.header.stamp = ros::Time::now();
    imu.header.frame_id = "uav_base_link";
    navFix.position_covariance_type = 0;
    navFix.altitude = 0;
    navFix.longitude = 0;
    navFix.latitude = 0;
    navFix.status.status = -1;
    navFix.status.service = 1;
    navFix_pub.publish(navFix);

    //!Barometric pressure
    pressure_height.header.stamp = ros::Time::now();
    pressure_height.header.frame_id = "uav_base_link";
    pressure_height.radiation_type = 2;
    pressure_height.field_of_view = 0;
    pressure_height.min_range = -1000;
    pressure_height.max_range = 1000;
    pressure_height.range = mkSerial.rawSensorData.Height_Value;
    pressure_height_pub.publish(pressure_height);

    //!UAV Status
    status.header.stamp = ros::Time::now();
    status.header.frame_id = "uav_base_link";
    status.battery_voltage = 100 * (mkSerial.rawSensorData.Voltage - 13.2) / 3.4;
    status.battery_capacity = mkSerial.rawSensorData.Capacity;
    status.current = mkSerial.rawSensorData.Current;
    status.pressure = mkSerial.rawSensorData.Height_Value;
    status.rc_radio_level = mkSerial.rawSensorData.Receiver_Level;
    status.nick = mkSerial.rawSensorData.AngleNick;
    status.roll = mkSerial.rawSensorData.AngleRoll;
    status.yaw = mkSerial.rawSensorData.Gyro_Compass;
    status_pub.publish(status);

    battery.currentBatteryLevel = status.battery_voltage;
    battery_pub.publish(battery);
  }
}

void MkFlightControl::run()
{

  if (mkSerial.isOpen())
  {
    //Set motor speed to zero
    //motors(false);

    // getting frequency
    nodeHandle.param<double>("FlightControl/loop_rate", frequency, 40.0);
    ros::Rate loop_rate(frequency);
    ROS_INFO("Looping at %fHz.", frequency);

    ROS_INFO_STREAM("Running!");
    SetUpReadRAWSpeed(RAWmsgSpeed);
    watchdog();

    // loop
    while (nodeHandle.ok())
    {
      update_all();
      loop_rate.sleep();
      ros::spinOnce();
    }

    std::cout << "Motors Off" << std::endl;
    motors(false);
  }
  std::cout << "Finished!" << std::endl;
}
