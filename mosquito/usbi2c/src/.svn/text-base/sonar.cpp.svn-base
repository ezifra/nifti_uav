/*********************************************************************
* Software License Agreement (BSD License)
*
*  ROS driver for USB-I2C communications module
*  See http://www.robot-electronics.co.uk/htm/usb_i2c_tech.htm
*  Copyright (c) 2010, Bob Mottram
*  fuzzgun@gmail.com
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include "usbi2c/sonar_params.h"
#include "serialport.cpp"

#include "sensor_msgs/Range.h"
#include "filter/ContingencyAnalysisWindowMeanFilter.h"
#include "filter/OutlierRejectionFilter.h"


#define I2CD_CMD 0x53  // direct I2C control command
#define I2C_CMD  0x55  // registered I2C control command
#define CM01_CMD 0x5a  // CM01 command

#define TRANSMISSION_PING         1
#define TRANSMISSION_READ         2
#define TRANSMISSION_GAIN         3
#define TRANSMISSION_CHIP_WRITE   4
#define TRANSMISSION_CHIP_READ    5
#define TRANSMISSION_SET_ADDRESS  6

using namespace std;

const int ping_separation_mS = 50;
//ros::Time LastPinged = ros::Time::now();
ros::Time LastPinged;
int no_of_sensors = 0;
int sensor_index;  
long sensor_address = 0;
unsigned char sbuf[100];
int write_x = 0;
int CurrentTransmissionType;
ros::Publisher sonar_pub;
ros::Publisher sonar_filtered_pub;
bool transmission_complete = false;

ContingencyAnalysisWindowMeanFilter *cAFilter;
OutlierRejectionFilter *outlierFilter;

/*!
 * \brief write to the chip
 * \param serial serial port object
 * \param timeout_mS communications timeout in milliseconds
 * \return true if successful
 */
bool SonarChipWrite(
    SerialPort &serial,
    unsigned timeout_mS)
{
    sbuf[0] = I2C_CMD;
    sbuf[1] = 0x40;
    if ((write_x & 3) == 3)
        sbuf[2] = 0xfe;
    else
        sbuf[2] = 0xff;
    write_x++;

    CurrentTransmissionType = TRANSMISSION_CHIP_WRITE;

    if (serial.Out(sbuf, 3, timeout_mS) > 0)
        return true;
    else
        return false;
}

/*!
 * \brief read from the chip
 * \param serial serial port object
 * \param timeout_mS communications timeout in milliseconds
 * \return true if successful
 */
bool SonarChipRead(
    SerialPort &serial,
    unsigned timeout_mS)
{
    sbuf[0] = I2C_CMD;
    sbuf[1] = 0x41;

    CurrentTransmissionType = TRANSMISSION_CHIP_READ;

    if (serial.In(sbuf, 3, timeout_mS) > 0)
        return true;
    else
        return false;
}
               
/*!
 * \brief sends out a ping
 * \param serial serial port object
 * \param sensor_address address
 * \param timeout_mS communications timeout in milliseconds
 * \return true if successful
 */
bool SonarPing(
    SerialPort &serial,
    unsigned char sensor_address,
    unsigned timeout_mS)
{
    if ((sensor_address <= 0xfe) && (sensor_address >=0xe0)) {                      
        sbuf[0] = I2C_CMD;  // send sonar rangeing (uS) command
        sbuf[1] = sensor_address;
        sbuf[2] = 0x00;
        sbuf[3] = 0x01;
        sbuf[4] = 0x52;
               
        CurrentTransmissionType = TRANSMISSION_PING;

        if (serial.Out(sbuf, 5, timeout_mS) > 0)
            return true;
        else
            return false;
    }
    else {
        ROS_ERROR("SRF08 invalid sensor address: %d",(int)sensor_address);
        return false;
    }
}
               
/*!
 * \brief sends a request to read the time of flight of a previous ping
 * \param serial serial port object
 * \param sensor_address address
 * \param timeout_mS communications timeout in milliseconds
 * \return true if successful
 */
bool SonarRead(
    SerialPort &serial,
    unsigned char sensor_address,
    unsigned timeout_mS)
{
    if ((sensor_address <= 0xfe) && (sensor_address >= 0xe0)) {                      
        sbuf[0] = I2C_CMD;  // send sonar read command
        sbuf[1] = (unsigned char)(sensor_address+1);
        sbuf[2] = 0x00;
        sbuf[3] = 0x04;

        CurrentTransmissionType = TRANSMISSION_READ;
        if (serial.Out(sbuf, 4, timeout_mS) > 0)
            return true;
        else
            return false;
    }
    else {
        ROS_ERROR("SRF08 invalid sensor address: %d",(int)sensor_address);
        return false;
    }
}
               
/*!
 * \brief sets the sensor gain
 * \param serial serial port object
 * \param sensor_address address
 * \param timeout_mS communications timeout in milliseconds
 * \return true if successful
 */
bool SonarGainLimit(
    SerialPort &serial,
    unsigned char sensor_address,
    unsigned timeout_mS)
{
    if ((sensor_address <= 0xfe) && (sensor_address >=0xe0)) {                      
        sbuf[0] = I2C_CMD; // send gain limit
        sbuf[1] = sensor_address;
        sbuf[2] = 0x01;
        sbuf[3] = 0x01;
        sbuf[4] = 20;
               
        CurrentTransmissionType = TRANSMISSION_GAIN;
        if (serial.Out(sbuf, 5, timeout_mS) > 0)
            return true;
        else
            return false;
    }
    else {
        ROS_ERROR("SRF08 invalid sensor address: %d",(int)sensor_address);
        return false;
    }
}

/*!
 * \brief sets the address of a sensor
 * \param serial serial port object
 * \param sensor_address address
 * \param timeout_mS communications timeout in milliseconds
 * \return true if successful
 */
bool SetSensorAddress(
    SerialPort &serial,
    unsigned char sensor_address,
    unsigned timeout_mS)
{
    if ((sensor_address <= 0xfe) && (sensor_address >= 0xe0)) {
        sbuf[0] = CM01_CMD;
        sbuf[1] = 0x02;
        sbuf[2] = sensor_address;
        sbuf[3] = 0;

        CurrentTransmissionType = TRANSMISSION_SET_ADDRESS;
        if (serial.Out(sbuf, 4, timeout_mS) > 0)
            return true;
        else
            return false;
    }
    else {
        ROS_ERROR("SRF08 invalid sensor address: %d", (int)sensor_address);
        return false;
    }
}

/*!
 * \brief sends a ping command to a sensor
 * \param serial serial port object
 * \param timeout_mS communications timeout in milliseconds
 */
void PingNextSensor(
    SerialPort &serial,
    unsigned timeout_mS)
{
    // move to the next sensor in sequence
    sensor_index++;
    if (sensor_index >= no_of_sensors) sensor_index = 0;

    // ping the next sensor
    unsigned char sensor_address = (unsigned char)(0xe0 + (sensor_index * 2));
    SonarRead(serial, sensor_address, timeout_mS);
}

/*!
 * \brief periodically ping each sensor
 * \param serial serial port object
 * \param timeout_mS communications timeout in milliseconds
 * \param range_mm array in which to store range data
 */
void Update(
    SerialPort &serial,
    unsigned timeout_mS,
    int *range_mm)
{
    ros::Time t = ros::Time::now();
    double time_since_last_ping_sec = (t - LastPinged).toSec();
    if ((time_since_last_ping_sec*1000 > ping_separation_mS) && (sensor_address == 0)) {
        // issue a ping
        PingNextSensor(serial, timeout_mS);
        LastPinged = t;
    }
    else {
        // read the time of flight or other returned value
        const int bytes = 4;
        unsigned char comBuffer[bytes];
        int returned_bytes = serial.In(comBuffer, bytes, timeout_mS);
        if (returned_bytes > 0) {

            unsigned char sensor_address = (unsigned char)(0xe0 + (sensor_index * 2));

            switch (CurrentTransmissionType)
            {
                case TRANSMISSION_PING: {
                    SonarChipWrite(serial, timeout_mS);
                    break;
                }
                case TRANSMISSION_CHIP_WRITE: {
                    SonarChipRead(serial, timeout_mS);
                    break;
                }
                case TRANSMISSION_CHIP_READ: {
                    break;
                }
                case TRANSMISSION_GAIN: {
                    SonarPing(serial, sensor_address, timeout_mS);
                    break;
                }
                case TRANSMISSION_SET_ADDRESS: {
                    ROS_INFO("Sensor address set");
                    SonarChipWrite(serial, timeout_mS);
                    transmission_complete = true;
                    break;
                }
                case TRANSMISSION_READ: {
                    if (returned_bytes > 2) {
                        unsigned int n = (unsigned int)(comBuffer[2] << 8);
                        if (returned_bytes > 3) n |= comBuffer[3];
                        int echo_uS = (int)n;
                        if (echo_uS > 0) {
                            range_mm[sensor_index] = n * 10 / 58;

                            // publish the range
                            usbi2c::sonar_params s;
                            s.index = sensor_index;
                            s.range_mm = range_mm[sensor_index];
			    s.stamp = ros::Time::now();
                            sonar_pub.publish(s);

			    // publish filtered range
			    sensor_msgs::Range sonar_height;
			    sonar_height.header.stamp = ros::Time::now();
			    sonar_height.header.frame_id = "uav_base_link";
			    sonar_height.radiation_type = 2;
			    sonar_height.field_of_view = 0.0;
			    sonar_height.min_range = -1000.0;
			    sonar_height.max_range = 1000.0;
			    sonar_height.range = cAFilter->run(outlierFilter->run(s.range_mm / 1000.0));
			    sonar_filtered_pub.publish(sonar_height);

                           //ROS_INFO("Sensor %d  Range mm %d", sensor_index, range_mm[sensor_index]);
                        }

                        // send gain limit
                        SonarGainLimit(serial, sensor_address, timeout_mS);
                    }
                    break;
                }
            }
        }
    }
}

int main(int argc, char* argv[])
{

sleep(1);
    ros::init(argc, argv, "sonar");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    // get the number of sonar sensors
    nh.getParam("sensors", no_of_sensors);

  
    if (no_of_sensors <= 0) {
        ROS_ERROR("The number of sensors must be specified");
    }
    else {

        // get the device to read from
        std::string device_name = "";
        nh.getParam("device", device_name);
        if (device_name == "") device_name = "/dev/ttyUSB0";

        // do we need to set the address?
        std::string sensor_address_str = "";        
        nh.getParam("address", sensor_address_str);
        nh.setParam("address", ""); // clear the address parameter after it has been read
        if (sensor_address_str != "") {
            char* endstr;
            sensor_address = strtol(sensor_address_str.c_str(), &endstr, 16);
            if ((sensor_address > 0xfe) || (sensor_address < 0xe0)) {
                ROS_ERROR("The sensor address must be between 0xe0 and 0xfe");
                sensor_address_str = "";
            }
            else {
                if (no_of_sensors != 1) {
                    ROS_ERROR("When setting the address %s only one sensor should be attached to the module", sensor_address_str.c_str());
                    sensor_address_str = "";
                }
                else {
                    ROS_INFO("Setting sensor address to %s", sensor_address_str.c_str());
                }
            }
        }

        // sonar ranges are stored in this array
        int range_mm[no_of_sensors];

        // create the publisher
        sonar_pub = n.advertise<usbi2c::sonar_params>("sonar", 10);
	sonar_filtered_pub = n.advertise<sensor_msgs::Range> ("/sonar_height", 2, true);

	// create filters
	cAFilter = new ContingencyAnalysisWindowMeanFilter(3, 1.0);
  	outlierFilter = new OutlierRejectionFilter(-0.25, 0.5);

        SerialPort serial;
        if (serial.Open(device_name) == 0) {
            // 19200 baud, 8 data bits, no parity, 2 stop bits
            serial.Initialise(19200, 8, 2, PARITY_NONE);

            ROS_INFO("Connected to %s", device_name.c_str());

            if (sensor_address_str != "") {
                // set the address of the sonar sensor
                SetSensorAddress(serial, (unsigned char)sensor_address, 2000);
            }

            //ros::Rate loop_rate(20);
            while ((n.ok()) && (!transmission_complete)) {
                Update(serial,10,range_mm);
                ros::spinOnce();
                //loop_rate.sleep();
            }

            ROS_INFO("Closing %s", device_name.c_str());
        }
        else {
            ROS_WARN("Can't open %s", device_name.c_str());
        }
    }

    return 0;
}

