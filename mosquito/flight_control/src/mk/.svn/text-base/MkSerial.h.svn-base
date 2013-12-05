/*
 * MkSerial.h
 *
 *  Created on: Sep 26, 2011
 *      Author: tlinder
 */

#ifndef MKSERIAL_H_
#define MKSERIAL_H_

#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include "fcntl.h"

#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <time.h>
#include <errno.h>
#include <bitset>
#include <math.h>

#include "fcntl.h"
#include "MKHandler.h"
#include "MKDefines.h"

#include <ros/ros.h>

class MkSerial
{
public:
  MkSerial();
  virtual ~MkSerial();

  //device section
  void setSerialDev(std::string port);
  std::string getSerialDev();

  //speed section
  void setSerialSpeed(unsigned int speed);
  unsigned int getSerialSpeed();

  //open and close section
  bool openPort();
  bool closePort();
  bool isOpen();

  //read, write and flush section
  std::string readData();
  void parse_MK_Data(std::string t_Data);
  bool writeData(std::string data);
  bool flushData();
  char c_Data[150];
  int Debug_Data[MAX_DebugData];

  std::string encode64(std::string Data, unsigned int Length);
  void resetExternControlStruct();

  str_rawSensorData rawSensorData;
  str_ExternControl ExternControl;

private:
  std::string _serialport_name;
  uint32_t _serialport_speed;
  int _device;
  bool _isOpen;

#define RXD_BUFFER_LEN  300
  unsigned char _raw_buffer[RXD_BUFFER_LEN];
  unsigned char _received_buffer[RXD_BUFFER_LEN];
  struct termios termios_configuration;

  speed_t bitrate(int Bitrate);
  char* addCRC(char* g_txd_buffer, unsigned int datalen);

};

#endif /* MKSERIAL_H_ */
