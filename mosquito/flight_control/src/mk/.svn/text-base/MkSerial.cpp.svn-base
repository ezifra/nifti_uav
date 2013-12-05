/*
 * MkSerial.cpp
 *
 *  Created on: Sep 26, 2011
 *      Author: tlinder
 */

#include "MkSerial.h"

MkSerial::MkSerial() :
//	_serialport_name("/dev/ttyUSB0"), _serialport_speed(57600), _device(-1),
    _serialport_name("/dev/communication/flightcontrol"), _serialport_speed(57600), _device(-1), _isOpen(false)
{
  resetExternControlStruct();
}

MkSerial::~MkSerial()
{
  if (_isOpen)
  {
    closePort();
  }
  pthread_exit(NULL);
}

void MkSerial::setSerialDev(std::string port)
{
  if (("" != port) && (!_isOpen))
  {
    _serialport_name = port;
    ROS_INFO("Serial port path is set to %s", _serialport_name.c_str());
  }
  else
  {
    if (("" == port))
    {
      ROS_ERROR("Failure during setup of serial port path! The selected path %s could not been opened", port.c_str());
    }
    if (_isOpen)
    {
      ROS_ERROR(
          "Failure during setup of serial port path! The serial port is still open (%b). Please close first!", _isOpen);
    }
  }
}

std::string MkSerial::getSerialDev()
{
  return _serialport_name;
}

void MkSerial::setSerialSpeed(unsigned int speed)
{
  if ((B0 != bitrate(speed)) && (!_isOpen))
  {
    _serialport_speed = speed;
    ROS_INFO_STREAM("Serial port speed is set to "<< _serialport_speed);
  }
}

unsigned int MkSerial::getSerialSpeed()
{
  return _serialport_speed;
}

bool MkSerial::isOpen()
{
  return _isOpen;
}

bool MkSerial::closePort()
{
  if (_isOpen)
  {
    _isOpen = close(_device) != 0;
    return !_isOpen;
  }
  return false;
}

bool MkSerial::openPort()
{

  if (_isOpen)
  {
    ROS_INFO_STREAM("Serial port "<<_serialport_name<<" is already open!");
    return true;
  }

  ROS_INFO_STREAM("Initializing serial port "<< _serialport_name);

  _device = open(_serialport_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY, 0777);
  ROS_DEBUG("[MkSerial::openPort] dev: %d, serialport_baud %d", _device, bitrate(_serialport_speed));
  ROS_ASSERT_MSG(_device != -1, "Failed to open serial port: %s %s", _serialport_name.c_str(), strerror(errno));
  if (_device == -1)
  {
    return false;
  }

  int failure = tcgetattr(_device, &termios_configuration);
  ROS_ASSERT_MSG(failure == 0, "Unknown Error: %s", strerror(errno));
  if (0 != failure)
  {
    return false;
  }

  /*
   * Set the baud rates to 19200...
   */

  cfsetispeed(&termios_configuration, bitrate(_serialport_speed));
  cfsetospeed(&termios_configuration, bitrate(_serialport_speed));

  /*
   * Enable the receiver and set local mode...
   */

  termios_configuration.c_cflag |= (CLOCAL | CREAD);

  /*
   * No parity (8N1):
   */
  termios_configuration.c_cflag &= ~PARENB;
  termios_configuration.c_cflag &= ~CSTOPB;
  termios_configuration.c_cflag &= ~CSIZE;
  termios_configuration.c_cflag |= CS8;

  termios_configuration.c_cc[VMIN] = 0;
  termios_configuration.c_cc[VTIME] = 20;

  /*
   * No flow control
   */

  termios_configuration.c_cflag &= ~CRTSCTS;

  /*
   * Raw Input/Ouput
   */

  termios_configuration.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  termios_configuration.c_oflag &= ~OPOST;

  //  cfsetispeed(&termios_configuration, bitrate(_serialport_speed));
  //  cfsetospeed(&termios_configuration, bitrate(_serialport_speed));
  //#include "Class_HandlerMK/HandlerMK.h"
  //  //  tio.c_iflag = 0;
  //  //  tio.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL);
  //  //  tio.c_iflag |= IGNBRK;
  //  termios_configuration.c_iflag &= (~(INPCK | IGNPAR | PARMRK | ISTRIP | ICRNL | IXANY));
  //
  //  //  tio.c_oflag = 0;
  //  //  tio.c_oflag &= ~(OPOST | ONLCR);
  //  termios_configuration.c_oflag &= (~OPOST);
  //
  //  //tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;
  //  //tio.c_cflag &= ~(PARENB | CRTSCTS | CSTOPB);
  //  termios_configuration.c_cflag |= CREAD | CLOCAL;
  //
  //  //tio.c_lflag = 0;
  //  //tio.c_lflag |= NOFLSH;
  //  //tio.c_lflag &= ~(ISIG | IEXTEN | ICANON | ECHO | ECHOE);
  //  termios_configuration.c_lflag &= (~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG));
  //
  //  termios_configuration.c_cc[VMIN] = 0;
  //  termios_configuration.c_cc[VTIME] = 20;
  //  termios_configuration.c_cc[VINTR] = _POSIX_VDISABLE;
  //  termios_configuration.c_cc[VQUIT] = _POSIX_VDISABLE;
  //  termios_configuration.c_cc[VSTART] = _POSIX_VDISABLE;
  //  termios_configuration.c_cc[VSTOP] = _POSIX_VDISABLE;
  //  termios_configuration.c_cc[VSUSP] = _POSIX_VDISABLE;
  //
  failure = tcsetattr(_device, TCSADRAIN, &termios_configuration);
  ROS_ASSERT_MSG( failure == 0, "Unknown Error: %s", strerror(errno));
  if (0 != failure)
  {
    return false;
  }
  //  tio.c_cc[VMIN] = 0;
  //  tio.c_cc[VTIME] = 0;

  tcflush(_device, TCIOFLUSH);

  _isOpen = true;
  return true;
}

std::string MkSerial::readData()
{

  int input_buffer_size;
  ioctl(_device, FIONREAD, &input_buffer_size);
  if (0 < input_buffer_size)
  {
    ROS_DEBUG("[MkSerial::readData] Input Buffer Size: %d", input_buffer_size);
    int nLength = read(_device, _raw_buffer, sizeof(_raw_buffer));
    ROS_DEBUG("[MkSerial::readData] RAW Input: %s",_raw_buffer);

    static uint16_t crc;
    static uint8_t ptr_rxd_buffer = 0;
    uint8_t crc1, crc2;
    uint8_t letter;
    bool foundStart = false;
    //int ReceivedBytes=0;

    for (int i = 0; i < nLength; i++)
    {
      //      c = UDR0;  // catch the received byte
      letter = _raw_buffer[i]; // catch the received byte

      // the rxd buffer is unlocked
      if ((ptr_rxd_buffer == 0) && (letter == '#')) // if rxd buffer is empty and syncronisation character is received
      {
        _received_buffer[ptr_rxd_buffer++] = letter; // copy 1st byte to buffer
        crc = letter; // init crc
        foundStart = true;
      }
      else if ((foundStart) && (ptr_rxd_buffer < RXD_BUFFER_LEN)) // collect incomming bytes
      {
        if (letter == '#')
        {
          //Error Start symbols after the message did already start
          ptr_rxd_buffer = 0;
          _received_buffer[ptr_rxd_buffer++] = letter; // copy 1st byte to buffer
          crc = letter; // init crc
          foundStart = true;
        }
        else if (letter != '\r') // no termination character
        {
          _received_buffer[ptr_rxd_buffer++] = letter; // copy byte to rxd buffer
          crc += letter; // update crc
        }
        else // termination character was received
        {
          // the last 2 bytes are no subject for checksum calculation
          // they are the checksum itself
          crc -= _received_buffer[ptr_rxd_buffer - 2];
          crc -= _received_buffer[ptr_rxd_buffer - 1];
          // calculate checksum from transmitted data
          crc %= 4096;
          crc1 = '=' + crc / 64;
          crc2 = '=' + crc % 64;
          // compare checksum to transmitted checksum bytes
          if ((crc1 == _received_buffer[ptr_rxd_buffer - 2]) && (crc2 == _received_buffer[ptr_rxd_buffer - 1]))
          { // checksum valid
            ROS_DEBUG("[MkSerial::readData] CRC - Valid packet");
            _received_buffer[ptr_rxd_buffer] = '\r'; // set termination character
            //ReceivedBytes = ptr_rxd_buffer + 1;// store number of received bytes
            // if 2nd byte is an 'R' enable watchdog that will result in an reset
          }
          else
          { // checksum invalid
            //rxd_buffer_locked = FALSE; // unlock rxd buffer
            ROS_ERROR_STREAM("Checksum invalid, skip the frame. "<< _raw_buffer);
            ptr_rxd_buffer = 0; // reset rxd buffer pointer
            return "";
          }
          ptr_rxd_buffer = 0; // reset rxd buffer pointer
          foundStart = false;
        }
      }
      else // rxd buffer overrun
      {
        ptr_rxd_buffer = 0; // reset rxd buffer
        foundStart = false;
      }

    }

    std::stringstream res;
    res << _received_buffer;
    return res.str();
  }
  return "";
}

bool MkSerial::writeData(std::string data)
{

  int sendBytes;
  int length = data.size();
  ROS_DEBUG("[MkSerial::writeData] Writing %d element(s): %s", length, data.c_str());
  flushData();
  sendBytes = write(_device, data.c_str(), length);
  if (sendBytes != length)
  {
    ROS_ERROR("Error wrote %d out of %d element(s): %s", sendBytes, length, strerror (errno));
    //ROS_BREAK ();

    return false;
  }

  ROS_DEBUG("[MkSerial::writeData] Write completed");
  return flushData();
}

bool MkSerial::flushData()
{
  return 0 == ioctl(_device, TCFLSH);
}

speed_t MkSerial::bitrate(int Bitrate)
{
  switch (Bitrate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default: // invalid bitrate
      return B0;
  }
}

char* MkSerial::addCRC(char* g_txd_buffer, unsigned int datalen)
{
  uint16_t tmpCRC = 0, i;
  for (i = 0; i < datalen; i++)
  {
    tmpCRC += g_txd_buffer[i];
  }
  tmpCRC %= 4096;
  g_txd_buffer[datalen++] = '=' + tmpCRC / 64;
  g_txd_buffer[datalen++] = '=' + tmpCRC % 64;
  g_txd_buffer[datalen++] = '\r';

  return g_txd_buffer;
}

void MkSerial::parse_MK_Data(std::string t_Data)
{
  unsigned char OutData[150];
  char *InData = (char *)t_Data.c_str();
  if (MKHandler::Decode_64(InData, t_Data.length(), OutData) != 0)
  {
    switch (t_Data[2])
    {
      case 'D': // Debug-Data
      {
        for (int z = 0; z < MAX_DebugData; z++)
        {
          Debug_Data[z] = MKHandler::Data2Int(OutData, (z * 2) + 2);
        }

        rawSensorData.AngleNick = Debug_Data[0] * (90.0 / 650.0);
        rawSensorData.AngleRoll = Debug_Data[1] * (90.0 / 650.0);
        rawSensorData.AccX = Debug_Data[2];
        rawSensorData.AccY = Debug_Data[3];
        rawSensorData.YawGyroRate = Debug_Data[4];
        rawSensorData.Height_Value = Debug_Data[5] / 100.0;
        rawSensorData.AccZ = Debug_Data[6];
        rawSensorData.Gas = Debug_Data[7];
        rawSensorData.Compass_Value = Debug_Data[8];
        rawSensorData.Voltage = Debug_Data[9] / 10.0;
        rawSensorData.Receiver_Level = Debug_Data[10];
        rawSensorData.Gyro_Compass = -1 * Debug_Data[11] / 10.0 - 180.0;
        rawSensorData.NickGyroRate = Debug_Data[12];
        rawSensorData.RollGryoRate = Debug_Data[13];
        rawSensorData.MagneticX = Debug_Data[14];
        rawSensorData.MagneticY = Debug_Data[15];
        rawSensorData.MagneticZ = Debug_Data[16];
        rawSensorData.Current_Motor2 = Debug_Data[17] / 10.0;
        rawSensorData.Field_18 = Debug_Data[18];
        rawSensorData.Field_19 = Debug_Data[19];
        rawSensorData.Servo = Debug_Data[20];
        rawSensorData.Hovergas = Debug_Data[21];
        rawSensorData.Current = Debug_Data[22] / 10.0;
        rawSensorData.Capacity = Debug_Data[23] / 1000.0;
        rawSensorData.Hight_Setpoint = Debug_Data[24];
        rawSensorData.Field_25 = Debug_Data[25];
        rawSensorData.Field_26 = Debug_Data[26];
        rawSensorData.Compass_Setpoint = Debug_Data[27];
        rawSensorData.I2C_Error = Debug_Data[28];
        rawSensorData.BL_Limit = Debug_Data[29];
        rawSensorData.GPS_Nick = Debug_Data[30];
        rawSensorData.GPS_Roll = Debug_Data[31];

        //ROS_INFO_STREAM( "Read Voltage: " << rawSensorData.Voltage << " V");
        break;
      }
      case 'V':
      {

        break;
      }
        break;
      default:
      {
        //ROS_INFO_STREAM(t_Data);
        break;
      }
    }
  }
}

void MkSerial::resetExternControlStruct()
{
  ExternControl.free = 2;
  ExternControl.Digital[0] = 0x55;
  ExternControl.Gas = 0;
  ExternControl.Height = 0;
  ExternControl.RemoteButtons = 0;
  ExternControl.Config = 1;
  ExternControl.Nick = 0;
  ExternControl.Frame = 1;
  ExternControl.Yaw = 0;
  ExternControl.Roll = 0;
}

// Base64 Encoder
std::string MkSerial::encode64(std::string Data, unsigned int Length)
{
  unsigned int pt = 0;
  unsigned char a, b, c;
  unsigned char ptr = 0;

  char TX_Buff[Length];

  while (Length > 0)
  {
    if (Length)
    {
      a = Data[ptr++];
      Length--;
    }
    else
      a = 0;
    if (Length)
    {
      b = Data[ptr++];
      Length--;
    }
    else
      b = 0;
    if (Length)
    {
      c = Data[ptr++];
      Length--;
    }
    else
      c = 0;

    TX_Buff[pt++] = '=' + (a >> 2);
    TX_Buff[pt++] = '=' + (((a & 0x03) << 4) | ((b & 0xf0) >> 4));
    TX_Buff[pt++] = '=' + (((b & 0x0f) << 2) | ((c & 0xc0) >> 6));
    TX_Buff[pt++] = '=' + (c & 0x3f);
  }
  TX_Buff[pt] = 0;

  return string(TX_Buff);
}
