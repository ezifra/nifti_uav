/*********************************************************************
* Software License Agreement (BSD License)
*
*  Serial port class, based upon code written by J.D.Medhurst (a.k.a. Tixy)
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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
 
#define PARITY_NONE  0
#define PARITY_EVEN  1
#define PARITY_ODD   2

class SerialPort
{
public:
    enum Errors {
        ErrorUnspecified    = -100, 
        ErrorInvalidPort    = -101, 
        ErrorPortInUse      = -102, 
        ErrorInvalidSettings= -103, 
        ErrorTransmitError  = -104, 
        ErrorReceiveError   = -105  
    };

    int SerialHandle; 

    SerialPort() { SerialHandle = 0; }
    ~SerialPort();

    int Open(std::string device_name);
    int Initialise(unsigned baud, unsigned inDataBits, unsigned inStopBits, unsigned inParity);
    int Out(const uint8_t* data, size_t size, unsigned timeout);
    int In(uint8_t* data, size_t maxSize, unsigned timeout);
    void Close();
    int Error(int defaultError=ErrorUnspecified);

};

/*!
 * \brief report an error
 * \param defaultError error number
 */
int SerialPort::Error(
    int defaultError)
{
    ROS_INFO("ERROR: %d (errno=%d '%s')",defaultError,errno,strerror(errno));
    return defaultError;
}

/*!
 * \brief open serial comms
 * \param device_name name of the serial device, eg. /dev/ttyUSB0
 * \return 0 on success, or an error value
 */
int SerialPort::Open(
    std::string device_name)
{
    SerialHandle = open(device_name.c_str(), O_RDWR | O_NOCTTY);
    if (SerialHandle < 0) {
        switch(errno)
        {
            default:
               return Error(ErrorInvalidPort);
        }
    }
 
    return 0;
}

/*!
 * \brief initialise the serial comms parameters
 * \param baud baud rate
 * \param inDataBits number of data bits
 * \param inStopBits number of stop bits
 * \param inParity parity
 * \return zero on success, or an error value
 */
int SerialPort::Initialise(
    unsigned baud, 
    unsigned inDataBits, 
    unsigned inStopBits, 
    unsigned inParity)
{
    switch(baud)
    {
        case 1200: baud = B1200; break;
        case 2400: baud = B2400; break;
        case 4800: baud = B4800; break;
        case 9600: baud = B9600; break;
        case 19200: baud = B19200; break;
        case 38400: baud = B38400; break;
        case 57600: baud = B57600; break;
        case 115200: baud = B115200; break;
        case 230400: baud = B230400; break;
        default: return ErrorInvalidSettings;
    }
 
    unsigned int parity = 0;
    switch (inParity) 
    {
        case PARITY_NONE: {
            parity = 0;
            break;
        }
        case PARITY_EVEN: {
            parity = PARENB;
            break;
        }
        case PARITY_ODD: {
            parity = PARENB | PARODD;
            break;
        }
    }

    unsigned int dataBits = 0;
    switch( inDataBits ) {
        case 5: {
            dataBits = CS5;
            break;
        }
        case 6: {
            dataBits = CS6;
            break;
        }
        case 7: {
            dataBits = CS7;
            break;
        }
        case 8: {
            dataBits = CS8;
            break;
        }
    }

    unsigned int stopBits = 0;
    switch( inStopBits ) {
        case 1: {
            stopBits = 0;
            break;
        }
        case 2: {
            stopBits = CSTOPB;
            break;
        }
    }

    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    tio.c_cflag = baud | parity | dataBits | stopBits | CLOCAL | CREAD;
    tio.c_iflag     = IGNPAR;
    tio.c_oflag     = 0;
    tio.c_lflag     = 0;    // set input mode (non-canonical, no echo,...)
    tio.c_cc[VTIME] = 0;    // inter-character timer unused
    tio.c_cc[VMIN]  = 0;    // don't block if no chars available
 
    tcflush(SerialHandle, TCIFLUSH);
    if (tcsetattr(SerialHandle, TCSANOW, &tio) < 0)
        return ErrorInvalidSettings;

    return 0;
}

/*!
 * \brief sends data to the serial device
 * \param data data to be sent
 * \param size number of bytes
 * \param timeout timeout in milliseconds
 * \return number of bytes written
 */
int SerialPort::Out(
    const uint8_t* data, 
    size_t size, 
    unsigned timeout)
{
    int bytes = 0;
    for(;;) {
        // read...
        bytes = write(SerialHandle,data,size);
        if(bytes > 0) break; // end if any data was received
        if (bytes < 0) return Error(ErrorTransmitError);
        if(timeout == 0) break; // end if we have timed out
 
        // wait for 10 milliseconds...
        unsigned sleep = 10;
        if (sleep > timeout) sleep = timeout;
        timeout -= sleep;
        usleep(sleep*1000);
    }
 
    return bytes;
} 
 
/*!
 * \brief reads data from the serial device
 * \param data array into which to read the data
 * \param maxSize maximumnumber of bytes to read
 * \param timeout timeout in milliseconds
 * \return number of bytes read
 */
int SerialPort::In(
    uint8_t* data, 
    size_t maxSize, 
    unsigned timeout)
{
    int bytes = 0;
    for(;;) {
        // read...
        bytes = read(SerialHandle,data,maxSize);
        if (bytes > 0) break; // end if any data was received
        if (bytes < 0) return Error(ErrorReceiveError);
        if (timeout == 0) break; // end if we have timed out
 
        // wait for 10 milliseconds...
        unsigned sleep = 10;
        if (sleep > timeout) sleep = timeout;
        timeout -= sleep;
        usleep(sleep*1000);
    }
 
    return bytes;
}

/*!
 * \brief close the serial port
 */
void SerialPort::Close()
{
    if (SerialHandle) {
        close(SerialHandle);
        SerialHandle = 0;
    }
}
 
/*!
 * \brief destructor
 */
SerialPort::~SerialPort()
{
    Close();
}

