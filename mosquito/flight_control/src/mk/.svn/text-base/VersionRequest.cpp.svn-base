/*
 * VersionRequest.cpp
 *
 *  Created on: Mar 8, 2013
 *      Author: tlinder
 */

#include "VersionRequest.h"
#include "MKDefines.h"

VersionRequest::VersionRequest()
{
  resetStruct();
}

VersionRequest::~VersionRequest()
{
  // TODO Auto-generated destructor stub
}

std::string VersionRequest::sent()
{
 std::string commandString = MKHandler::make_Frame('v',FC_ADDRESS , c_Data, sizeof(0));
 return commandString;

}

void VersionRequest::received(std::string msg)
{
  //HandlerMK::


  //memcpy((char *) &c_Data, (char *) &versionInfo,sizeof(versionInfo));
  //return c_Data;
}

void VersionRequest::resetStruct()
{
  versionInfo.SWPatch=0;
  versionInfo.SWMinor=0;
  versionInfo.SWMajor=0;
  versionInfo.ProtoMinor=0;
  versionInfo.ProtoMajor=0;
  versionInfo.HardwareError[0]=0;
  versionInfo.HardwareError[1]=0;
  versionInfo.HardwareError[2]=0;
  versionInfo.HardwareError[3]=0;
  versionInfo.HardwareError[4]=0;

}


