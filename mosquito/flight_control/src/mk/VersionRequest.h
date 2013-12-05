/*
 * VersionRequest.h
 *
 *  Created on: Mar 8, 2013
 *      Author: tlinder
 */

#ifndef VERSIONREQUEST_H_
#define VERSIONREQUEST_H_

/*
  '#''v''any-ID'"struct"'CRC1''CRC2''\r'
 */
#include "MKHandler.h"

class VersionRequest
{
public:
  VersionRequest();
  virtual ~VersionRequest();
  void received(std::string msg);
  std::string sent();

  str_VersionInfo versionInfo;

  private:

  void resetStruct();
  char c_Data[10];
};

#endif /* VERSIONREQUEST_H_ */
