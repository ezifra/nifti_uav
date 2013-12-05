/***************************************************************************
 *   Copyright (C) 2009 by Manuel Schrape                                  *
 *   manuel.schrape@gmx.de                                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License.        *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef KOPTER_H
#define KOPTER_H

#include "stdio.h"
#include <stdint.h>
#include <string>
using namespace std;

#ifdef _BETA_
    static const QString QA_HWVERSION = "FlightCtrl v0.78b & NaviCtrl v0.18c";
#else
    static const string QA_HWVERSION = "FlightCtrl v0.78b & NaviCtrl v0.18c";
#endif

// Datenfeld-ID's
static const int DATA_VERSION        = 1;
static const int DATA_READ_SETTINGS  = 2;
static const int DATA_WRITE_SETTINGS = 3;
static const int DATA_READ_MIXER     = 4;
static const int DATA_WRITE_MIXER    = 5;
static const int DATA_READ_LABEL     = 6;
static const int DATA_WRITE_WAYPOINT = 7;

//static const

static const string HardwareType[] = {"Default", "FlightCtrl", "NaviCtrl", "MK3Mag"};

static const int MAX_DebugData = 32;

static const string DEF_DebugNames[] = {"Integral Nick", "Integral Roll", "ACC Nick", "ACC Roll", "Gyro Gier", "Hoehen-Wert", "ACC Z", "GAS", "Kompass-Value", "Spannung", "Empfang", "Ersatzkompass", "Motor Vorne", "Motor Hinten", "Motor Links", "Motor Rechts", "Analog 16", "Distance", "OSD-Bar", "MK3Mag", "Servo", "Nick", "Roll", "Analog 23", "Analog 24",  "Analog 25",  "Analog 26",  "Kalman Max",  "Analog 28",  "Kalman K", "GPS Nick", "GPS Roll"};

//static const QRgb DEF_DebugColors[] = {0x00FF0000, 0x0000FF00, 0x00FFFF00, 0x000000FF, 0x00FF00FF, 0x0000FFFF, 0x00FFFFFF, 0x00660000, 0x00006600, 0x00666600, 0x00000066, 0x00660066, 0x000066, 0x00666666, 0x00990000, 0x00009900, 0x00999900, 0x00000099, 0x00990099, 0x00009999, 0x00999999, 0x00CC0000, 0x0000CC00, 0x00CCCC00, 0x000000CC, 0x00CC00CC, 0x0000CCCC, 0x00CCCCCC, 0x0066FF99, 0x009966FF, 0x00FF9966, 0x0099FF66};

#endif // KOPTER_H
