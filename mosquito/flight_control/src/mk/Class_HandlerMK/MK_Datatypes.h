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
#ifndef MK_DATATYPES_H
#define MK_DATATYPES_H

#include <stdint.h>

#ifdef _BETA_
    static const int MK_VERSION_SETTINGS = 80; // wird angepasst, wenn sich die EEPROM-Daten ge�ndert haben
#else
    static const int MK_VERSION_SETTINGS = 80; // wird angepasst, wenn sich die EEPROM-Daten ge�ndert haben
#endif

static const int MK_VERSION_NAVI     =  2; // wird angepasst, wenn sich die Mixer-Daten ge�ndert haben

static const int MK_VERSION_MIXER    =  1; // wird angepasst, wenn sich die Mixer-Daten ge�ndert haben
static const int MK_MAX_MOTOR        =  16; // Maximale Anzahl der Motoren im Mixer

// Version des Seriellen Protokoll
static const int VERSION_SERIAL_MAJOR = 10;
static const int VERSION_SERIAL_MINOR = 0;

// Basis-Addressen der verschiedenen Hardware
static const int ADDRESS_ALL    = 0;
static const int ADDRESS_FC     = 1;
static const int ADDRESS_NC     = 2;
static const int ADDRESS_MK3MAG = 3;

#define FLAG_MOTOR_RUN  1
#define FLAG_FLY        2
#define FLAG_CALIBRATE  4
#define FLAG_START      8
#define FLAG_NOTLANDUNG 16
#define FLAG_LOWBAT	32

#define CFG_HOEHENREGELUNG       0x01
#define CFG_HOEHEN_SCHALTER      0x02
#define CFG_HEADING_HOLD         0x04
#define CFG_KOMPASS_AKTIV        0x08
#define CFG_KOMPASS_FIX          0x10
#define CFG_GPS_AKTIV            0x20
#define CFG_ACHSENKOPPLUNG_AKTIV 0x40
#define CFG_DREHRATEN_BEGRENZER  0x80

#define CFG_LOOP_OBEN            0x01
#define CFG_LOOP_UNTEN           0x02
#define CFG_LOOP_LINKS           0x04
#define CFG_LOOP_RECHTS          0x08
#define CFG_MOTOR_BLINK	         0x10
#define CFG_MOTOR_OFF_LED1       0x20
#define CFG_MOTOR_OFF_LED2       0x40
#define CFG_RES4	         0x80

#define CFG2_HEIGHT_LIMIT        0x01
#define CFG2_VARIO_BEEP          0x02
#define CFG_SENSITIVE_RC         0x04

struct s_MK_Settings
{
    // Die ersten beiden Bytes nicht an den MK senden.
   unsigned char Index;
   unsigned char Version;

   unsigned char Kanalbelegung[8];       // GAS[0], GIER[1],NICK[2], ROLL[3], POTI1, POTI2, POTI3
   unsigned char GlobalConfig;           // 0x01=H�henregler aktiv,0x02=Kompass aktiv, 0x04=GPS aktiv, 0x08=Heading Hold aktiv
   unsigned char Hoehe_MinGas;           // Wert : 0-100
   unsigned char Luftdruck_D;            // Wert : 0-250
   unsigned char MaxHoehe;               // Wert : 0-32
   unsigned char Hoehe_P;                // Wert : 0-32
   unsigned char Hoehe_Verstaerkung;     // Wert : 0-50
   unsigned char Hoehe_ACC_Wirkung;      // Wert : 0-250
   unsigned char Hoehe_HoverBand;        // Wert : 0-250
   unsigned char Hoehe_GPS_Z;            // Wert : 0-250
   unsigned char Hoehe_StickNeutralPoint;// Wert : 0-250
   unsigned char Stick_P;                // Wert : 1-6
   unsigned char Stick_D;                // Wert : 0-64
   unsigned char Gier_P;                 // Wert : 1-20
   unsigned char Gas_Min;                // Wert : 0-32
   unsigned char Gas_Max;                // Wert : 33-250
   unsigned char GyroAccFaktor;          // Wert : 1-64
   unsigned char KompassWirkung;         // Wert : 0-32
   unsigned char Gyro_P;                 // Wert : 10-250
   unsigned char Gyro_I;                 // Wert : 0-250
   unsigned char Gyro_D;                 // Wert : 0-250
   unsigned char Gyro_Gier_P;            // Wert : 10-250
   unsigned char Gyro_Gier_I;            // Wert : 0-250
   unsigned char UnterspannungsWarnung;  // Wert : 0-250
   unsigned char NotGas;                 // Wert : 0-250     //Gaswert bei Emp�ngsverlust
   unsigned char NotGasZeit;             // Wert : 0-250     // Zeitbis auf NotGas geschaltet wird, wg. Rx-Problemen
   unsigned char UfoAusrichtung;         // X oder + Formation
   unsigned char I_Faktor;               // Wert : 0-250
   unsigned char UserParam1;             // Wert : 0-250
   unsigned char UserParam2;             // Wert : 0-250
   unsigned char UserParam3;             // Wert : 0-250
   unsigned char UserParam4;             // Wert : 0-250
   unsigned char ServoNickControl;       // Wert : 0-250     // Stellung des Servos
   unsigned char ServoNickComp;          // Wert : 0-250     // Einfluss Gyro/Servo
   unsigned char ServoNickMin;           // Wert : 0-250     // Anschlag
   unsigned char ServoNickMax;           // Wert : 0-250     // Anschlag
//--- Seit V0.75
   unsigned char ServoRollControl;       // Wert : 0-250     // Stellung des Servos
   unsigned char ServoRollComp;          // Wert : 0-250
   unsigned char ServoRollMin;           // Wert : 0-250
   unsigned char ServoRollMax;           // Wert : 0-250
//---
   unsigned char ServoNickRefresh;       //
   unsigned char LoopGasLimit;           // Wert: 0-250  max. Gas w�hrend Looping
   unsigned char LoopThreshold;          // Wert: 0-250  Schwelle f�r Stickausschlag
   unsigned char LoopHysterese;          // Wert: 0-250  Hysterese f�r Stickausschlag
   unsigned char AchsKopplung1;          // Wert: 0-250  Faktor, mit dem Gier die Achsen Roll und Nick koppelt (NickRollMitkopplung)
   unsigned char AchsKopplung2;          // Wert: 0-250  Faktor, mit dem Nick und Roll verkoppelt werden
   unsigned char CouplingYawCorrection;  // Wert: 0-250  Faktor, mit dem Nick und Roll verkoppelt werden
   unsigned char WinkelUmschlagNick;     // Wert: 0-250  180�-Punkt
   unsigned char WinkelUmschlagRoll;     // Wert: 0-250  180�-Punkt
   unsigned char GyroAccAbgleich;        // 1/k  (Koppel_ACC_Wirkung)
   unsigned char Driftkomp;
   unsigned char DynamicStability;
   unsigned char UserParam5;             // Wert : 0-250
   unsigned char UserParam6;             // Wert : 0-250
   unsigned char UserParam7;             // Wert : 0-250
   unsigned char UserParam8;             // Wert : 0-250
//---Output ---------------------------------------------
   unsigned char J16Bitmask;             // for the J16 Output
   unsigned char J16Timing;              // for the J16 Output
   unsigned char J17Bitmask;             // for the J17 Output
   unsigned char J17Timing;              // for the J17 Output
// seit version V0.75c
   unsigned char WARN_J16_Bitmask;       // for the J16 Output
   unsigned char WARN_J17_Bitmask;       // for the J17 Output
//---NaviCtrl---------------------------------------------
   unsigned char NaviGpsModeControl;     // Parameters for the Naviboard
   unsigned char NaviGpsGain;
   unsigned char NaviGpsP;
   unsigned char NaviGpsI;
   unsigned char NaviGpsD;
   unsigned char NaviGpsPLimit;
   unsigned char NaviGpsILimit;
   unsigned char NaviGpsDLimit;
   unsigned char NaviGpsACC;
   unsigned char NaviGpsMinSat;
   unsigned char NaviStickThreshold;
   unsigned char NaviWindCorrection;
   unsigned char NaviSpeedCompensation;
   unsigned char NaviOperatingRadius;
   unsigned char NaviAngleLimitation;
   unsigned char NaviPH_LoginTime;
//---Ext.Ctrl---------------------------------------------
   unsigned char ExternalControl;        // for serial Control
//------------------------------------------------
   unsigned char BitConfig;          // (war Loop-Cfg) Bitcodiert: 0x01=oben, 0x02=unten, 0x04=links, 0x08=rechts / wird getrennt behandelt
   unsigned char ServoCompInvert;    // //  0x01 = Nick, 0x02 = Roll   0 oder 1  // WICHTIG!!! am Ende lassen
   unsigned char ExtraConfig;        // bitcodiert
   char Name[12];
 };

struct s_MK_Mixer
{
    char Revision;
    char Name[12];
    signed char Motor[16][4];
};

///////////////
// Navi-Ctrl //
///////////////

#define INVALID	0x00
#define NEWDATA	0x01
#define PROCESSED	0x02

#define NC_FLAG_FREE		0x01
#define NC_FLAG_PH		0x02
#define NC_FLAG_CH		0x04
#define NC_FLAG_RANGE_LIMIT	0x08
#define NC_FLAG_NOSERIALLINK	0x10
#define NC_FLAG_TARGET_REACHED	0x20
#define NC_FLAG_MANUAL_CONTROL	0x40
#define NC_FLAG_8		0x80

typedef struct
{
    int32_t Longitude;      // in 1E-7 deg
    int32_t Latitude;       // in 1E-7 deg
    int32_t Altitude;       // in mm
    uint8_t Status;         // validity of data
} __attribute__((packed)) GPS_Pos_t;

typedef struct
{
    uint16_t Distance;      // distance to target in dm
    int16_t Bearing;        // course to target in deg
}  __attribute__((packed)) GPS_PosDev_t;

typedef struct
{
    uint8_t      Version;		// version of the data structure
    GPS_Pos_t    CurrentPosition;		// see ubx.h for details
    GPS_Pos_t    TargetPosition;
    GPS_PosDev_t TargetPositionDeviation;
    GPS_Pos_t    HomePosition;
    GPS_PosDev_t HomePositionDeviation;
    uint8_t      WaypointIndex;		// index of current waypoints running from 0 to WaypointNumber-1
    uint8_t      WaypointNumber;		// number of stored waypoints
    uint8_t      SatsInUse;		// number of satellites used for position solution
    int16_t      Altimeter; 		// hight according to air pressure
    int16_t      Variometer;		// climb(+) and sink(-) rate
    uint16_t     FlyingTime;		// in seconds
    uint8_t      UBat;			// Battery Voltage in 0.1 Volts
    uint16_t     GroundSpeed;		// speed over ground in cm/s (2D)
    int16_t      Heading;		// current flight direction in ° as angle to north
    int16_t      CompassHeading;		// current compass value in °
    int8_t       AngleNick;		// current Nick angle in 1°
    int8_t       AngleRoll;		// current Rick angle in 1°
    uint8_t      RC_Quality;		// RC_Quality
    uint8_t      MKFlags;		// Flags from FC
    uint8_t      NCFlags;		// Flags from NC
    uint8_t      Errorcode;		// 0 --> okay
    uint8_t      OperatingRadius;               // current operation radius around the Home Position in m
    int16_t      TopSpeed;		// velocity in vertical direction in cm/s
    uint8_t      TargetHoldTime;		// time in s to stay at the given target, counts down to 0 if target has been reached
    uint8_t      RC_RSSI;		// Receiver signal strength (since version 2 added)
    int16_t      SetpointAltitude;			// setpoint for altitude
    uint8_t      Gas;						// for future use
} __attribute__((packed)) s_MK_NaviData;

typedef struct
{
    GPS_Pos_t Position;          // the gps position of the waypoint, see ubx.h for details
    int16_t   Heading;           // orientation, future implementation
    uint8_t   ToleranceRadius;   // in meters, if the MK is within that range around the target, then the next target is triggered
    uint8_t   HoldTime;          // in seconds, if the was once in the tolerance area around a WP, this time defies the delay before the next WP is triggered
    uint8_t   Event_Flag;        // future emplementation
    uint8_t   reserve[12];       // reserve
} __attribute__((packed)) s_MK_WayPoint;

#endif // MK_DATATYPES_H
