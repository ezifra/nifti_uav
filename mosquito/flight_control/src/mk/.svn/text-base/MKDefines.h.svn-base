/*
 * MKDefines.h
 *
 *  Created on: Mar 11, 2013
 *      Author: tlinder
 */

#ifndef MKDEFINES_H_
#define MKDEFINES_H_


static const int MAX_DebugData = 32;

//The protocol is based on individual serial data frames that are organized as shown in the following table.
//Start-Byte - Address - Byte ID-Byte - n Data-Bytes coded - CRC-Byte1 - CRC-Byte2 - Stop-Byte
//'#' - 'a'+ Addr - 'V','D' etc - "modified-base64" - variable - variable - '\r'


//Address used for communication:
#define ALL_ADDRESS     0 //(a)
#define FC_ADDRESS      1  //(b)
#define NC_ADDRESS      2  //(c)
#define MK3MAG_ADDRESS  3  //(d)
#define BL_CTRL_ADDRESS 5  //(f)

//
#define FLAG_MOTOR_RUN  1
#define FLAG_FLY        2
#define FLAG_CALIBRATE  4
#define FLAG_START      8
#define FLAG_NOTLANDUNG 16
#define FLAG_LOWBAT     32

//Configuration IDs
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
#define CFG_MOTOR_BLINK          0x10
#define CFG_MOTOR_OFF_LED1       0x20
#define CFG_MOTOR_OFF_LED2       0x40
#define CFG_RES4                 0x80

#define CFG2_HEIGHT_LIMIT        0x01
#define CFG2_VARIO_BEEP          0x02
#define CFG_SENSITIVE_RC         0x04


//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

typedef struct
{
    int Longitude;      // in 1E-7 deg
    int Latitude;       // in 1E-7 deg
    int Altitude;       // in mm
    unsigned char Status;         // validity of data
} __attribute__((packed)) GPS_Pos_t;


typedef struct
{
    unsigned short Distance;      // distance to target in dm
    short Bearing;        // course to target in deg
}  __attribute__((packed)) GPS_PosDev_t;

typedef struct
{
    GPS_Pos_t Position;          // the gps position of the waypoint, see ubx.h for details
    short   Heading;           // orientation, future implementation
    unsigned char   ToleranceRadius;   // in meters, if the MK is within that range around the target, then the next target is triggered
    unsigned char   HoldTime;          // in seconds, if the was once in the tolerance area around a WP, this time defies the delay before the next WP is triggered
    unsigned char   Event_Flag;        // future emplementation
    unsigned char   reserve[12];       // reserve
} __attribute__((packed)) s_MK_WayPoint;

#define INVALID       0x00
#define NEWDATA 0x01
#define PROCESSED       0x02


//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

// Configuration/Setting Struct
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

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

// VersionInfo
struct str_VersionInfo
{
  unsigned char SWMajor;
  unsigned char SWMinor;
  unsigned char ProtoMajor;
  unsigned char ProtoMinor;
  unsigned char SWPatch;
  unsigned char HardwareError[5];
};

// bitmask for HardwareError[0]
#define FC_ERROR0_GYRO_NICK             0x01
#define FC_ERROR0_GYRO_ROLL             0x02
#define FC_ERROR0_GYRO_YAW              0x04
#define FC_ERROR0_ACC_NICK              0x08
#define FC_ERROR0_ACC_ROLL              0x10
#define FC_ERROR0_ACC_TOP               0x20
#define FC_ERROR0_PRESSURE              0x40
#define FC_ERROR0_CAREFREE              0x80

// bitmask for HardwareError[1]
#define FC_ERROR1_I2C                   0x01
#define FC_ERROR1_BL_MISSING            0x02
#define FC_ERROR1_SPI_RX                0x04
#define FC_ERROR1_PPM                   0x08
#define FC_ERROR1_MIXER                 0x10
#define FC_ERROR1_RES1                  0x20
#define FC_ERROR1_RES2                  0x40
#define FC_ERROR1_RES3                  0x80

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

// Brushless motor controller
typedef struct
{
        unsigned char Index;            // address of BL-Ctrl
        unsigned char Current;
        unsigned char Temperature;      // only valid fpr BL-Ctrl >= V2.0
        unsigned char MaxPWM;
        unsigned char Status;
}  BLData_t;

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

// Debug Request Reply
// see Debug Request
struct str_DebugOut
{
 unsigned char Status[2];
 signed int Analog[32];    // Debugvalue can be displayed in MK-Tool as value or graph
};

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

//Compass
struct str_Data3D
{
   signed int  Winkel[3]; // nick, roll, compass in 0,1°
   signed char Centroid[3];
   signed char reserve[5];
};
extern struct str_Data3D Data3D;

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

//External Control
struct str_ExternControl
{
 unsigned char Digital[2];
 unsigned char RemoteButtons;
 signed char   Nick;
 signed char   Roll;
 signed char   Yaw;
 unsigned char Gas;
 signed char   Height;
 unsigned char free;
 unsigned char Frame;
 unsigned char Config;
};
//extern struct str_ExternControl   ExternControl;

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

// RAW Digital Sensor Data
struct str_rawSensorData
  {
    double AngleNick;
    double AngleRoll;
    double AccX;//AccNick
    double AccY;//AccRoll
    double YawGyroRate;//YawGyro
    double Height_Value;
    double AccZ;//AccZ
    double Gas;
    double Compass_Value;
    double Voltage;
    double Receiver_Level;
    double Gyro_Compass;
    double NickGyroRate;
    double RollGryoRate;
    double MagneticX;
    double MagneticY;
    double MagneticZ;
    double Current_Motor2;
    double Field_18;
    double Field_19;
    double Servo;
    double Hovergas;
    double Current;
    double Capacity;
    double Hight_Setpoint;
    double Field_25;
    double Field_26;
    double Compass_Setpoint;
    double I2C_Error;
    double BL_Limit;
    double GPS_Nick;
    double GPS_Roll;
  };
extern struct str_rawSensorData rawSensorData;

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

//NaviDataStruct

  typedef struct
  {
          unsigned char Version;                                     // version of the data structure
          GPS_Pos_t CurrentPosition;                      // see ubx.h for details
          GPS_Pos_t TargetPosition;
          GPS_PosDev_t TargetPositionDeviation;
          GPS_Pos_t HomePosition;
          GPS_PosDev_t HomePositionDeviation;
          unsigned char  WaypointIndex;                              // index of current waypoints running from 0 to WaypointNumber-1
          unsigned char  WaypointNumber;                             // number of stored waypoints
          unsigned char  SatsInUse;                                  // number of satellites used for position solution
          signed short Altimeter;                                  // hight according to air pressure
          signed short Variometer;                                 // climb(+) and sink(-) rate
          unsigned short FlyingTime;                                 // in seconds
          unsigned char  UBat;                                       // Battery Voltage in 0.1 Volts
          unsigned short GroundSpeed;                                // speed over ground in cm/s (2D)
          signed short Heading;                                    // current flight direction in ° as angle to north
          signed short CompassHeading;                             // current compass value in °
          signed char  AngleNick;                                  // current Nick angle in 1°
          signed char  AngleRoll;                                  // current Rick angle in 1°
          unsigned char  RC_Quality;                                 // RC_Quality
          unsigned char  FCStatusFlags;                              // Flags from FC
          unsigned char  NCFlags;                                    // Flags from NC
          unsigned char  Errorcode;                                  // 0 --> okay
          unsigned char  OperatingRadius;                            // current operation radius around the Home Position in m
          signed short TopSpeed;                                   // velocity in vertical direction in cm/s
          unsigned char  TargetHoldTime;                             // time in s to stay at the given target, counts down to 0 if target has been reached
          unsigned char  FCStatusFlags2;                             // StatusFlags2 (since version 5 added)
          signed short SetpointAltitude;                           // setpoint for altitude
          unsigned char  Gas;                                        // for future use
          unsigned short Current;                                    // actual current in 0.1A steps
          unsigned short UsedCapacity;                               // used capacity in mAh
  } __attribute__((packed)) NaviData_t;



  // ------- NCFlags -------------------------------------
  #define NC_FLAG_FREE                            0x01
  #define NC_FLAG_PH                              0x02
  #define NC_FLAG_CH                              0x04
  #define NC_FLAG_RANGE_LIMIT                     0x08
  #define NC_FLAG_NOSERIALLINK                    0x10
  #define NC_FLAG_TARGET_REACHED                  0x20
  #define NC_FLAG_MANUAL                          0x40
  #define NC_FLAG_GPS_OK                          0x80

  // ------- FCStatusFlags -------------------------------
  #define FC_STATUS_MOTOR_RUN                     0x01
  #define FC_STATUS_FLY                           0x02
  #define FC_STATUS_CALIBRATE                     0x04
  #define FC_STATUS_START                         0x08
  #define FC_STATUS_EMERGENCY_LANDING             0x10
  #define FC_STATUS_LOWBAT                        0x20
  #define FC_STATUS_VARIO_TRIM_UP                 0x40
  #define FC_STATUS_VARIO_TRIM_DOWN               0x80

  // ------- FCStatusFlags2 ------------------------------
  #define FC_STATUS2_CAREFREE_ACTIVE              0x01
  #define FC_STATUS2_ALTITUDE_CONTROL_ACTIVE      0x02
  #define FC_STATUS2_FAILSAFE_ACTIVE              0x04
  #define FC_STATUS2_OUT1                         0x08
  #define FC_STATUS2_OUT2                         0x10
  #define FC_STATUS2_RES1                         0x20
  #define FC_STATUS2_RES2                         0x40
  #define FC_STATUS2_RES3                         0x80

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

  //-------------------------------------------------------------------------------------------------------

#endif /* MKDEFINES_H_ */
