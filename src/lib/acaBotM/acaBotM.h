/* 
  acaBotM.h --- Arduino Library File
     
  Andreas Döpkens 

  rev. 2015-04-10   
  
*/

#ifndef ACABOTM_H
#define ACABOTM_H

     
//CONSTANTS: Physical connections on aCTino-Bot and its Geometry
//================================================================================

//Remote Control (RC5) -----------------------------------------------------------
const int RC5_PIN = 14;  //D14 - MISO

#define RC5_POW     12     // fpai  0   (fpai: functionPointers array index)
#define RC5_1        1     // fpai  1
#define RC5_2        2     // fpai  2
#define RC5_3        3     // fpai  3
#define RC5_4        4     // fpai  4
#define RC5_5        5     // fpai  5
#define RC5_6        6     // fpai  6
#define RC5_7        7     // fpai  7
#define RC5_8        8     // fpai  8
#define RC5_9        9     // fpai  9
#define RC5_PLAY    43     // fpai 10
#define RC5_PAUSE   14     // fpai 11
#define RC5_STOP    41     // fpai 12
#define RC5_FF      10     // fpai 13 //fast forward
#define RC5_FR      11     // fpai 14 //fast rewind
#define RC5_VOL_P   16     // fpai 15 //P: plus
#define RC5_VOL_M   17     // fpai 16 //M: minus
#define RC5_CH_P    57     // fpai 17
#define RC5_CH_M    56     // fpai 18


//H-Bridge connections ------------------------------------------------------------
const int ENA = 10;   //Enable left motor  - motor speed by PWM  (~D10)  Timer1
const int ENB = 11;   //Enable right motor        ...            (~D11)  Timer0 -> Timer1 by Martin
const int IN1 =  7;   //H Bridge, direction left motor   (D7)
const int IN2 =  8;   //  ...                            (D8)
const int IN3 = 12;   //H Bridge, direction right motor (D12)
const int IN4 = 13;   //  ...

//encoder connections -------------------------------------------------------------
const int ENCA_L =  2;  //Encoder A left  motor - ext. Int. 1 ( D2)
const int ENCA_R =  3;  //Encoder A right motor - ext. Int. 0 ( D3)
const int ENCB_L = 15;  //Encoder B left  motor               (D15 - SCK)
const int ENCB_R = 16;  //Encoder B right motor               (D16 - MOSI)


//exported GLOBAL "user" VARIABLES and corresponding Default-Values
//=================================================================================

const int  serialPortEnDefault    = 1;    //[aka UART] USB-Communication with Serial Monitor
const long serialBaudrateDefault  = 9600;
const int  serialPort1EnDefault   = 0;    //           XBEE  ..            .. Zig-Bee, Wifi-Bee, Bluetooth-Bee etc.       
const long serialBaudrate1Default = 9600; 

const int rc5AddressDefault = 0;    //addr: 0 -> TV1   5 -> VCR1   [8 -> CBL]   20 -> AUX1
const float rc5JoystickSpeedForwardDefault = 15.0;  //speedForward cm/s (keys: 2,8) 
const float rc5JoystickOmegaForwardDefault = 30.0;  //omegaForward °/s (keys: 1,3,7,9)
const float rc5JoystickOmegaSpotDefault    = 90.0;  //omegaSpot °/s (keys: 4,6)

const unsigned int motorControlTimeIntervalDefault = 50;  // in ms!!!

const float kpPidDefault = 5.0;  // 5.0 for MotorControlTimeInterval =  10ms, 50ms, 100ms
const float kiPidDefault = 1.7;  // 1.7 for MotorControlTimeInterval =  10ms, 50ms, 100ms
const float kdPidDefault = 5.0;  //30.0 for MotorControlTimeInterval =  10ms
                                 // 5.0  ..              ..          =  50ms
                                 // 2.0  ..              ..          = 100ms

const float kpPiDefault  = 0.8;  //KP-Factor for function piMotorControl() [50ms]
const float kpPDefault   = 7.0;  //KP-Factor for function pMotorControl()  [50ms]
                             

//Communications ...................................................................

extern int  serialPortEn; 
extern int  serialPort1En;
extern long serialBaudrate;
extern long serialBaudrate1;

extern bool emergencyStop;

extern float xt;
extern float yt;
extern float phit;

extern long  numberEncoderPulsesLeft;
extern long  numberEncoderPulsesRight;

extern int rc5Address;  
extern float rc5JoystickSpeedForward; 
extern float rc5JoystickOmegaForward; 
extern float rc5JoystickOmegaSpot;

typedef void (* rc5FunctionPointer) ();  //function pointers to handle remote control buttons

//Motor Control .....................................................................
extern unsigned int   motorControlTimeInterval;  // in ms!!!
                         
extern float kpPid;  
extern float kiPid;  
extern float kdPid; 

extern float kpPi;  //KP-Factor for function piMotorControl() [50ms]
extern float kpP;   //KP-Factor for function pMotorControl()  [50ms]

//Bot Geometry
extern int   epPerRev;      //Encoder pulses per revolution
extern double wheelRadius;  //Wheel-Radius/cm
extern float wheelDist;     //Wheel-Distance/cm

extern bool invertPolarityLeft;   // invert direction for left Motor
extern bool invertPolarityRight;  // invert direction for left Motor
extern bool invertEncoderLeft;    // invert left encoder count direction
extern bool invertEncoderRight;   // invert right encoder count direction

//**************************************************************************************

void rc5AttachFunction(int index, rc5FunctionPointer f);

void resetBotDynamics();
void setBotForwardKinematics(float x, float y, float fi);

// READ:  this function uses the contents of the global variables 
//             botSpeedForward, botSpeedOmega, motorSpeedRight, and motorSpeedLeft
// this function updates the global variables xt,yt, and phit that are the Bot's world coordinates
// it is NOT shown where the Bot IS but where it is ASSUMED TO BE
void updateForwardKinematics();


void printForwardKinematics();

// uses remote control's numbers 1..9 as joystick to drive Bot
// thisSpeedForward cm/s (2,8), thisOmegaForward Deg/s (1,3,7,9), thisOmegaSpot Deg/s (4,6)
//                              [slower curve]                      [faster curve]
void rc5Joystick();


//this function calculates the left and right motor Speed out of the given Forward- and Curve-Speed
//the data motorSpeedLeft and motorSpeedRight are needed by the motor control
void vOmega(float v, float omega);


void pidMotorControl();
void piMotorControl();
void pMotorControl();


void printEncoderPulses();

void botHardwareInit();

void setMotorFastStopRight();
void setMotorFastStopLeft();
void setMotorDirectionRight(char direction);
void setMotorDirectionLeft(char direction);
void setMotorPwmRight(int value);
void setMotorPwmLeft(int value);
void countEncoderPulsesLeft();
void countEncoderPulsesRight();

#endif

