/*
   This program is for the aCTino robot.
   This program make the robot able to follow the wall on it's right by driving quarters of circle curves
   It contains the following features
      - Proportionnal controller to keep the robot at the right distance from the wall
      - Timer algorithm to manage the time
      - Dead-end detection (turn back)
      - End wall detection (turn to the right)
      - Front wall detection (turn to the left)
      - Detection of turning around an object (move straight)
*/

#include <RC5.h>
#include <acaBotM.h>

//Constants
//=================================================================================

//distance sensors connections "Sharp 2D120X  4 ... 30 cm" ------------------------
const int DIST_SENS_R = 0;  //analog input 0  (A0)
const int DIST_SENS_M = 1;  //analog input 1  (A1)
const int DIST_SENS_L = 2;  //analog input 2  (A2)

//'L'ine 'F'inders ----------------------------------------------------------------
const int LF_R = 3; //analog input 3 (A3)
const int LF_L = 4; //analog input 4 (A4)


//GLOBAL VARIABLES
//=================================================================================

int distSensR = 0;     // the Bot's three 'eyes' R/M/L to detect obstacles
int distSensM = 0;     // range: 0..1024 (connected to 10bit-ADC
int distSensL = 0;

int lineFinderR;
int lineFinderL;

unsigned long currentMillis = 0;
unsigned long previousMillisPrint = 0;
unsigned long previousMillisMotorControl = 0;


// ----------------- USER VARIABLES (Begin) ---------------------------------------
// ================================================================================

//Timer
unsigned long nextEvent     = 0;        //Time when the next event will start from the start of the program (in ms)
const unsigned int timeBase = 1000;     //Amount of time from the end of an avant to the next event (in ms)

//State machine
unsigned int robotState = 0;          //Current state of the state machine
#define STATE_SEARCH_WALL 0           //Searching for a wall
#define STATE_FOLLOW_WALL 1           //Follow the wall in the ritht side of the robot
#define STATE_TURN_SPOT   2           //Turn the robot on the spot
#define STATE_TURN_LEFT   3           //Turn the robot on the left
#define STATE_TURN_RIGHT  4           //Turn the robot on the right
#define STATE_TURN_BACK   5           //Turn the robot back

//Robot measurements
#define ROBOT_RADIUS 7.0                //Radius of the robot (in cm)
#define DISTANCE_SENSOR_OFFSET 6.0      //Distance from the middle of the robot and the right sensor (in cm)

//Distances
const float distWallSet      = 5.0;                                                                       //Expected distance from the robot to the wall in reality (in cm)
const float distWallAngleSet = (distWallSet / cos(M_PI / 4)) + ROBOT_RADIUS - DISTANCE_SENSOR_OFFSET;     //Expected distance from the robot to the wall for the right distance sensor (in cm)
const float leftCurveRadius  = ROBOT_RADIUS * 1.5;                                                        //Right curve radius (in cm)
const float radiusCurveRight = ROBOT_RADIUS + distWallSet;                                                //Left curve radius (in cm)
const float freeDrive        = (ROBOT_RADIUS + distWallSet) * tan(M_PI/4);                                //Free drive lenght (in cm)
const float distFront        = ROBOT_RADIUS + leftCurveRadius + distWallSet - DISTANCE_SENSOR_OFFSET;     //Detection distance from the wall to the front (in cm)

//Speeds
const float botOmega     = 45.0;                                              //Robot angular speed (during the right and left curves)
const float botForward_V = 10.0;                                              //Robot linear speed during the wall following (in cm/s)
const float botLeft_V    = (2 * M_PI * leftCurveRadius) / (360 / botOmega);   //Robot linear speed during the left curve (in cm/s)
const float botRight_V   = (2 * M_PI * radiusCurveRight) / (360 / botOmega);  //Robot linear speed during the right curve (in cm/s)
const float freeDrive_V  = freeDrive / 2;                                     //Robot linear speed during the free drive (in cm/s)
const float kp           = 10.0;                                              //Proportionnal controller adjustment coefficiant

//Measurement
float distanceL = 0.0;  //Left distance sensor value (in cm)
float distanceM = 0.0;  //Middle distance sensor value (in cm)
float distanceR = 0.0;  //Right distance sensor value (in cm)
float omega     = 0.0;  //Proportionnal controller value (in degrees)

//Counters
unsigned int curveRightCounter   = 0;   //Count the number of right turns in a row
unsigned int stateMachineEntries = 0;   //Count the time ticks inside the state machine

// ===============================================================================
// ------------------ USER VARIABLES (End)--------------------------------------


void setup() {

  serialPortEn    = 1;    //0: disabled, 1: enabled   --- via USB-Cable connected to PC
  serialPort1En   = 0;    //0: disabled, 1: enabled   --- XBEE-Carrier (wireless)
  serialBaudrate  = 9600; //max: 38400 Baud
  serialBaudrate1 = 9600; //max: 38400 Baud

  botHardwareInit();

  motorControlTimeInterval =   50;  // in ms!!!
  //PID-Control .........................................................
  kpPid = 5.0;
  kiPid = 1.7;
  kdPid = 5.0;

  rc5Address = 0;  //remote control:   addr: 0 -> TV1   5 -> VCR1   [8 -> CBL]   20 -> AUX1
  rc5JoystickSpeedForward = 15.0;  //Remote/Joystick: speedForward cm/s (keys: 2,8)
  rc5JoystickOmegaForward = 30.0;  //Remote/Joystick: omegaForward °/s (keys: 1,3,7,9)
  rc5JoystickOmegaSpot    = 90.0;  //Remote/Joystick: omegaSpot °/s (keys: 4,6)

  resetBotDynamics();                       // clear Dynamic Data "EncoderValues Left/Right" and nescessarily MotorControl Data
  // (the variables "Bot speed forward/omega" are NOT reset but keep their given value!)
  setBotForwardKinematics(0.0, 0.0, 0.0);   // (re-)set xt, yt, phit (= Bot's World Data)
  vOmega(0.0, 0.0);                         // drive Bot: Param.1: v (bot forward speed: cm/s), Param.2: omega (curve speed: °/s)
  // Note: Use ONLY this function to set Bot Speed!!!

  // ----------------- USER SETUP (Begin) ---------------------------------------
  // ================================================================================

  //Entry point of the state machine
  robotState = STATE_SEARCH_WALL;
  stateMachineEntries = 0;
  vOmega(botForward_V, 0.0);

  // ===============================================================================
  // ------------------ USER SETUP (End)--------------------------------------

} //end setup()



void loop() {

  // +++++++++++ ADMINISTRATION CODE +++ (better don't touch!) +++++++++++++++++++++++++++++++++++++++++++++++++++++

  currentMillis = millis();  //take snapshot of time

  rc5Joystick();

  if ((unsigned long)(currentMillis - previousMillisMotorControl) >= motorControlTimeInterval) {
    //in this time intervall not only pidMotorControl() is executed but also some other functions of importance

    readDistanceSensors();      //store readings in the global int-variables distSensL, distSensM, distSensR [range 0..1024]
    readLineFinders();          //store readings in the global int-variables lineFinderL, lineFinderR [range 0..1024]
    updateForwardKinematics();  //store readings in the global float-variables xt (cm), yt (cm), phit (Grad)
    pidMotorControl();

    previousMillisMotorControl += motorControlTimeInterval;
  }

  if ((unsigned long)(currentMillis - previousMillisPrint) >= 1000) {
    /*depending on the global variable "serialPort" in setup() the print-output of the following functions is sent to
      either to Serial0 (USB) or Serial1 (XBee)*/

    //printDistanceSensors();    // d: distSensL, distSensM, distSensR [range 0..1024]
    //printLineFinders();        // l: lineFinderL, lineFinderR
    //printEncoderPulses();      // e: numberEncoderPulsesLeft, numberEncoderPulsesRight
    //printForwardKinematics();  // k: xt, yt, phit

    previousMillisPrint += 1000;
  }

  // +++++++++++ USER CODE (Begin) ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // ==========================================================================================

  /*Note: You can read (but not change!) the following global variables for the benefits of your Code:
    the Bot's Pacemaker Variables: (type "long") numberEncoderPulsesLeft, numberEncoderPulsesRight
                                   (type "float") botSpeedForward [actual v, cm/s], botSpeedOmega [deg/s]
    the Bot's Eyes Variables     : (type "int")  distSensR, distSensM, distSensL [range 0..1024]
    the Bot's Look Down          : (type "int")  lineFinderL; lineFinderR [range 0..1024]
    the Bot's World Coordinates  : (type "float") xt (cm), yt (cm), phit (Grad 0..380°)
  */

  //Measure the distance of each distance sensors
  measureDistance();    

  //Beginning of the state machine
  switch (robotState)
  {

    case STATE_SEARCH_WALL :    //If there is a wall in the left or in front of the robot
                                if( (distanceL < distWallSet) || (distanceM < distWallSet) )
                                {
                                  //Turn the robot on the spot
                                  //Prepare the state STATE_TURN_SPOT
                                  vOmega(0.0, botOmega);
                                  robotState = STATE_TURN_SPOT;
                                }
                                
                                //If there is a wall in the right of the robot
                                else if(distanceR < (distWallSet * 1.5))
                                {
                                  //Follow the wall in the right of the robot
                                  //Prepare the state STATE_FOLLOW_WALL
                                  pController();
                                  robotState = STATE_FOLLOW_WALL;
                                }
                                break;
                                
      case STATE_TURN_SPOT :    //Reset curve right counter
                                curveRightCounter = 0;
                                
                                //If there is a wall in the right of the robot
                                if( (distanceR > distWallAngleSet) && (distanceM > (distWallSet * 1.5)) )
                                {
                                  //Follow the wall in the right of the robot
                                  //Prepare the state STATE_FOLLOW_WALL
                                  pController();
                                  robotState = STATE_FOLLOW_WALL;
                                }
                                break;
                                
    case STATE_FOLLOW_WALL :  //If the robot is in a dead-end
                              if(  (distanceM < distWallSet) && (distanceL < (distWallAngleSet * 2)) )
                              {
                                //Turn the robot back
                                //Prepare the state STATE_TURN_BACK
                                vOmega(0.0, botOmega);
                                robotState = STATE_TURN_BACK;
                              }
                              
                              //If there is a wall in front of the robot
                              else if( (distanceM <= distFront) && (distanceL > (distWallAngleSet * 2)) )
                              {
                                //Turn the robot to the left
                                //Prepare the state STATE_TURN_LEFT
                                vOmega(botLeft_V, botOmega);
                                robotState = STATE_TURN_LEFT;
                              }
                        
                              //If the robot arrive at the end of the wall
                              else if (distanceR >= 20.0)
                              {
                              
                                if(curveRightCounter < 4)
                                {
                                  //Turn the robot to the right
                                  //Prepare the state STATE_TURN_RIGHT
                                  vOmega(freeDrive_V,0.0);
                                  robotState = STATE_TURN_RIGHT;
                                }

                                //If the robot is turning around an obstacle
                                else
                                {
                                  //Prepare the state STATE_STRAIGHT
                                  vOmega(botForward_V, 0.0);
                                  robotState = STATE_SEARCH_WALL;
                                }
                              }
                        
                              else
                              {
                                //Follow the wall in the right of the robot
                                pController();
                              }
                              break;

    case STATE_TURN_LEFT :    //Reset curve right counter
                              curveRightCounter = 0;

                              //If there is a wall in front of the robot
                              if(distanceM <= distWallAngleSet)
                              {
                                //Reset state machine ticks counter
                                stateMachineEntries = 0;
                                  
                                //Turn the robot on the spot
                                //Prepare the state STATE_TURN_SPOT
                                vOmega(0.0, botOmega);
                                robotState = STATE_TURN_SPOT;
                              }
                              
                              //Turn the robot during 2 seconds to turn to the left
                              else if (currentMillis >= nextEvent)
                              {
                                if (stateMachineEntries == 2)
                                {
                                  //Reset state machine ticks counter
                                  stateMachineEntries = 0;
                        
                                  //Follow the wall in the right of the robot
                                  //Prepare the state STATE_FOLLOW_WALL
                                  pController();
                                  robotState = STATE_FOLLOW_WALL;
                                }
                                else
                                {
                                  //Increment the state machine tick counter
                                  stateMachineEntries++;
                                }

                                //Increment counter
                                nextEvent = currentMillis + timeBase;
                              }
                              else if( (distanceM > ( distWallSet * 3)) && (distanceR > distWallAngleSet ) )
                              {
                                //Reset state machine ticks counter
                                stateMachineEntries = 0;
                        
                                //Follow the wall in the right of the robot
                                //Prepare the state STATE_FOLLOW_WALL
                                pController();
                                robotState = STATE_FOLLOW_WALL;
                              }
                              break;

    case STATE_TURN_RIGHT :   //If there is a wall in front of the robot
                              if(distanceM <= distFront)
                              {
                                //Reset state machine tick counter
                                stateMachineEntries = 0;
                                
                                //Turn the robot to the left
                                //Prepare the state STATE_TURN_LEFT
                                vOmega(botLeft_V, botOmega);
                                robotState = STATE_TURN_LEFT;
                              }

                              //Move the robot straight during 2 seconds before turing on the right
                              else if (currentMillis >= nextEvent)
                              {
                                if(stateMachineEntries == 1)
                                {
                                  //Increment curve right counter
                                  curveRightCounter++;

                                  //Turn the robot to the right
                                  stateMachineEntries++;
                                  vOmega(botRight_V, -botOmega);
                                }

                                //Turn the robot during 2 seconds to turn to the right
                                else if ( (stateMachineEntries == 3) && (distanceR <= distWallAngleSet) )
                                {
                                  //Reset counter
                                  stateMachineEntries = 0;
                        
                                  //Follow the wall in the right of the robot
                                  //Prepare the state STATE_FOLLOW_WALL
                                  pController();
                                  robotState = STATE_FOLLOW_WALL;
                                }

                                //If there is still no wall in the right side of the robot after the right curve continue to turn on the right during 2 seconds
                                else if (stateMachineEntries == 5)
                                {
                                  //Reset counter
                                  stateMachineEntries = 0;
                        
                                  //Follow the wall in the right of the robot
                                  //Prepare the state STATE_FOLLOW_WALL
                                  pController();
                                  robotState = STATE_FOLLOW_WALL;
                                }

                                else
                                {
                                  //Increment the state machine tick counter
                                  stateMachineEntries++;
                                }

                                //Increment timer
                                nextEvent = currentMillis + timeBase;
                              }
                              break;

    case STATE_TURN_BACK :    //Reset curve right counter
                              curveRightCounter = 0;

                              //Turn the robot on the spot during 4 seconds to go back
                              if(currentMillis >= nextEvent)
                              {
                                if(stateMachineEntries == 4)
                                {
                                  //Reset state machine counter
                                  stateMachineEntries = 0;

                                  //Follow the wall in the right of the robot
                                  //Prepare the state STATE_FOLLOW_WALL
                                  pController();
                                  robotState = STATE_FOLLOW_WALL;
                                }
                                else
                                {
                                  //Increment the state machine tick counter
                                  stateMachineEntries++;
                                }

                                //Increment timer
                                nextEvent = currentMillis + timeBase;
                              }

                              //If there is no more wall in front of the robot
                              else if( (distanceM > (distWallAngleSet * 3)) && (distanceR > (distWallAngleSet)) )
                              {
                                //Reset state machine counter
                                stateMachineEntries = 0;
                                
                                //Follow the wall in the right of the robot
                                //Prepare the state STATE_FOLLOW_WALL
                                pController();
                                robotState = STATE_FOLLOW_WALL;
                              }
                              break;
  }

  // ==========================================================================================
  // +++++++++++ USER CODE (End) ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


} //end loop()



void printDistanceSensors() {

  if (serialPortEn == 1) {
    Serial.print('d'); //identifier d stands for d_istance sensor data
    Serial.print(distSensL);
    Serial.print(',');
    Serial.print(distSensM);
    Serial.print(';');
    Serial.println(distSensR);
    //Serial.print('.');
  }
  if (serialPort1En == 1) {
    Serial1.print('d'); //identifier d stands for d_istance sensor data
    Serial1.print(distSensL);
    Serial1.print(',');
    Serial1.print(distSensM);
    Serial1.print(';');
    Serial1.println(distSensR);
    //Serial1.print('.');
  }

} //end printDistanceSensors()


void printLineFinders() {

  if (serialPortEn == 1) {
    Serial.print('l'); //identifier l stands for l_ine finder data
    Serial.print(lineFinderL);
    Serial.print(',');
    Serial.println(lineFinderR);
    //Serial.print('.');
  }
  if (serialPort1En == 1) {
    Serial1.print('l'); //identifier l stands for l_ine finder data
    Serial1.print(lineFinderL);
    Serial1.print(',');
    Serial1.println(lineFinderR);
    //Serial1.print('.');
  }

} //end printLineFinders()


void readDistanceSensors() {
  /*Measuring Cycle of the Sharp Distance Sensors is minimum 16,5ms */

  distSensL = analogRead(DIST_SENS_L);
  distSensM = analogRead(DIST_SENS_M);
  distSensR = analogRead(DIST_SENS_R);

} //end readDistanceSensors()


void readLineFinders() {
  lineFinderR = analogRead(LF_R);
  lineFinderL = analogRead(LF_L);

} //end readLineFinders()

void measureDistance() {
  /*Convert the analog value from the distance sensors in cm*/

  distanceR = (float)((2076 / (distSensR * 0.545 - 11)) - 1);
  distanceM = (float)((2076 / (distSensM * 0.545 - 11)) - 1);
  distanceL = (float)((2076 / (distSensL * 0.545 - 11)) - 1);


  //Limit the value at the limits of the sensors
  if (distanceR > 30.0)
  {
    distanceR = 30.0;
  }
  else if (distanceR < 0.0)
  {
    distanceR = 30.0;
  }

  if (distanceL > 30.0)
  {
    distanceL = 30.0;
  }
  else if (distanceL < 0.0)
  {
    distanceL = 30.0;
  }

  if (distanceM > 30.0)
  {
    distanceM = 30.0;
  }
  else if (distanceM < 0.0)
  {
    distanceM = 30.0;
  }
} //end measureDistance()


void pController() {
  /*Keep the robot at the expected distance from the wall*/
  omega = (distWallAngleSet - distanceR) * kp ;

  if(omega > 30.0)
  {
    omega = 30.0;
  }
  else if(omega < -30.0)
  {
    omega = -30.0;
  }
  vOmega(botForward_V , omega);

} //end pController()

