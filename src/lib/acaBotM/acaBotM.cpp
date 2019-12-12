/* 
  acaBotM.cpp --- Arduino Library File       
     
  Andreas D�pkens

  rev. 2015-04-10  
  
*/ 

#include "acaBotM.h"
#include <Arduino.h>
#include <RC5.h>     //http://www.elektronik-labor.de/Arduino/RC5.html
#include "wiring_private.h" //needed for Martin's "substitute code"

RC5 rc5(RC5_PIN);                

//GLOBAL "state" VARIABLES
//=================================================================================

float reqEpSumLeft=0;      //Global vs. Static for: pid- and pi-MotorControl()
float reqEpSumRight=0; 
int   contrDevOldLeft=0;   //Global vs. Static for: pid-MotorControl()
int   contrDevOldRight=0;
long  oldNumberEncoderPulsesLeft=0; //Global vs. Static for: p-MotorControl()
long  oldNumberEncoderPulsesRight=0;

//Bot's Kinematics ................................................................... 
long  numberEncoderPulsesLeft  = 0;
long  numberEncoderPulsesRight = 0;
long  prevNumberEncoderPulsesLeft = 0;
long  prevNumberEncoderPulsesRight = 0;
float botSpeedForward = 0.0;  // v:     contains actual Bot Forward Speed cm/s
float botSpeedOmega   = 0.0;  // omega: contains actual Bot Curve Speed deg/s
float motorSpeedLeft  = 0.0;  // evaluating v and omega, the results are matched on the two wheels
float motorSpeedRight = 0.0;  // (left and right) of the Bot [cm/s]
int   motorPwmLeft    = 0;    // the motor control function provides both motors with an appropriate 
int   motorPwmRight   = 0;    // PWM-Voltage (0 .. 255)

//Bot's World Coordinates .............................................................
float xt  = 0.0;  //cm
float yt  = 0.0;  //cm
float phit= 0.0;  //degree

//Bot Geometry ......................................................................
//Bot Geometry
int   epPerRev           = 528;    //Encoder pulses per revolution
double wheelRadius       = 2.80;   //Wheel-Radius/cm
float wheelDist          = 9.7;    //Wheel-Distance/cm


bool emergencyStop = 0;

//GLOBAL "user" VARIABLES
//=================================================================================

//Communications ...................................................................
int rc5Address  = rc5AddressDefault;    

int serialPortEn     = serialPortEnDefault;    
int serialPort1En    = serialPort1EnDefault;
long serialBaudrate  = serialBaudrateDefault;
long serialBaudrate1 = serialBaudrate1Default;

//function pointers to handle remote control buttons
typedef void (* rc5FunctionPointer) (); //see .h !!!
rc5FunctionPointer functionPointers[19];

float rc5JoystickSpeedForward = rc5JoystickSpeedForwardDefault; 
float rc5JoystickOmegaForward = rc5JoystickOmegaForwardDefault; 
float rc5JoystickOmegaSpot    = rc5JoystickOmegaSpotDefault;

//Motor Control .....................................................................
unsigned int motorControlTimeInterval = motorControlTimeIntervalDefault;  
                         
float kpPid = kpPidDefault;  
float kiPid = kiPidDefault;  
float kdPid = kdPidDefault;  

float kpPi  = kpPiDefault;  
float kpP   = kpPDefault; 

bool invertPolarityLeft = false;
bool invertPolarityRight = false;
bool invertEncoderLeft = false;
bool invertEncoderRight = false;

//**************************************************************************************

void rc5AttachFunction(int index, rc5FunctionPointer f) {
  functionPointers[index] = f;
}


void rc5_pow() { //functionPointers[12]'s action
  
  static float oldBotSpeedForward=0;
  static float oldBotSpeedOmega=0;


  if(emergencyStop==0){  //stop Bot and set emergencyStop to 1
    //save old v and omega
    oldBotSpeedForward=botSpeedForward;
    oldBotSpeedOmega  =botSpeedOmega;       
    //stop Bot
    vOmega(0, 0);
          
    emergencyStop=1;
  }
  else {  //go Bot and reset emergencyStop to 0
    //restore old values 
    vOmega(oldBotSpeedForward, oldBotSpeedOmega);
          
    emergencyStop=0;
  }
} //end rc5_pow()

void rc5_1() {vOmega(rc5JoystickSpeedForward,  rc5JoystickOmegaForward);}
void rc5_2() {vOmega(rc5JoystickSpeedForward, 0);}
void rc5_3() {vOmega(rc5JoystickSpeedForward, -rc5JoystickOmegaForward);}
void rc5_4() {vOmega(0, rc5JoystickOmegaSpot);}
void rc5_5() {vOmega(0, 0); resetBotDynamics();}
void rc5_6() {vOmega(0, -rc5JoystickOmegaSpot);}
void rc5_7() {vOmega(-rc5JoystickSpeedForward, rc5JoystickOmegaForward);}
void rc5_8() {vOmega(-rc5JoystickSpeedForward, 0);}
void rc5_9() {vOmega(-rc5JoystickSpeedForward, -rc5JoystickOmegaForward);}


void rc5Joystick() {
  // uses remote control's numbers 1..9 as joystick to drive Bot
  //    rc5JoystickSpeedForward cm/s (2,8), rc5JoystickOmegaForward deg/s (1,3,7,9), rc5JoystickOmegaSpot deg/s (4,6)
  //                                                [slower curve]                      [faster curve]
  // uses Power-Button as Emergency-Stop
  //    if Power-Button is hit again after an Emergency Stop the Bot resumes driving at previous speed  

  //by remote control sent data
  unsigned char toggle;
  static unsigned char prevToggle=255;
  unsigned char command=0;
  unsigned char address;


  if (rc5.read(&toggle, &address, &command)){ // read remote control data

    if (address == rc5Address) {
      
      //in case of Joystick(1-9): this function requires positive omega only
      rc5JoystickSpeedForward=fabs(rc5JoystickSpeedForward);
      rc5JoystickOmegaForward=fabs(rc5JoystickOmegaForward);
      rc5JoystickOmegaSpot   =fabs(rc5JoystickOmegaSpot);
    
      switch (command) {

        //++++++++++ Emergency Stop +++++++++++++++++++
        case RC5_POW: 

         if(prevToggle != toggle) {
           prevToggle=toggle;
           if(functionPointers[0]) functionPointers[0]();
          }

          break;

	   //++++++++++ joystick ++[numbers 1-9] +++++++++
        case RC5_2:    
            if(functionPointers[2]) functionPointers[2]();
          break;
        case RC5_8:
            if(functionPointers[8]) functionPointers[8]();
           break;
        case RC5_4:    
            if(functionPointers[4]) functionPointers[4]();
          break;        
        case RC5_6:
            if(functionPointers[6]) functionPointers[6]();
           break;        
        
        case RC5_1:    
            if(functionPointers[1])functionPointers[1]();
           break;
        case RC5_3:    
            if(functionPointers[3])functionPointers[3]();
          break;
        case RC5_7:    
            if(functionPointers[7])functionPointers[7]();
           break;
        case RC5_9:    
            if(functionPointers[9])functionPointers[9]();
          break;
        
        case RC5_5: 
            if(functionPointers[5])functionPointers[5]();
           break;

        //++++++++++ video control buttons ++++++++++++++

       case RC5_PLAY: 
            if(functionPointers[10])functionPointers[10]();
           break;
       case RC5_PAUSE: 
            if(functionPointers[11])functionPointers[11]();
           break;
       case RC5_STOP: 
            if(functionPointers[12])functionPointers[12]();
           break;
       case RC5_FF: 
            if(functionPointers[13])functionPointers[13]();
           break;
       case RC5_FR: 
            if(functionPointers[14])functionPointers[14]();
           break;
       case RC5_VOL_P: 
            if(functionPointers[15])functionPointers[15]();
           break;
       case RC5_VOL_M: 
            if(functionPointers[16])functionPointers[16]();
           break;
       case RC5_CH_P: 
            if(functionPointers[17])functionPointers[17]();
           break;
       case RC5_CH_M: 
            if(functionPointers[18])functionPointers[18]();
           break;
      
      } //end switch    
    } // end if(address==rcAddress)
  } // end if(rc5read() ...)
  
} //end rc5Joystick()


void resetBotDynamics(){

  reqEpSumLeft=0;                //pid- and pi-MotorControl()
  reqEpSumRight=0; 
  contrDevOldLeft=0;             //pid-MotorControl()
  contrDevOldRight=0;
  oldNumberEncoderPulsesLeft=0;  //p-MotorControl()
  oldNumberEncoderPulsesRight=0;
  
  numberEncoderPulsesLeft  = 0;
  numberEncoderPulsesRight = 0;
  prevNumberEncoderPulsesLeft = 0;
  prevNumberEncoderPulsesRight = 0;

} //end resetBotDynamics()


void setBotForwardKinematics(float x, float y, float fi) {
  
  xt   =  x;
  yt   =  y;
  phit = fi/180.0*M_PI; //convert phit (given in deg) to rad for internal calculations
  
} //end setBotForwardKinematics()



// Update odometry by distances
void updateOdometryD(double dl,double dr)
{
  double dv,dw;
  double l;
  
  l=(double)wheelDist;
  
  dv=(dr + dl)/2.0;      // Forward motion   dl/dr = distance left/right (cm)
  dw=(dr - dl)/l;        // Rotation         dw    = difference omega (rad) 

  xt+=cos(phit+dw/2.0)*dv; // Update pose (x)
  yt+=sin(phit+dw/2.0)*dv; // Update pose (y)
  phit+=dw;              // Update angle
  
  // Keep angle in bounds (-3.14 ... 3.14)
  if (phit>M_PI) { phit-=2*M_PI; }
  else if (phit<-M_PI) { phit+=2*M_PI; }
}


// Update odometry by encoder ticks
void updateForwardKinematics()
{
  float epPerCm = epPerRev / (2.0 * M_PI * wheelRadius);
  updateOdometryD( (double)(numberEncoderPulsesLeft - prevNumberEncoderPulsesLeft)/epPerCm,
                   (double)(numberEncoderPulsesRight - prevNumberEncoderPulsesRight)/epPerCm);

  prevNumberEncoderPulsesLeft  = numberEncoderPulsesLeft;
  prevNumberEncoderPulsesRight = numberEncoderPulsesRight;
}


void printForwardKinematics() {
  
  if(serialPortEn==1) {
    Serial.print('k'); //identifier k stands for k_inematics data
    Serial.print(xt);
    Serial.print(',');
    Serial.print(yt);
    Serial.print(';');
    Serial.println(phit*180.0/M_PI); //convert internal rad-phit to deg
    //Serial.print('.');
  }
  if (serialPort1En==1) {
    Serial1.print('k'); //identifier k stands for k_inematics data
    Serial1.print(xt);
    Serial1.print(',');
    Serial1.print(yt);
    Serial1.print(';');
    Serial1.println(phit*180.0/M_PI); //convert internal rad-phit to deg
    //Serial1.print('.');    
  }

} //end printForwardKinematics()


void vOmega(float v, float omega) {
  //this function calculates the left and right motor Speed out of the given Forward- and Curve-Speed
  //the data motorSpeedLeft and motorSpeedRight are needed by the motor control
  
  //save parameter values in global variables for use in function forwardKinematics()
  //[is this Saving really nescessary?????]
  botSpeedForward=v;
  botSpeedOmega=omega;
  
  omega = omega*M_PI/180; //convert parameter-omega from Grad in Rad

  motorSpeedLeft=v-omega*wheelDist/2;    
  motorSpeedRight=v+omega*wheelDist/2; 

} //end vOmega()


void pidMotorControl(){

    float epPerCm = epPerRev / (2.0 * M_PI * wheelRadius);
    float reqEpLeft  =  motorSpeedLeft*epPerCm*motorControlTimeInterval/1000.0;  //required Encoder Pulses
    float reqEpRight =  motorSpeedRight*epPerCm*motorControlTimeInterval/1000.0;
    int contrDevLeft;  //control deviation (Regelabweichung)
    int contrDevRight; 
/*  Global vs. Static:  static float reqEpSumLeft=0; static float reqEpSumRight=0; 
                        static int   contrDevOldLeft; static int   contrDevOldRight;
*/

    //control left motor 
    contrDevLeft = reqEpSumLeft - numberEncoderPulsesLeft;
    motorPwmLeft=(int)(motorSpeedLeft*kpPid+(float)contrDevLeft*kiPid+(contrDevLeft-contrDevOldLeft)*kdPid); //if contrDevLeft>0 then Motor too slow, too little EncPulses
    if(motorPwmLeft>255) motorPwmLeft=255;      //motorPwmLeft (= actuating variable, dt. Stellgröße) can be pos. or neg.
    else if(motorPwmLeft<-255)motorPwmLeft=-255; 
    
    //control right motor 
    contrDevRight = reqEpSumRight - numberEncoderPulsesRight;
    motorPwmRight=(int)(motorSpeedRight*kpPid+(float)contrDevRight*kiPid+(contrDevRight-contrDevOldRight)*kdPid);
    if(motorPwmRight>255) motorPwmRight=255; 
    else if(motorPwmRight<-255) motorPwmRight=-255;
       
    reqEpSumLeft+=reqEpLeft;        // Integration
    reqEpSumRight+=reqEpRight;

    contrDevOldLeft=contrDevLeft;   // Differentiation
    contrDevOldRight=contrDevRight;

    setMotorPwmLeft(motorPwmLeft);
    setMotorPwmRight(motorPwmRight);
      
} //end pidMotorControl()


void piMotorControl(){

    float epPerCm = epPerRev / (2.0 * M_PI * wheelRadius);
    float reqEpLeft  =  motorSpeedLeft*epPerCm*motorControlTimeInterval/1000.0;  //required Encoder Pulses
    float reqEpRight =  motorSpeedRight*epPerCm*motorControlTimeInterval/1000.0;
    int contrDevLeft;  //control deviation (Regelabweichung)
    int contrDevRight; 
    /* Global vs. Static:  static float reqEpSumLeft=0; static float reqEpSumRight=0; */

    //control left motor 
    contrDevLeft = reqEpSumLeft - numberEncoderPulsesLeft;
    motorPwmLeft=(int)((float)contrDevLeft*kpPi); //if contrDevLeft>0 then Motor too slow, too little EncPulses
    if(motorPwmLeft>255) motorPwmLeft=255;      //motorPwmLeft (= actuating variable, dt. Stellgröße) can be pos. or neg.
    else if(motorPwmLeft<-255)motorPwmLeft=-255; 
    
    //control right motor 
    contrDevRight = reqEpSumRight - numberEncoderPulsesRight;
    motorPwmRight=(int)((float)contrDevRight*kpPi);
    if(motorPwmRight>255) motorPwmRight=255; 
    else if(motorPwmRight<-255) motorPwmRight=-255;
       
    reqEpSumLeft+=reqEpLeft;  //Integration
    reqEpSumRight+=reqEpRight;

    setMotorPwmLeft(motorPwmLeft);
    setMotorPwmRight(motorPwmRight);
      
} //end piMotorControl()


void pMotorControl(){

    float epPerCm = epPerRev / (2.0 * M_PI * wheelRadius);
    float reqEpLeft  = (motorSpeedLeft*epPerCm*motorControlTimeInterval/1000.0);
    float reqEpRight = (motorSpeedRight*epPerCm*motorControlTimeInterval/1000.0);
    int contrDevLeft; //control deviation (Regelabweichung)
    int contrDevRight; 
    long actNumberEncoderPulsesLeft;
    long actNumberEncoderPulsesRight;
    /* Global vs. Static:  static long oldNumberEncoderPulsesLeft=0; static long oldNumberEncoderPulsesRight=0; */
  
    actNumberEncoderPulsesLeft=numberEncoderPulsesLeft-oldNumberEncoderPulsesLeft;
    oldNumberEncoderPulsesLeft=numberEncoderPulsesLeft;
    actNumberEncoderPulsesRight=numberEncoderPulsesRight-oldNumberEncoderPulsesRight;
    oldNumberEncoderPulsesRight=numberEncoderPulsesRight;

    //control left motor 
    contrDevLeft = reqEpLeft - actNumberEncoderPulsesLeft;
    motorPwmLeft=(int)((float)contrDevLeft*kpP); 
    if(motorPwmLeft>255) motorPwmLeft=255; 
    else if(motorPwmLeft<-255) motorPwmLeft=-255;

    //control right motor 
    contrDevRight = reqEpRight - actNumberEncoderPulsesRight;
    motorPwmRight=(int)((float)contrDevRight*kpP);
    if(motorPwmRight>255) motorPwmRight=255; 
    else if(motorPwmRight<-255) motorPwmRight=-255;
     
    setMotorPwmLeft(motorPwmLeft);
    setMotorPwmRight(motorPwmRight);
    
} //end pMotorControl()


void printEncoderPulses(){
    
  if(serialPortEn==1) {
    Serial.print('e'); //identifier e stands for e_ncoder data
    Serial.print(numberEncoderPulsesLeft);
    Serial.print(',');
    Serial.println(numberEncoderPulsesRight);
    //Serial.print('.');
  }
  if (serialPort1En==1) {
    Serial1.print('e'); //identifier e stands for e_ncoder data
    Serial1.print(numberEncoderPulsesLeft);
    Serial1.print(',');
    Serial1.println(numberEncoderPulsesRight);
    //Serial1.print('.');
  }

} //end printEncoderPulses()


void botHardwareInit() {
  
  pinMode(IN1, OUTPUT);    //IN1 directionChannel left Wheel
  pinMode(IN2, OUTPUT);    //IN2     ... 
  pinMode(IN3, OUTPUT);    //IN3 directionChannel right Wheel
  pinMode(IN4, OUTPUT);    //IN4     ...

  pinMode(ENCA_R, INPUT);   //Encoder pulses Right wheel Channel A
  pinMode(ENCA_L, INPUT);   //Encoder pulses Left  wheel Channel A
  attachInterrupt(0, countEncoderPulsesRight, RISING);
  attachInterrupt(1, countEncoderPulsesLeft, RISING);
  pinMode(ENCB_R, INPUT);   //Encoder pulses Right wheel Channel B
  pinMode(ENCB_L, INPUT);   //Encoder pulses Left  wheel Channel B
  
  pinMode(ENA, OUTPUT);     //PWM Control Signal for left motor
  pinMode(ENB, OUTPUT);     //PWM Control Signal for right motor

  /* Note: The frequency for the two PWM Signals ENA and ENB are by default set to 1KHz, driven by Timer1. This 
     rather low frequency causes an audible and annoying sound on the left and right motor. To avoid this, the PWM  
     frequency on Timer1 is increased to about 30KHz. */
  TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz

  if(serialPortEn==1) {
    Serial.begin(serialBaudrate);
    //while (!Serial);     //this code has meaning only for Leonardo
  }
  if(serialPort1En==1) {
    Serial1.begin(serialBaudrate1); //for XBee (on Leonardo: pin D0(RX) and D1(TX)
    //while (!Serial1);
  }

  //initialize some elements of array of functionPointers[] (fpai, see above)
  rc5AttachFunction( 0, rc5_pow);  
  rc5AttachFunction( 1, rc5_1);  
  rc5AttachFunction( 2, rc5_2);  
  rc5AttachFunction( 3, rc5_3);  
  rc5AttachFunction( 4, rc5_4);  
  rc5AttachFunction( 5, rc5_5);  
  rc5AttachFunction( 6, rc5_6);  
  rc5AttachFunction( 7, rc5_7);  
  rc5AttachFunction( 8, rc5_8);  
  rc5AttachFunction( 9, rc5_9);  
  
} //end botHardwareInit()


void setMotorFastStopRight() {
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, LOW);    

} //end setMotorFastStopRight()


void setMotorFastStopLeft() {
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW); 

} //end setMotorFastStopLeft()


void setMotorDirectionRight(char direction) {
  if (direction == 'f') {    //set right Wheel forward
    digitalWrite(IN3, invertPolarityRight != HIGH); 
    digitalWrite(IN4, invertPolarityRight != LOW);  
  }
  else if (direction == 'r') {  //set right Wheel reverse/backward
    digitalWrite(IN3, invertPolarityRight != LOW); 
    digitalWrite(IN4, invertPolarityRight != HIGH);  
  }

} //end setMotorDirectionRight()


void setMotorDirectionLeft(char direction) {
  if (direction == 'f') {    //set left Wheel forward
    digitalWrite(IN1, invertPolarityLeft != HIGH); 
    digitalWrite(IN2, invertPolarityLeft != LOW);  
  }
  else if (direction == 'r') {  //set left Wheel reverse/backward
    digitalWrite(IN1, invertPolarityLeft != LOW); 
    digitalWrite(IN2, invertPolarityLeft != HIGH);  
  }

} //end setMotorDirectionLeft()


void setMotorPwmRight(int value) {
  if(value > 0) {
    setMotorDirectionRight('f'); 
//    analogWrite(ENB, value);
      /*
      The following code is a special and very fine WorkAround-function by Martin von Löwis:
      Replacing analogWrite() for GPIO ENB, it rewires pin11 (ENB, see above) that originally
      goes with Timer0 to Timer1, and therefore now goes with the same Timer as ENA. 
      (Note: The PWM-frequency of Timer1 is increased from 1khz to 31KHz in "botHardwareInit()"; 
      this avoids motor noise.)
      */
    //pinMode(ENB, OUTPUT);  //Martin's WorkAround
    sbi(TCCR1A, COM1C1);
    OCR1C = value; // set pwm duty
  }
  else if(value < 0) {
    setMotorDirectionRight('r');
//    analogWrite(ENB, -value);
    //pinMode(ENB, OUTPUT);  //Martin's WorkAround
    sbi(TCCR1A, COM1C1);
    OCR1C = -value; // set pwm duty    
  }
  else if(value==0) {
//    analogWrite(ENB, 0);
    //pinMode(ENB, OUTPUT);  //Martin's WorkAround
    sbi(TCCR1A, COM1C1);
    OCR1C = 0; // set pwm duty    
  }

} //end setMotorPwmRight()


void setMotorPwmLeft(int value) {
  if(value > 0) {
    setMotorDirectionLeft('f'); 
    analogWrite(ENA, value);
  }
  else if(value < 0) {
    setMotorDirectionLeft('r');
    analogWrite(ENA, -value);
  }
  else if(value==0) {
    analogWrite(ENA, 0);
  }
} //end setMotorPwmLeft()


void countEncoderPulsesLeft(){
   if(digitalRead(ENCB_L)!=invertEncoderLeft) {
     numberEncoderPulsesLeft++; 
   }
   else {
     numberEncoderPulsesLeft--;
   }

} // end countEncoderPulsesLeft()


void countEncoderPulsesRight(){
   if(digitalRead(ENCB_R)==invertEncoderRight) {
     //left and right motor side-inverted!!!
     numberEncoderPulsesRight++; 
   }
   else {
     numberEncoderPulsesRight--;
   }
} // end countEncoderPulsesLeft()

