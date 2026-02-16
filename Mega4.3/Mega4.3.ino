#include <L298N.h>
//Backup plan
#include <PWMServo.h>
#include <DualTB9051FTGMotorShieldMod3230.h>
//Unsure if this one works
#include "L298NMotorDriverMega.h"
//Libarries
PWMServo Servo;
DualTB9051FTGMotorShield md;
// Variable Intialization
unsigned long time = 0;
unsigned long time_old = 0;
// int receivedLEDValue = 0;
// int receivedServoAngle  = 0;
//Pin table
//________________ Serial comms
int USBRXCable = 0;
int USBTXCable = 1;
//________________ Dual Motor Shield
int M1ENDual = 2;
int DriveEncoder1ADual = 3;
int DriveEncoder1BDual = 18;
int M1DIAGDual = 6; 
int M1DIRDual = 7;
int M1PWMDual = 9;
int M1OCMDual = A0; 
int M2ENDual = 4;
int M2DIRDual = 8;
int M2PWMDual = 10; 
int M2DIAGDual = 12;
int DriveEncoder2ADual = 19;
int DriveEncoder2BDual = 20;
int M2OCMDual = A1;
//_________________ Xbee Shield
int XBeeTX = 16;
int XBeeRX = 17; 
//_________________ Pusher servo pin
int ButtonServoPWM = 11;
//_________________ Solo Motor Driver Shield
int M1DIAGsolo = 22;
int M1PWMsolo = 44;
int M2PWMsolo = 45;
int M1OCMsolo = A1;
//_________________ Reflectance Array
int Reflect1 = 25;
int Reflect2 = 26;
int Reflect3 = 27;
int Reflect4 = 28;
int Reflect5 = 29;
int Reflect6 = 30;
int Reflect7 = 31;
//_________________ Color Sensor
int Color1 = 32;
int Color2 = 33;
int Color3 = 34;
int Color4 = 35;
int Color5 = 36;
int Color6 = 37;
//_________________ Hall Effect 
int HallEffect = A3;
//_________________ Distance Sensor
int DistanceSensor = A4;
//_________________ Logic Variables + other
char inputChar = 'm'; //m not used
int LeftMotorVal = 0;
int RightMotorVal = 0;
int servoAngle =  0;
int conveyorVal = 0;

//End Pin table
//Stop if motor drivers are faulty (I think)
void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while (1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while (1);
  }
}
L298NMotorDriverMega Conveyormotor(55,M1PWMsolo,M2PWMsolo); // This pin is intentionally not a real pin
L298N Conveyormotor2(55,M1PWMsolo,M2PWMsolo);// This pin is intentionally not a real pin (This code is duplicate to make sure things work)
void setup(){
  // Open serial communications with computer and wait for port to open:
  Serial.begin(57600); // make sure to also select this baud rate in your Serial Monitor window
  // Print a message to the computer through the USB
  Serial.println("Hello Computer!");
  // Open serial communications with the other Arduino board
  Serial2.begin(115200);  // this needs to match the mySerial baud rate in UnoSending
  // for wireless comms, it also needs to match the Xbee firmware setting of 115200
  // Send a message to the other Arduino board
  md.init();
  md.enableDrivers();
  Serial2.print("Hello other Arduino!");
  //pinMode(LEDpin,OUTPUT);
  Servo.attach(ButtonServoPWM);
  Conveyormotor.setSpeed(70);
}
void loop(){
  
  if (Serial.available()) {
    Serial2.println(Serial.readStringUntil('\n'));
  }
  if (Serial2.available()>2) {
    // Serial.println(Serial1.readStringUntil('\n'));
    //inputString = Serial1.readStringUntil('\n').c_str();
    //inputString = ;
    inputChar = Serial2.read();
    }
  switch (inputChar) {
    case 'f': // forward drive motors
      Serial.println("Forward");
      LeftMotorVal = 120;
      RightMotorVal = 120;
      break;
    case 'b' : //backward drive motors
      Serial.println("Backward");
      LeftMotorVal = -120;
      RightMotorVal = -120;
      break;
    case 'l': // drive left
      Serial.println("Left");
      LeftMotorVal = -120;
      RightMotorVal = 120;
      break;
    case 'r' : //drive right
      Serial.println("Right");
      LeftMotorVal = 120;
      RightMotorVal = -120;
      break;
    case 'u': // conveyer "forward"
      Serial.println("Conveyer Forward");
      e = 120;
      break;
    case 'd' : // conveyer "Backward"
      Serial.println("Conveyer Backward");
      e = -120;
      break; 
    case 's': // stop drives
      Serial.println("Stopping Drive Motors");
      LeftMotorVal = 0;
      RightMotorVal = 0;
      break;
    case 'x': // stop all
      Serial.println("Stopping everything");
      LeftMotorVal = 0;
      RightMotorVal = 0;
      e = 0;
      servoAngle = 0;
      break; 
    case 'a' : // stop conveyer
      Serial.println("Stopping conveyer");
      e = 0;
      break;
    case 'p': // Servo state push
      Serial.println("Servo push button");
      //servoAngle = ;//Fill in
      break; 
    case 'z': // Servo state return
      Serial.println("Servo return position");
      //servoAngle = ;//Fill in
      break; 
    default:
      Serial.println("Doing Nothing");
      LeftMotorVal = 0;
      RightMotorVal = 0;
      e = 0;
      servoAngle = 0;
      break;
      //Turn everything off
  }
  //Set motors to numbers set during switch case 
  Servo.write(servoAngle);
  md.setM1Speed(Motor1Val);
  stopIfFault();
  md.setM1Speed(Motor2Val);
  stopIfFault();
  if(conveyorVal>0){
    Conveyormotor2.setSpeed(conveyorVal);//Use one or the other
    Conveyormotor2.forward();
    //demarcating the difference between libs
    Conveyormotor.setM1Speed(conveyorVal);
    Conveyormotor.setM2Speed(conveyorVal);
  } else if(conveyorVal <0){
    Conveyormotor2.setSpeed(|conveyorVal|);//Use one or the other
    Conveyormotor2.backward();
    //demarcating the difference between libs
    Conveyormotor.setM1Speed(|conveyorVal|);
    Conveyormotor.setM2Speed(|conveyorVal|);
  }else if(conveyorVal == 0){
    Conveyormotor2.setSpeed(0)//Use one or the other
    Conveyormotor2.stop();
    //demarcating the difference between libs
    Conveyormotor.setM1Speed(0);
    Conveyormotor.setM2Speed(0);
  }
  //Same for conveyor motor
  
}
  //Potentially recommended pseudocode
  /*If Serial.available() Delay(20)
    If Serial.available() == numOfDesiredInputChars + 1
      xbeeSerial.write(Serial.read())
      Do this numOfDesiredInputChars times
      Serial.read()
    Else
      Print(“Wrong number of inputs, please input your command again”)
      While Serial.available() Serial.read()*/