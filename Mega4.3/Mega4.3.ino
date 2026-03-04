#include <Arduino.h>
#include <QTRSensors.h>
// #include <L298NMotorDriverMega.h>
// #include <L298N.h>
//Backup plan
#include <PWMServo.h>
#include <DualTB9051FTGMotorShieldMod3230.h>
//Unsure if this one works
//#include "L298NMotorDriverMega.h"
//Libarries
PWMServo Servo; // Create servo object
QTRSensors qtr; // create a reflectance sensor object
DualTB9051FTGMotorShieldMod3230 md; // Create motor driver object
// Variable Intialization
unsigned long time = 0;
unsigned long time_old = 0;
unsigned long print_time=0;
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
bool ButtonPushed = false;
//_________________ Solo Motor Driver Shield
int M1DIAGsolo = 22;
int M1PWMsolo = 44;
int M2PWMsolo = 45;
int M1OCMsolo = A2;
//_________________ Reflectance Array
int Reflect1 = 25;
int Reflect2 = 26;
int Reflect3 = 27;
int Reflect4 = 28;
int Reflect5 = 29;
int Reflect6 = 30;
int Reflect7 = 31;
int Reflect8 = 32;
//_________________ Color Sensor
int s0 = 34; // was s1 = 33
int s1 = 35; // was s2 = 34;
int s2 = 36; // was s3 = 35
int s3 = 37; // was s4 = 36
int sOut = 38; // was s5 = 37
//int s6 = 38;
int LEDPin = 52;
//_________________ Hall Effect 
int HallEffect = A3;
//_________________ Distance Sensor
int DistanceSensor = A4;
//End Pin table
//_________________ Logic Variables + other
char inputChar = '1'; //o not used
int LeftMotorVal = 0;
int RightMotorVal = 0;
int servoAngle =  0;
int conveyorVal = 0;
int distVal = 0;
// Reflectance Sensor Variable initialization
const uint8_t SensorCount = 8;  // # of sensors in reflectance array
uint16_t sensor_Values[SensorCount];  //reflectance sensor readings
uint16_t sensor_bias[SensorCount] = {140,140,140,140,140,92,92,140};
double di[SensorCount] = {0, 0.8, 1.6, 2.4, 3.2, 4.0, 4.8, 5.6};
uint16_t Sensor_value_unbiased[SensorCount];
double d = 0;
double dZero = 2.8;
double Ai = 0;
double Aid = 0;
double error= 0;
double Kp=25; //Proportional Gain for Line Following
double base_speed=50; //Nominal speed of robot
//Color Sensor Vals
const int numSamples = 8;
float R[numSamples], G[numSamples], B[numSamples], C[numSamples]; // raw pulse time samples
float RF, GF, BF, CF; // filtered data
float RN, GN, BN; // normalized data
char color = 'e';

//pm8 state machine
int hitsLeft = -2;


//Stop if motor drivers are faulty (I think)
// void stopIfFault()
// {
//   if (md.getM1Fault())
//   {
//     Serial.println("M1 fault");
//     while (1);
//   }
//   if (md.getM2Fault())
//   {
//     Serial.println("M2 fault");
//     while (1);
//   }
// }
// L298NMotorDriverMega Conveyormotor(60,M1PWMsolo,M2PWMsolo,60,60,60);
// L298N Conveyormotor2(55,M1PWMsolo,M2PWMsolo);// This pin is intentionally not a real pin (This code is duplicate to make sure things work)
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
  Serial2.print("Hello Uno Arduino!");
  Servo.attach(ButtonServoPWM);
  //This pinmode makes the conveyer work, we are manually going to analogWrite()
  pinMode(M1PWMsolo,OUTPUT);
  pinMode(M2PWMsolo,OUTPUT);
  //Init reflectance sensor
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){25,26,27,28,29,30,31,32},SensorCount);
  //Set up color sensor
  pinMode(s0,OUTPUT);
  pinMode(s1,OUTPUT);
  pinMode(s2,OUTPUT);
  pinMode(s3,OUTPUT);
  pinMode(sOut,INPUT);
  //pinMode(s6,OUTPUT);
  pinMode(LEDPin, OUTPUT);
  // s1 and s0 choose frequency scaling
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);

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
  time = millis(); //time in seconds
  switch (inputChar) {
    case 'f': // forward drive motors
      Serial.println("Forward");
      LeftMotorVal = 400;
      RightMotorVal = 400;
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
      conveyorVal = 400;
      break;
    case 'd' : // conveyer "Backward"
      Serial.println("Conveyer Backward");
      conveyorVal = -400;
      break; 
    case 's': // Read distance sensor val
      distVal = analogRead(DistanceSensor);
      if ((time-print_time)>250) { 
        Serial2.println(distVal);
        Serial.println(distVal);
        print_time=time;
      }
      break;
    case 'x': // stop all
      Serial.println("Stopping everything");
      LeftMotorVal = 0;
      RightMotorVal = 0;
      conveyorVal = 0;
      servoAngle = 0;
      break; 
    case 'a' ://read reflectance vals
      qtr.read(sensor_Values);
      if ((time-print_time)>250) { 
        for (uint8_t i=0; i < SensorCount; i++){
          Serial.print(sensor_Values[i]);
          Serial.print('\t');
          // Serial.print(Sensor_value_unbiased[i]);
          // Serial.print('\t');
          Serial2.print(sensor_Values[i]);
          Serial2.print('\t');
        }
      Serial2.println(" ");
      Serial.println("");
      print_time=time;
      }
      break;
    case 'k': //Line following
      qtr.read(sensor_Values);
      for (int i = 0; i < SensorCount; i++){
        Sensor_value_unbiased[i] = sensor_Values[i] - sensor_bias[i];
        if(Sensor_value_unbiased[i]>5000){
          Sensor_value_unbiased[i] = 0;
        }
        Serial2.print(Sensor_value_unbiased[i]);
        Serial2.print('\t');
      }
      Ai = 0;
      Aid = 0;
      for(int i = 0; i < SensorCount; i++){
        Aid = Aid + Sensor_value_unbiased[i] * di[i];
        Ai = Ai+ Sensor_value_unbiased[i];
      }
      d = Aid/Ai;
      Serial2.print(Aid);
      Serial2.print('\t');
      Serial2.print(Ai);
      Serial2.print('\t');
      error = dZero-double(d);
      // Serial2.print("  Error: ");
      Serial2.println(d);
      RightMotorVal = base_speed + Kp*error;
      LeftMotorVal = base_speed - Kp*error;
      break;
    case 'w': // stop 4cm from the wall
      distVal = analogRead(DistanceSensor);
      LeftMotorVal = 120;
      RightMotorVal = 120;
      if(distVal>200){ // change this val to be able to stop, depending on the sensor calibration
        LeftMotorVal = 0;
        RightMotorVal = 0;
      }
      break;
    case 'p': // Servo state push
      Serial.println("Servo push button");
      servoAngle = 52;
      break; 
    case 'z': // Servo state return
      Serial.println("Servo return position");
      servoAngle = 0;
      break; 
    case 'o': // Servo oscillate push
      if (time-print_time>200) {
        if (ButtonPushed) {
          Serial.println("Servo return position");
          servoAngle = 0;
          ButtonPushed = false;
        } else {
          Serial.println("Servo push button");
          servoAngle = 52;
          ButtonPushed = true;
        }
        print_time = time;
      }
      break; 
    case 'n':// Read Hall Effect Sensor Vals
      // TODO: CHANGE TO SERIAL2
      if (time-print_time>250) {
        if (checkSilverfish()){
          Serial.println("Silverfish detected");
        } else{
          Serial.println("No silverfish detected");
        }
        print_time = time;
      }
      break;
    case 'm': // Read color sensor vals
      if (time-print_time>250) {
        color = checkColor();
        Serial.print("detected a ");
        Serial.print(color);
        Serial.println(" LED");
        print_time = time;
      }
      break;

    case '8': //pm8
    //controll logic with hitsLeft variable
    //0 means it has finished mining and will wait to get a good hall effect reading
    //-1 means it has finished mining block and will check hall effect
    //-2 means it is ready to mine next block, will check color and set hits left to 10
    //greater than 0 means it is working on mining
      if (hitsLeft == -2) {
        color = checkColor();
        switch (color){
        case 'y':
            Serial.println("Mining Stone");
            break;

          case 'r':
            Serial.println("Mining Iron");
            break;

          case 'b':
            Serial.println("Mining Diamond");
            break;

          default:
            Serial.println("Did not successfuly determine color");
            break;
          }
        hitsLeft = 10;
        ButtonPushed =false;
        }

      if (hitsLeft > 0){
        if (time-print_time>400) {
          if (ButtonPushed) {
            //Serial.println("Servo return position");
            servoAngle = 0;
            ButtonPushed = false;
            hitsLeft = hitsLeft-1;
            print_time = time;
          } else {
            //Serial.println("Servo push button");
            servoAngle = 52;
            ButtonPushed = true;
          }
          print_time = time;
        }
      }
      
      if (hitsLeft == 0){ //wait for block to fall to check hall effect
        Serial.println(time-print_time);
        if (time-print_time>3000) {
          hitsLeft = -1;
          print_time = time;
        }
      }

      if (hitsLeft == -1){
        if(checkSilverfish()){
          Serial.println("Silverfish Detected, hitting 10 more times to kill it");
          hitsLeft = 10;
        } else{
          Serial.println("No Silverfish Detected, mining next block");
          hitsLeft = -2;
        }
      }
      break;

    default:
      Serial.println("Doing Nothing");
      LeftMotorVal = 0;
      RightMotorVal = 0;
      conveyorVal = 0;
      servoAngle = 0;
      hitsLeft = -2;
      break;
      //Turn everything off
    }
  //Set motors to numbers set during switch case 
  Servo.write(servoAngle);
  md.setM2Speed(LeftMotorVal);
  //stopIfFault();
  md.setM1Speed(RightMotorVal);
  //stopIfFault();
  if(conveyorVal>0){
    analogWrite(M1PWMsolo,map(conveyorVal,0,400,0,255));//M1 is forward, M2 is backward
    analogWrite(M2PWMsolo,0);
  } else if(conveyorVal <0){
    analogWrite(M2PWMsolo,map(abs(conveyorVal),0,400,0,255));//M1 is forward, M2 is backward
    analogWrite(M1PWMsolo,0);
  }else if(conveyorVal == 0){
    analogWrite(M2PWMsolo,0);
    analogWrite(M1PWMsolo,0);
    //Set both to 0
  }
}

