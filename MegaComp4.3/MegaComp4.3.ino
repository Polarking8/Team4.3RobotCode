#include <Encoder.h>
#include <QTRSensors.h>
#include <PWMServo.h>
#include <DualTB9051FTGMotorShieldMod3230.h>

//Libarries
PWMServo Servo; // Create servo object
QTRSensors qtr; // create a reflectance sensor object
DualTB9051FTGMotorShieldMod3230 md; // Create motor driver object

// Variable Intialization
//pi for everything
const double pi = 3.14159268;
//Pin table
//________________ Serial comms
int USBRXCable = 0;
int USBTXCable = 1;

//________________ Dual Motor Shield
int MRENDual = 2;
int DriveEncoderRADual = 3; //motor 1 is right
int DriveEncoderRBDual = 18;
int MRDIAGDual = 6; 
int MRDIRDual = 7;
int MRPWMDual = 9;
int MROCMDual = A0; 
int MLENDual = 4;
int MLDIRDual = 8;
int MLPWMDual = 10; 
int MLDIAGDual = 12;
int DriveEncoderLADual = 19;//motor 2 is left
int DriveEncoderLBDual = 20;
int MLOCMDual = A1;

//_________________ Xbee Shield
int XBeeTX = 16;
int XBeeRX = 17;

//_________________ Pusher servo pin
int ButtonServoPWM = 11;

//_________________ Solo Motor Driver Shield
int MDIAGsolo = 22;
int MPWM1solo = 44;
int MPWM2solo = 45;
int MOCMsolo = A13;

//_________________ Reflectance Array
int Reflect1 = 23;
int Reflect2 = 25;
int Reflect3 = 27;
int Reflect4 = 29;
int Reflect5 = 31;
int Reflect6 = 33;
int Reflect7 = 35;
int Reflect8 = 37;

//_________________ Color Sensor
int ColorS0 = 24; //not currently wired, will be different if we rewire
int ColorS1 = 26;
int ColorS2 = 28;
int ColorS3 = 30;
int ColorIN = 32;

//_________________ Hall Effect 
int HallEffect = A15;

//_________________ Distance Sensor
int DistanceSensor = A14;

//End Pin table
//_________________ Logic Variables + other
//timers
//ms timer for printing occasionaly
unsigned long timeMS = 0;
unsigned long timeMS_old = 0; //update time old only after performing a print

//micros timer for real time and pid applications
//put microseconds timer here

//serial coms vars
char inputChar = 'x'; //the stop everything state

//servo vars
int servoRetractPos = 0; //set servo out and in positions here.
int servoPushPos = 52;
int servoAngle =  servoRetractPos;

//motor control vars
int leftMotorPower = 0; //beteween +400 and -400
int rightMotorPower = 0;//beteween +400 and -400

int conveyorPower = 0;    //beteween +400 and -400

//encoder vars
Encoder encoderR(DriveEncoderRADual,DriveEncoderRBDual); //right
Encoder encoderL(DriveEncoderLADual,DriveEncoderLBDual); //left
double distanceMoved = 0; //distance moved in the last cycle
double mRPos = 0; //encoder rotation in linear cm
double mLPos = 0;
double mRPosLast = 0; //encoder rotation from the last odometry update
double mLPosLast = 0;

//navigation vars
double wheelSpacing = 25.54; //wheel spacing
//starting position of robot
double x = 0; //cm //all positions relative to center between wheels.
double y = 0; //cm
double theta = 0; //deg
double distanceMoved = 0;
double gearRatio = 70;
double countsPerRev = 64;

//sensor vars
//distance sensor
int distVal = 0;

//hall sensor
int hallVal = 0;

// Reflectance Sensor Vars
const uint8_t lineSensorCount = 8;  // # of sensors in reflectance array
uint16_t lineSensorValues[lineSensorCount];  //reflectance sensor readings
uint16_t lineSensorBias[lineSensorCount] = {140,140,140,140,140,92,92,140}; //calibration data goes here
double lineSensorPositions[lineSensorCount] = {0.0, 0.8, 1.6, 2.4, 3.2, 4.0, 4.8, 5.6}; //in cm relative to center of sensor
uint16_t lineSensorValuesUnbiased[lineSensorCount];
double lineAi = 0; //total sensor readings
double lineAid = 0; //sensor readings weighted for distance
double linePosition = 0; //where the line is relative to the center of the sensor in cm

//Color Sensor Vars
const int colorNumSamples = 8;
float colorR[colorNumSamples], colorG[colorNumSamples], colorB[colorNumSamples], colorC[colorNumSamples]; // raw pulse time samples
float colorRF, colorGF, colorBF, colorCF; // filtered data
float colorRN, colorGN, colorBN; //normalized data
char color = 'e'; //last read color (can be r, y, b, or e for error)


//Stop if motor drivers are faulty
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

void setup(){
  // Open serial communications with computer and wait for port to open:
  Serial.begin(57600); // make sure to also select this baud rate in your Serial Monitor window
  // Print a message to the computer through the USB
  Serial.println("Hello Computer!");
  // Open serial communications with the other Arduino board
  Serial2.begin(115200);  // this needs to match the mySerial baud rate in UnoSending
  // for wireless comms, it also needs to match the Xbee firmware setting of 115200
  // Send a message to the other Arduino board

  //initialize timers
  timeMS = millis();
  timeMS_old = timeMS;

  //
  md.init();
  md.enableDrivers();
  Serial2.print("Hello Uno Arduino!");
  Servo.attach(ButtonServoPWM);
  //This pinmode makes the conveyer work, we are manually going to analogWrite()
  pinMode(MPWM1solo,OUTPUT);
  pinMode(MPWM2solo,OUTPUT);
  //Init reflectance sensor
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){23,25,27,29,31,33,35,37},lineSensorCount);//cannot use variables from top make sure they match reflect1-8
  
  //Set up color sensor
  pinMode(ColorS0,OUTPUT);
  pinMode(ColorS1,OUTPUT);
  pinMode(ColorS2,OUTPUT);
  pinMode(ColorS3,OUTPUT);
  pinMode(ColorIN,INPUT);
  digitalWrite(ColorS0, HIGH); // s1 and s0 choose frequency scaling
  digitalWrite(ColorS1, LOW);
}


void loop(){
  //update timers
  timeMS = millis();

  //update odometry this does all the encoder reading internaly
  OdoUpdate();
  
  //check serial monitor
  if (Serial.available()) {
    Serial2.println(Serial.readStringUntil('\n'));
  }
  if (Serial2.available()>2) {
    // Serial.println(Serial1.readStringUntil('\n'));
    //inputString = Serial1.readStringUntil('\n').c_str();
    //inputString = ;
    inputChar = Serial2.read();
  }
  
  //main switch to decide what operating mode
  switch (inputChar) {
    //main comp code
    case 's':
      Serial.println("Running main comp code");
      //all comp logic flow lives here
      break;

    //debuging modes
    //legend: keep this updated please
    //x = stop all

    //actuator testing
    //f = drive forwards
    //b = drive backwards
    //l = turn left
    //r = turn right
    //u = conveyor forwards
    //d = conveyor backwards
    //p = servo push
    //z = servo retract

    //sensor testing
    //i = distnace sensor
    //n = hall sensor
    //a = line sensor
    //m = color sensor
    //o = odometry
    //ect ect
    case 'x': // stop all
      Serial.println("Stopping everything");
      leftMotorPower = 0;
      rightMotorPower = 0;
      conveyorPower = 0;
      servoAngle = servoRetractPos;
      break; 

    //cases for testing actuators
    case 'f': // dumb forward drive motors
      Serial.println("Forward");
      leftMotorPower = 200;
      rightMotorPower = 200;
      break;
    case 'b' : // dumb back backward drive motors
      Serial.println("Backward");
      leftMotorPower = -200;
      rightMotorPower = -200;
      break;
    case 'l': // dumb turn left
      Serial.println("Left");
      leftMotorPower = -200;
      rightMotorPower = 200;
      break;
    case 'r' : // dumb turn right
      Serial.println("Right");
      leftMotorPower = 200;
      rightMotorPower = -200;
      break;
    case 'u': // conveyer "forward"
      Serial.println("Conveyer Forward");
      conveyorPower = 400;
      break;
    case 'd' : // conveyer "Backward"
      Serial.println("Conveyer Backward");
      conveyorPower = -400;
      break;
    case 'p': // Servo state push
      Serial.println("Servo push button");
      servoAngle = servoPushPos;
      break; 
    case 'z': // Servo state return
      Serial.println("Servo return position");
      servoAngle = servoRetractPos;
      break; 

    //cases for testing sensors
    case 'i': // Read distance sensor val 
      readDistanceSensor(); //function saves to global distVal
      if ((timeMS-timeMS_old)>50) { 
        Serial.println("reading distance sensor");
        Serial.println(distVal);
        //Serial.println();
        timeMS_old = timeMS;
      }
      break;

    case 'n': //read and print hall effect also print out thresholding results
      readHallSensor(); //function saves to global hallVal

      if ((timeMS-timeMS_old)>250) { 
        Serial.println("reading hall effect");
        Serial.println(hallVal);
        if (checkSilverfish()){
          Serial.println("silverfish detected");
        } else{
          Serial.println("no silverfish detected");
        }
        Serial.println();
        timeMS_old = timeMS;
      }
      break;
    
    case 'a' ://read and print line sensor vals
      readReflectanceSensor(); //function reads sensor does math and sets global variables;
      //print diagnostic data
      if ((timeMS-timeMS_old)>250) { 
        Serial.println("reading line sensor");
        for (uint8_t i=0; i < lineSensorCount; i++){ //print raw
          Serial.print(lineSensorValues[i]);
          Serial.print('\t');
        }
        Serial.println();
        for (uint8_t i=0; i < lineSensorCount; i++){ //print unbiased
          Serial.print(lineSensorValuesUnbiased[i]);
          Serial.print('\t');
        }
        Serial.println();
        Serial.print(lineAi);
        Serial.print('\t');
        Serial.print(lineAid);
        Serial.print('\t');
        Serial.print(linePosition);
        Serial.println();
        Serial.println();
        timeMS_old = timeMS;
      }
      break;

    case 'm': // Read color sensor and print
      //only run every 1/4sec bevause it is blocking
      if ((timeMS-timeMS_old)>250) {
        color = readColorSensor();

        //print results
        Serial.println("reading color sensor");
        Serial.print(colorRN, 4);
        Serial.print(",\t");
        Serial.print(colorGN, 4);
        Serial.print(",\t");
        Serial.print(colorBN, 4);
        Serial.print(",\t");
        Serial.println(colorCF, 4);
        
        Serial.print("detected a ");
        Serial.print(color);
        Serial.println(" led");

        timeMS_old = timeMS;
      }
      break;

    case 'o': //case to readout odometry
    //note odometry updates regardless of case
      if ((timeMS-timeMS_old)>100) { 
        Serial.println("reading odometry results");
        Serial.print("x = ");
        Serial.print(x);
        Serial.print('\t');

        Serial.print("y = ");
        Serial.print(y);
        Serial.print('\t');

        Serial.print("theta = ");
        Serial.println(theta);
        Serial.println();
        timeMS_old = timeMS;
      }
      break;

    default:
      Serial.println("Doing Nothing");
      leftMotorPower = 0;
      rightMotorPower = 0;
      conveyorPower = 0;
      servoAngle = 0;
      break;
      //Turn everything off //same as case x
  }


  //Set motors to numbers set during switch case 
  Servo.write(servoAngle);
  md.setM1Speed(rightMotorPower); //motor 1 = right motor
  stopIfFault();
  md.setM2Speed(leftMotorPower); //motor 2 = left motor
  stopIfFault();
  if(conveyorPower > 0){
    analogWrite(MPWM1solo,map(conveyorPower,0,400,0,255));//M1 is forward, M2 is backward
    analogWrite(MPWM2solo,0);
  } else if(conveyorPower < 0){
    analogWrite(MPWM2solo,map(abs(conveyorPower),0,400,0,255));//M1 is forward, M2 is backward
    analogWrite(MPWM1solo,0);
  }else if(conveyorPower == 0){
    analogWrite(MPWM2solo,0);
    analogWrite(MPWM1solo,0);
    //Set both to 0
  }
}