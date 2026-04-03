#include <PID_v1.h>
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
unsigned long timeMS_old = 0;
unsigned long timeMSpusher_old = 0; //update time old only after performing a print

//micros timer for real time and pid applications
double time = 0;
double timeTraj = 0; //time in seconds relative to trajectory start
double timeTrajStart = 0;
double timeTrajStepStart = 0; //time from when overall trajectory started to when traj step started
double timeTrajStepFinish = 0; //time trajectory step will finish at, relative to start of trajectory step

double timeOld = 0; //micros timer for loop duration finding
double deltaT = 0; //time the last whole loop took

//state machine variables
int state = 0; // main state machine controll variable
bool isPushed = false;

//serial coms vars
char inputChar = 'x';//'x'; //the stop everything state
bool freshCommand = true; //flag var for restarting state machines

//servo vars
int servoRetractPos = 0; //set servo out and in positions here.
int servoPushPos = 52;
int servoAngle =  servoRetractPos;

//motor control vars
int leftMotorPower = 0; //beteween +400 and -400
int rightMotorPower = 0;//beteween +400 and -400
double leftMotorPowerDouble = 0.0;
double rightMotorPowerDouble = 0.0;
int conveyorPower = 0;    //beteween +400 and -400

//encoder vars
Encoder encoderR(DriveEncoderRADual,DriveEncoderRBDual); //right
Encoder encoderL(DriveEncoderLADual,DriveEncoderLBDual); //left
double distanceMoved = 0; //distance moved in the last cycle
double mRPos = 0; //encoder rotation in linear cm
double mLPos = 0;
double mRPosLast = 0; //encoder rotation from the last odometry update
double mLPosLast = 0;
double mRVel = 0;
double mLVel = 0; // calculated thru Odometry.ino
double alpha = 0.2; // this is for filtering the velocity
double mRVelDes = 0;
double mLVelDes = 0; // desired velocity 
double mLVelDesLimit = 0;
double mRVelDesLimit = 0; // rate limiters

//RAM-SETE variables + PID initialization
double KfVel = 7.5;
double KpVel = 15; // Proportional Gain
double KiVel = 0; // Integral 
double KdVel = 0; // Derivative
PID pidL(&mLVel, &leftMotorPowerDouble, &mLVelDesLimit, KpVel, KiVel, KdVel, DIRECT);
PID pidR(&mRVel, &rightMotorPowerDouble, &mRVelDesLimit, KpVel, KiVel, KdVel, DIRECT);
double attemptAccelL = 0;
double attemptAccelR = 0;
//trajectory gen vars
struct Pose{ //struct to store any x, y, theta coordiante
  double x;
  double y;
  double theta;
};
struct PandV{ //struct to store all position and velocity vars needed to run ramsete
  Pose p;
  double v;
  double w;
};

double maxVel = 25; //cm/s //max vel of center of robot
double maxAccel = 100;//100; //cm/s/s //implement in the velocity controller as a form of smoothing, tune lower to prevent wheel slip.
//will be updated during the trajectory following to the current theoretical (if it was following perfectly) x,y,theta, and velocities
PandV PandVdes;
int trajStep = 0;
bool followingLine = false;
Pose initialP = {
  .x = 4.445, //1.75in (up against back wall)
  .y = 15.24, //6in (centered in channel)
  .theta = 0 //pointing out of chanenl
};

Pose stepStartP = initialP;

double arcRadiusNext = 0; //occasionaly used variable for the arc radius of the upcoming trajectory step
double vNext = 0; //used to remember what the velocity will be for the current step.
//navigation vars
double wheelSpacing = 25.54*1.02065; //wheel spacing
//starting position of robot
Pose actualP = initialP;

//ramsete vars
Pose errorP = { //error in the local frame of the robot (rotation matrix applied)
  .x = 0,
  .y = 0,
  .theta = 0
};
//gains
double ramB =2.0/ (wheelSpacing*wheelSpacing); //proportional term for ramsete controller
double ramD = 0.7; //damping term of ramsete controller

double ramK = 0; //intermediate gain value for ramsete conroller
double ramVdes = 0; //linear and rotational velocities the robot needs to follow (after closing loop)
double ramWdes = 0;

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
bool onLine = false; //true if is over the line and reading is valid
double linePosition = 0; //where the line is relative to the center of the sensor in cm
double linePositionDes = 2.8;//2.8 = center of sensor
double lineError = 0;
double lineKp = 1;

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
  time = micros() / 1000000.0;
  timeTrajStart = time;
  timeTraj = time - timeTrajStart;
  timeOld = time;
  deltaT = 0;

  md.init();
  md.enableDrivers();
  //send start flag
  Serial2.print('<');
  Serial2.print("Hello Uno Arduino!");
  Serial2.print('>');
  Servo.attach(ButtonServoPWM);
  //This pinmode makes the conveyer work, we are manually going to analogWrite()
  pinMode(MPWM1solo,OUTPUT);
  pinMode(MPWM2solo,OUTPUT);
  //Init reflectance sensor
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){23,25,27,29,31,33,35,37},lineSensorCount);//cannot use variables from top make sure they match reflect1-8
  //Set up PIDs
  pidL.SetMode(AUTOMATIC);
  pidR.SetMode(AUTOMATIC);
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
  time = micros() / 1000000.0;
  timeTraj = time - timeTrajStart;
  deltaT = time - timeOld;
  timeOld = time;
  //update odometry this does all the encoder reading internaly
  OdoUpdate();
  
  //check serial monitor
  
  if (Serial2.available()>2) {
    // Serial.println(Serial1.readStringUntil('\n'));
    //inputString = Serial1.readStringUntil('\n').c_str();
    //inputString = ;
    inputChar = Serial2.read();

    //fresh comand reset state machine
    
    freshCommand = true;
  } else{
    freshCommand = false;
  }
  
  //main switch to decide what operating mode
  switch (inputChar) {
    //main comp code
    case 's':
      //Serial.println("Running main comp code");
      //all comp logic flow lives here

      //if flag var is true, reset state machine timers and state
      if (freshCommand){
        state = 0;
        isPushed = false;
        timeMS_old = timeMS;
        timeMSpusher_old = timeMS;
        timeTrajStart = micros() / 1000000.0;
      }

      //update timers
      

      //start trajectory
      switch (state) {
        case 0: //start traj
          trajStep = 0;
          //reset actaul position
          actualP = initialP;
          mRVel = 0;
          mRVelDes = 0;
          mRVelDesLimit = 0;
          mLVel = 0;
          mLVelDes = 0;
          mLVelDesLimit = 0;
          errorP = { //error in the local frame of the robot (rotation matrix applied)
            .x = 0,
            .y = 0,
            .theta = 0
          };

          state = state+1;
          break;

        case 1: //while running traj
          //update trajectory command
          GenTrajectory(); //updates PandVdes to follow the trajectory

          //escape once trajStep reaches the end
//watch out for wrong traj step ending number.
  //yes I know this is defnitely a bad way to do this.
          if (trajStep == 8){
            state = state + 1;
          }
          break;

        default:
          break;
      }
      //do ramsete to calculate target motor velocity
      //sets mLVelDes, and mLVelDes
      if (followingLine){
        //put code to find mLVelDes and mRVelDes based on line position.
        readReflectanceSensor();

        mRVelDes = 10 + lineKp*lineError;
        mLVelDes = 10 - lineKp*lineError;
      } else{
        //calulate velocities with ramsete only if not line following
        Ramsete();
      }
      


      //temp manualy set values
      // if ((timeMS-timeMSpusher_old)<1500) { 
      //   mRVelDes = 40;
      //   mLVelDes = 40;
      // } else if ((timeMS-timeMSpusher_old)<3000){
      //   mRVelDes = -40;
      //   mLVelDes = -40;
      // } else{
      //   timeMSpusher_old = timeMS;
      // }
      
      // //temp set vals with serial      
      // if (Serial.available()>=4) {
      //   String _ = Serial.readStringUntil('\n');
      //   mRVelDes = _.toFloat();
      //   mLVelDes = mRVelDes;
      // }
      //do rate limiting to cap target motor velocity if it changed too much
      attemptAccelL = (mLVelDes-mLVelDesLimit) / deltaT; // compute attempted accelerations to check if we're gonna overtune
      attemptAccelR = (mRVelDes-mRVelDesLimit) / deltaT;
      if (attemptAccelL > maxAccel){
        mLVelDesLimit = mLVelDesLimit + (maxAccel*deltaT);
      }else if (attemptAccelL < -1*maxAccel){
        mLVelDesLimit = mLVelDesLimit - (maxAccel*deltaT);
      }else{
        mLVelDesLimit = mLVelDes;
      }
      if (attemptAccelR > maxAccel){
        mRVelDesLimit = mRVelDesLimit + (maxAccel*deltaT);
      }else if (attemptAccelR < -1*maxAccel){
        mRVelDesLimit = mRVelDesLimit - (maxAccel*deltaT);
      }else{
        mRVelDesLimit = mRVelDes;
      }

      //do velocity pid and set motor power
      pidL.Compute();
      pidR.Compute();
      leftMotorPower = round(leftMotorPowerDouble+KfVel*mLVelDesLimit); //also add feed forward
      rightMotorPower = round(rightMotorPowerDouble+KfVel*mRVelDesLimit);

      if ((timeMS-timeMS_old)>25) { 
        Serial2.print("<");
        Serial2.print(stepStartP.x,2);
        Serial2.print("\t");
        Serial2.print(stepStartP.y,2);
        Serial2.print("\t");
        Serial2.print(stepStartP.theta*180.0/pi,3);
        Serial2.print("\t");
        Serial2.print(PandVdes.p.x,2);
        Serial2.print("\t");
        Serial2.print(PandVdes.p.y,2);
        Serial2.print("\t");
        Serial2.print(PandVdes.p.theta*180.0/pi,3);
        Serial2.print("\t");
        Serial2.print(PandVdes.v,2);
        Serial2.print("\t");
        Serial2.print(PandVdes.w,2);
        Serial2.print("\t");
        Serial2.print(timeTraj-timeTrajStepStart,2);
        Serial2.print("\t");
        Serial2.print(trajStep);
        Serial2.print(">");
        timeMS_old = timeMS;
      }
      
      // if ((timeMS-timeMS_old)>25) { 
      //   //Serial2.print("<");
      //   Serial.print(PandVdes.p.x,2);
      //   Serial.print("\t");
      //   Serial.print(PandVdes.p.y,2);
      //   Serial.print("\t");
      //   Serial.print(PandVdes.p.theta,2);
      //   Serial.print("\t");
      //   Serial.print(PandVdes.v,2);
      //   Serial.print("\t");
      //   Serial.print(PandVdes.w,2);
      //   Serial.print("\t");
      //   Serial.print("\t");
      //   Serial.print(actualP.x,2);
      //   Serial.print("\t");
      //   Serial.print(actualP.y,2);
      //   Serial.print("\t");
      //   Serial.print(actualP.theta,2);
      //   Serial.print("\t");
      //   Serial.print(ramVdes,2);
      //   Serial.print("\t");
      //   Serial.print(ramWdes,2);
      //   Serial.print("\t");
      //   Serial.print("\t");
      //   Serial.print(errorP.x,2);
      //   Serial.print("\t");
      //   Serial.print(errorP.y,2);
      //   Serial.print("\t");
      //   Serial.print(errorP.theta,2);
      //   // Serial.print("\t");
      //   // Serial.print("\t");
      //   // Serial.print(mLVelDesLimit);
      //   // Serial.print("\t");
      //   // Serial.print(mRVelDesLimit);
      //   // Serial.print("\t");
      //   // Serial.print(mLVel,2);
      //   // Serial.print("\t");
      //   // Serial.print(mRVel,2);
      //   Serial.println();//(">");
      //   timeMS_old = timeMS;
      // }


      //will set leftMotorPower and rightMotorPower
      break;

    //debugging modes
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
    //y = PM10 Conveyer + servo pushing

    //sensor testing
    //i = distnace sensor
    //n = hall sensor
    //a = line sensor
    //m = color sensor
    //o = odometry
    //R = reset odometry to initial coordinates
    //etc.
    case 'x': // stop all
      //Serial.println("Stopping everything");
      leftMotorPower = 0;
      rightMotorPower = 0;
      conveyorPower = 0;
      servoAngle = servoRetractPos;
      isPushed = false;
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
      isPushed = true;
      break; 
    case 'z': // Servo state return
      Serial.println("Servo return position");
      servoAngle = servoRetractPos;
      isPushed = false;
      break; 
    case 'y': //PM10 Conveyor + servo
      //if flag var is true, reset state machine timers and state
      if (freshCommand){
        state = 0;
        isPushed = false;
        timeMS_old = timeMS;
        timeMSpusher_old = timeMS;
      }
      switch (state){
        case 0:
          //start dropping conveyor
          conveyorPower = -400;

          //start spaming button
          if (((timeMS-timeMSpusher_old)>140) && (isPushed)) { 
            timeMSpusher_old = timeMS;
            servoAngle = servoRetractPos;
            isPushed = false;
          }
          if (((timeMS-timeMSpusher_old)>125) && (!isPushed)) { 
            timeMSpusher_old = timeMS;
            servoAngle = servoPushPos;
            isPushed = true;
          }

          //wait 2000ms
          if ((timeMS-timeMS_old) > 2000){
            timeMS_old = timeMS;
            state = state + 1;
          }
          break;

        case 1:
          //start moving conveyor
          conveyorPower = 400;

          //start spaming button
          if (((timeMS-timeMSpusher_old)>140) && (isPushed)) { 
            timeMSpusher_old = timeMS;
            servoAngle = servoRetractPos;
            isPushed = false;
          }
          if (((timeMS-timeMSpusher_old)>125) && (!isPushed)) { 
            timeMSpusher_old = timeMS;
            servoAngle = servoPushPos;
            isPushed = true;
          }
          break;
      }
      break;
    //cases for testing sensors
    case 'i': // Read distance sensor val 
      readDistanceSensor(); //function saves to global distVal
      if ((timeMS-timeMS_old)>50) { 
        Serial2.print("<");
        Serial2.print(distVal);
        Serial2.print(">");

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
      if ((timeMS-timeMS_old)>250) { 
        //send start flag
        Serial2.print('<');;
        Serial2.print(actualP.x);
        Serial2.print('\t');

        Serial2.print(actualP.y);
        Serial2.print('\t');

        Serial2.print(actualP.theta* 180.0/pi);
        //send end flag
        Serial2.print('>');

        timeMS_old = timeMS;
      }
      break;
    
    //reset odometry to initial position
    case 'R':
      actualP = initialP;
      break;

    default:
      Serial.println("Doing Nothing");
      leftMotorPower = 0;
      rightMotorPower = 0;
      conveyorPower = 0;
      servoAngle = 0;
      break;
      //Turn everything off, same as case x
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

static double wrapPi(double a)
    {
        //while (a >  pi) a -= 2.0 * pi;
        //while (a < -pi) a += 2.0 * pi;
        return a;
    }
