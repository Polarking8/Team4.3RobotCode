#include <Encoder.h>
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
int Color1 = 33;
int Color2 = 34;
int Color3 = 35;
int Color4 = 36;
int Color5 = 37;
int Color6 = 38;
//_________________ Hall Effect 
int HallEffect = A3;
//_________________ Distance Sensor
int DistanceSensor = A4;
//End Pin table
//_________________ Logic Variables + other
char inputChar = 'o'; //o not used
int LeftMotorVal = 0;
int RightMotorVal = 0;
int servoAngle =  0;
int conveyorVal = 0;
int distVal = 0;
int hallVal = 0;
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
double t, t0, print_time=0; // declare some time variables
double Kp=25; //Proportional Gain for Line Following
double base_speed=50; //Nominal speed of robot
//Color Sensor Vals
const int numSamples = 8;
float R[numSamples], G[numSamples], B[numSamples], C[numSamples]; // raw pulse time samples
float RF, GF, BF, CF; // filtered data
// PM 9 Variables 
Encoder encoder1(DriveEncoder1ADual,DriveEncoder1BDual);
Encoder encoder2(DriveEncoder2ADual,DriveEncoder2BDual);
double t9, t_old9, deltaT, print_time, t09 = 0; // time vars
double Kp9 = 15;
double Pi = 3.14159268;
long counts1, counts2;
double GearRatio = 70;
int countsPerRev = 64;
double rw = 4.2;
double D = 26; // Change this, distance between wheels in cm
double theta1, theta1_old = 0, omega1;     //position and velocity of wheel 1
double theta2, theta2_old = 0, omega2;     //position and velocity of wheel 2
double omega2f = 0;
double omega1f = 0, alpha = 0.05;             // filtered velocity and filter weight
double theta1_des = 0, theta2_des = 0;     // desired position of wheels
double theta1_final, theta2_final;         // final desired position of wheels
double omega_des, omega1_des, omega2_des;  // desired velocity of wheels
double V1m, V2m; // Commanded velocity
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
  t0 = micros()/1000000.; // initialize time
  t_old9 = micros() / 1000000.;
  t09 = micros() / 1000000.; // init time for pm9
  //Set up color sensor
  pinMode(Color1,OUTPUT);
  pinMode(Color2,OUTPUT);
  pinMode(Color3,OUTPUT);
  pinMode(Color4,OUTPUT);
  pinMode(Color5,INPUT);
  pinMode(Color6,OUTPUT);
  digitalWrite(Color1, HIGH); // s1 and s0 choose frequency scaling
  digitalWrite(Color2, LOW);
  digitalWrite(Color6, HIGH); //turn on LED

}
void loop(){
  counts1 = encoder1.read();
  counts2 = encoder2.read();
  deltaT = t9-t_old9;
  if (Serial.available()) {
    Serial2.println(Serial.readStringUntil('\n'));
  }
  if (Serial2.available()>2) {
    // Serial.println(Serial1.readStringUntil('\n'));
    //inputString = Serial1.readStringUntil('\n').c_str();
    //inputString = ;
    inputChar = Serial2.read();
    }
  t = micros()/1000000.-t0;
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
      if ((t-print_time)>0.25) { 
        Serial2.println(distVal);
        Serial.println(distVal);
        print_time=t;
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
      if ((t-print_time)>0.25) { 
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
      print_time=t;
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
    case 'n':// Read Hall Effect Sensor Vals
      hallVal = analogRead(HallEffect);
      Serial2.println(hallVal);
      break;
    case 'm': // Read color sensor vals
      // Select RED Filter
      digitalWrite(s2, LOW);
      digitalWrite(s3, LOW);
      delay(10);
      for (int i = 0; i < numSamples; i++){
        R[i] = readPulse();
      }

      // Select BLUE Filter
      digitalWrite(s2, LOW);
      digitalWrite(s3, HIGH);
      delay(10);
      for (int i = 0; i < numSamples; i++){
        B[i] = readPulse();
      }

      // Select GREEN Filter
      digitalWrite(s2, HIGH);
      digitalWrite(s3, HIGH);
      delay(10);
      for (int i = 0; i < numSamples; i++){
        G[i] = readPulse();
      }

      // Select CLEAR Filter
      digitalWrite(s2, HIGH);
      digitalWrite(s3, LOW);
      delay(10);
      for (int i = 0; i < numSamples; i++){
        C[i] = readPulse();
      }
      RF = movingAverage(R);
      GF = movingAverage(G);
      BF = movingAverage(B);
      CF = movingAverage(C);
      //Print Vals
      Serial2.print(RF, 2);
      Serial2.print(",\t");
      Serial2.print(GF, 2);
      Serial2.print(",\t");
      Serial2.print(BF, 2);
      Serial2.print(",\t");
      Serial2.println(CF, 2);
      break;
    case 'q': // PM9 forward trajectory
      //TODO: ADD CODE FOR INPUT 

      theta1_final = 50/rw;
      theta2_final = -50/rw;  //move 50 cm
      omega1_des = theta1_final/10; // 10 is time var


      break;
    case 't': // PM9 Turn in place 

      theta1_final = D/2*Pi/rw;
      theta2_final = -D/2*Pi/rw;
      omega1_des = D/2*Pi/rw/5; //5 is our time var
      omega2_des = -D/2*Pi/rw/5;


      break;
    case 'e': // PM9 Drive In Circle
      theta1_final = (50+D)*Pi/2/rw;
      theta2_final = (50)*Pi/2/rw;
      omega1_des = theta1_final/5; // 5 is time var
      omega2_des = theta2_final/5;


      break:
    default:
      Serial.println("Doing Nothing");
      LeftMotorVal = 0;
      RightMotorVal = 0;
      conveyorVal = 0;
      servoAngle = 0;
      break;
      //Turn everything off
  }
  //PM9 stuff
  theta1 = counts1*2*Pi/GearRatio/countsPerRev;
  omega1 = (theta1-theta1_old)/(deltaT);
  omega1f = omega1*alpha + omega1f*(1-alpha);

  theta2 = -1*(counts2*2*Pi/GearRatio/countsPerRev);
  omega2 = (theta2-theta2_old)/(deltaT);
  omega2f = omega2*alpha + omega2f*(1-alpha);
  // add your trajectory design here
  if(theta1_des<theta1_final){
    theta1_des = theta1_des + omega1_des*deltaT;
    theta2_des = theta2_des + omega2_des*deltaT;
  }
  // add your control laws here
  V1m = Kp9*(theta1_des-theta1);
  V2m = Kp9*(theta2_des-theta2);
  

  // Uncomment these four lines in section 4.4
  V1m = constrain(V1m,-10,10);
  V2m = constrain(V2m,-10,10);
  LeftMotorVal = 400*V1m/10;
  RightMotorVal = 400*V2m/10;
  //Set motors to numbers set during switch case 
  Servo.write(servoAngle);
  md.setM2Speed(LeftMotorVal);
  stopIfFault();
  md.setM1Speed(RightMotorVal);
  stopIfFault();
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

float readPulse(){
  return pulseIn(readPin, LOW)+pulseIn(readPin, HIGH);
}

float movingAverage(float * arr) {
  float sum = 0;
  for (int i = 0; i < numSamples; i++){
    sum += arr[i]/numSamples;
  }
  return sum;
}