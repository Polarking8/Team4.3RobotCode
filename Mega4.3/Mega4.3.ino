#include <PWMServo.h>

//Libarries
PWMServo Servo;

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
int ButtonServoPWM0 = 11;
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
char inputChar = 'm';
//End Pin table
void setup(){
  // Open serial communications with computer and wait for port to open:
  Serial.begin(57600); // make sure to also select this baud rate in your Serial Monitor window
  // Print a message to the computer through the USB
  Serial.println("Hello Computer!");
  // Open serial communications with the other Arduino board
  Serial1.begin(115200);  // this needs to match the mySerial baud rate in UnoSending
  // for wireless comms, it also needs to match the Xbee firmware setting of 115200
  // Send a message to the other Arduino board
  Serial1.print("Hello other Arduino!");
  //pinMode(LEDpin,OUTPUT);
  //Servo.attach(ServoPin);
}
void loop(){
  if (Serial.available()) {
    Serial1.println(Serial.readStringUntil('\n'));
  }
  if (Serial1.available()) {
    // Serial.println(Serial1.readStringUntil('\n'));
    //inputString = Serial1.readStringUntil('\n').c_str();
    //inputString = ;
    inputChar = Serial1.read();
    }
    switch (inputChar) {
      case 'f': // forward drive motors
        Serial.println("Forward");
        break;
      case 'b' : //backward drive motors
        Serial.println("Backward");
        break;
      case 'l': // drive left
        Serial.println("Left");
        break;
      case 'r' : //drive right
        Serial.println("Right");
        break;
      case 'u': // conveyer "forward"
        Serial.println("Conveyer Forward");
        break;
      case 'd' : // conveyer "Backward"
        Serial.println("Conveyer Backward");
        break; 
      case 's': // stop drives
        Serial.println("Stopping Drive Motors");
        break;
      case 'x': // stop all
        Serial.println("Stopping everything");
        break; 
      case 'a' : // stop conveyer
        Serial.println("Stopping conveyer");
        break;
      case 'p': // Servo state push
        Serial.println("Servo push button");
        break; 
      case 'z': // Servo state return
        Serial.println("Servo return position");
        break; 
  }
  // if(Serial1.available()>2){
  //   if(Serial1.read()==255){
  //     receivedLEDValue = Serial1.read();
  //     receivedServoAngle = Serial1.read();
  //   }
  // }
  //digitalWrite(LEDpin,receivedLEDValue);
  //Serial.println(receivedServoAngle);
  //Servo.write(receivedServoAngle);
}
