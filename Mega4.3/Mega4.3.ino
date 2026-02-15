#include <PWMServo.h>

//Libarries
PWMServo Servo;

// Variable Intialization
unsigned long time = 0;
unsigned long time_old = 0;


int receivedLEDValue = 0;
int receivedServoAngle  = 0;
//Pin table
int LEDpin = 13;
int ServoPin = 11;

//End Pin table
void setup()  
{
  // Open serial communications with computer and wait for port to open:
  Serial.begin(57600); // make sure to also select this baud rate in your Serial Monitor window
  // Print a message to the computer through the USB
  Serial.println("Hello Computer!");
  // Open serial communications with the other Arduino board
  Serial1.begin(115200);  // this needs to match the mySerial baud rate in UnoSending
  // for wireless comms, it also needs to match the Xbee firmware setting of 115200
  // Send a message to the other Arduino board
  Serial1.print("Hello other Arduino!");
  pinMode(LEDpin,OUTPUT);
  Servo.attach(ServoPin);
}

//void loop() // run over and over
//{
//  time = millis();
//
//  if(time-time_old >= 2000) {
//    Serial.print("Hello World - ");
//    Serial.print(float(time)/1000);
//    Serial.println(" sec");
//    time_old = time;
//  }
//
//}

void loop(){
  // if (Serial.available()) {
  //   Serial1.println(Serial.readStringUntil('\n'));
  // }

  // if (Serial1.available()) {
  //   Serial.println(Serial1.readStringUntil('\n'));
  // }
  if(Serial1.available()>2){
    if(Serial1.read()==255){
      receivedLEDValue = Serial1.read();
      receivedServoAngle = Serial1.read();
    }
  }
  digitalWrite(LEDpin,receivedLEDValue);
  Serial.println(receivedServoAngle);

  Servo.write(receivedServoAngle);
}
