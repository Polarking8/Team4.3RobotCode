// Library and Object Setup
#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); // RX, TX



// Variable Intialization
unsigned long time = 0;
unsigned long time_old = 0;
unsigned long t_ms = 0;
float t = 0;
float t_old = 0;
int servoAngle;
int LEDval =0;
int potInput = 0;

void setup()  {
  // Open serial communications with computer and wait for port to open:
  Serial.begin(57600);  // make sure to also select this baud rate in your Serial Monitor window

  // Print a message to the computer through the USB
  Serial.println("Hello Computer!");

  // Open serial communications with the other Arduino board
  mySerial.begin(115200);  // this needs to match the Serial1 baud rate in MegaReceiving
  // for wireless comms, it also needs to match the Xbee firmware setting of 115200

  // Send a message to the other Arduino board
  mySerial.print("Hello other Arduino!");

}


void loop(){
  t_ms = millis();
  t = float(t_ms) / 1000.0;

  //Blink an LED without delay func
  if (t-t_old>1){
    if(LEDval==0){
      LEDval = 1;
    } else{
      LEDval = 0;
    }
    t_old = t;
  }
  potInput = analogRead(A0);
  servoAngle = int(map(potInput, 0 ,1023,0,179));
  //servoAngle = int(125*sin(t))+127;
  mySerial.write(255);
  mySerial.write(LEDval);
  mySerial.write(servoAngle);
  delay(20);
  // if (Serial.available()) {
  //   mySerial.println(Serial.readStringUntil('\n'));
  // }

  // if (mySerial.available()) {
  //   Serial.println(mySerial.readStringUntil('\n'));
  // }
}
