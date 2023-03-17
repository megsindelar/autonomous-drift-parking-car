/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  pos = 65;
  myservo.write(pos);
}

void loop() {
  char buffer[16] = {};
  // if we get a command, turn the LED on or off:
  if (Serial.available() > 0) {
    int size = Serial.readBytesUntil('\n', buffer, 12);
    // String position = "";
    // for(int i = 1; i < size; i++){
    //   position += buffer[i];
    // }
    // if (buffer[0] == 'Y') {
    //   pos = 180;
    //   digitalWrite(LED_BUILTIN, HIGH);
    //   myservo.write(pos);
    // }
    // if (buffer[0] == 'N') {
    //   pos = 65;
    //   digitalWrite(LED_BUILTIN, LOW);
    //   myservo.write(pos);
    // }
    if (buffer[0] == 'Y'){
      String s = "";
      pos = 50;
      for (int i = 1; i < 8; i++){
        if (buffer[i] == 'N'){
          i = 8;
        }
        else{
          s += buffer[i];
          // pos = 180;
        }
      }
      // pos = 180;
      pos = s.toInt();
      myservo.write(pos);
  }
  }
}