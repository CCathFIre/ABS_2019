/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 20;    // variable to store the servo position
int rotation = 0;
int extension = 90;
int retraction = 20;
int potPin = A6;
int minRotation = 2000;
int maxRotation = 2000;
float servoJamThreshold = 5;

void setup() {
  myservo.attach(7);  // attaches the servo on pin 9 to the servo object
}

void print_pos(int pos) {
  Serial.println(pos);
}

void loop() {
  //myservo.write(0);
  //while(1);
  
  for (pos = retraction; pos <= extension; pos += 5) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    Serial.print("pos= ");
    Serial.print(pos);
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
    analogReadResolution(12);
    rotation = analogRead(potPin);
    
    if(rotation > maxRotation){
      maxRotation = rotation;
    }
    if(rotation < minRotation){
      minRotation = rotation;
    }
    
    
    Serial.print(",  rotation= "); Serial.println(rotation);
    Serial.print("min rotation= "); Serial.println(minRotation);
    Serial.print("max rotation= "); Serial.println(maxRotation);
    Serial.println("\n");
  }

  delay(1000);
  
  for (pos = extension; pos >= retraction; pos -= 5) { // goes from 180 degrees to 0 degrees
    Serial.print("pos= ");
    Serial.print(pos);
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
    analogReadResolution(12);
    rotation = analogRead(potPin);

    if(rotation > maxRotation){
      maxRotation = rotation;
    }
    if(rotation < minRotation){
      minRotation = rotation;
    }
    
    Serial.print(",  rotation= "); Serial.println(rotation);
    Serial.print("min rotation= "); Serial.println(minRotation);
    Serial.print("max rotation= "); Serial.println(maxRotation);
    Serial.println("\n");
  
  }
  delay(1000);
}

bool Check_Jam(){
  float realTheta = ((float)rotation-381.95)/8.75; //ALWAYS MAKE SURE TO CALIBRATE THIS!!!
  if(fabs(realTheta-pos) > servoJamThreshold)
    return true;
  else
    return false;
}

