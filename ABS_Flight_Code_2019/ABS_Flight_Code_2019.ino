/*
 * Notre Dame Rocketry Team
 * Air-Braking System Flight Code
 * Version 1.0
 * 
 * Authors: John Fox, John Hoeksema
 * Last Update: 2/27/19
 */

#include <Wire.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include "SparkFunMPL3115A2.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>                // possibly not necessary
#include <MatrixMath.h>         // possibly not necessary

// state machine constants for code readability
#define WAITING -1
#define ARMED 0
#define LAUNCHED 1
#define BURNOUT 2
#define APOGEE 3
#define LANDED 4

// update constants before flight

// constants

// control variables

// flags
 
void setup() {
  

}

void loop() {

  // main control loop, progresses through states incrementally and can't go back
  switch(flightState){        
    case WAITING:
    break;
    case ARMED:
    break;
    case LAUNCHED:
    break;
    case BURNOUT:
    break;
    case APOGEE:
    break;
    case LANDED:
    break;
  }

  // subroutines based on specific flags
  
}
