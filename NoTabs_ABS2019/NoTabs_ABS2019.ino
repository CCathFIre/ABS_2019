/*
 * Notre Dame Rocketry Team
 * Air-Braking System Flight Code
 * Version 1.0
 * 
 * Authors: John Fox, John Hoeksema
 * Last Update: 2/27/19
 */

// Libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "SparkFunMPL3115A2.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <SPI.h>              // possibly not necessary
#include <MatrixMath.h>       // possibly not necessary

// State machine constants for code readability
#define DELAY_TIME 10         // milliseconds
#define ARMED 0
#define LAUNCHED 1
#define BURNOUT 2
#define APOGEE 3
#define LANDED 4

// Initialization
Adafruit_BNO055 bno = Adafruit_BNO055();
MPL3115A2 MPLPressure;
File dataFile;
char filename[9] = "data.txt";

/* 
 *  !!!   !!!   !!! 
 *  UPDATE CONSTANTS BEFORE FLIGHT
 *  !!!   !!!   !!! 
 */
const float Cd = 0.25;
const float mRocket = 10;
const int potPin = A6;
/* 
 *  !!!   !!!   !!! 
 *  UPDATE CONSTANTS BEFORE FLIGHT
 *  !!!   !!!   !!! 
 */

// Constants
const float pAir = 1.225;
const float aRocket = 0.1;
const int chipSelect = SDCARD_SS_PIN;
const float accelLiftoffThreshold = 50;     // m/s^2
const float baroLiftoffThreshold = 10;      // m
const float accelBurnoutThreshold = -5;     // m/s^2

// Control variables
float lastT, dT;
float k;
int flightstate = ARMED;
float launchA;
int rotation = 0;
float maxA = -100;

// Flags
bool SDINIT = false;
bool BNOINIT = false;
bool MPLINIT = false;
bool POTENTINIT = false;

// Kalman filter matrices, continually updated
mtx_type x[3][1] = {{0}, {0}, {0}};                                   // State (position, velocity, accel), varies with time
mtx_type P[3][3] = {{0.005, 0, 0}, {0, 0.0122, 0}, {0, 0, 0.0176}};   // Covariance, varies with time
mtx_type R[3][3] = {{0.1, 0, 0}, {0, 0.1, 0}, {0, 0, 0.4}};           // Distrust of sensors
mtx_type Theta[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};             // physics transformation matrix
mtx_type I3[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};                // Identity (for maths)
mtx_type Q[3][3] = {{0, 0, 0}, {0, 0.001, 0}, {0, 0, 0.001}};         // Distrust of predictions

void setup() {
  
  Wire.begin();        // Join i2c bus

  // Sensor Flags
  if (!SD.begin(chipSelect)) {        // SD Initialization
    return;
  } else {
    SDINIT = true;
  }
  if(bno.begin()){                    // BNO
    BNOINIT = true;
  }
  
  MPLPressure.begin();                // MPL
  
  // Sensor setup
  bno.setExtCrystalUse(true);
  MPLPressure.setModeAltimeter();     // Measure altitude above sea level in meters (MPL)
  MPLPressure.setOversampleRate(7);   // Set Oversample to the recommended 128
  MPLPressure.enableEventFlags();     // Enable all three pressure and temp event flags 
  
  k = -1*Cd*pAir*aRocket / (2*mRocket);
  x[0][0] = MPLPressure.readAltitude(); //Set initial altitude based on sensor reading

  // Initialize MPL
  if(x[0][0] >= 0 && x[0][0] <= 2500)
  {
    MPLINIT = true;
  }

  rotation = analogRead(potPin);
  if(rotation >= 1400 && rotation <= 3000)
  {
    POTENTINIT = true;
  }
  
  lastT = millis();

  launchA = x[0][0];

  delay(DELAY_TIME); // Delay time for sensor sampling rate

  Print_Header();

  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT); 
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT); 
  pinMode(6, OUTPUT);
  pinMode(8, OUTPUT); 
  digitalWrite(0, SDINIT);   
  digitalWrite(1, BNOINIT);     
  digitalWrite(2, MPLINIT);     
  digitalWrite(3, POTENTINIT);    
}

void loop() {

  /*
   * BEGIN DATA COLLECTION
   */
  
  // Temperature variables
  int8_t bno_temp = bno.getTemp();
  float mpl_temp = MPLPressure.readTemp();

  rotation = analogRead(potPin);

  // vector creation
  // acceleration units in m/s^2
  // gyroscope units in rad/s
  imu::Vector<3> bno_accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> bno_linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> bno_gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  double accel_x = bno_linearAccel.x();
  double accel_y = bno_linearAccel.y();
  double accel_z = bno_linearAccel.z();

  double bno_gyro_x = bno_gyro.x();
  double bno_gyro_y = bno_gyro.y();
  double bno_gyro_z = bno_gyro.z();

  // Altitude in m
  // Pressure in Pa
  float mpl_alt = MPLPressure.readAltitude();
  float mpl_pres = MPLPressure.readPressure();

  if (maxA < x[0][0]) {
    maxA = x[0][0];
  }

  //Kalman(mpl_alt, accel_z);


  /* 
   *  BEGIN DATA WRITING
   */

  dataFile = SD.open(filename, FILE_WRITE);

  // if the file is available, write to it:
  if (true) {

    dataFile.print(flightstate); dataFile.print(","); dataFile.flush();
    dataFile.print(millis()); dataFile.print(","); dataFile.flush();
    dataFile.print(bno_temp); dataFile.print(","); dataFile.flush();
    dataFile.print(mpl_temp, 8); dataFile.print(","); dataFile.flush();
    dataFile.print(accel_x, 8); dataFile.print(","); dataFile.flush();
    dataFile.print(accel_y, 8); dataFile.print(","); dataFile.flush();
    dataFile.print(accel_z, 8); dataFile.print(","); dataFile.flush();
    dataFile.print(bno_gyro_x, 8); dataFile.print(","); dataFile.flush();
    dataFile.print(bno_gyro_y, 8); dataFile.print(","); dataFile.flush();
    dataFile.print(bno_gyro_z, 8); dataFile.print(","); dataFile.flush();
    dataFile.print(mpl_alt, 8); dataFile.print(","); dataFile.flush();
    dataFile.print(mpl_pres, 8); dataFile.print(","); dataFile.flush();
    dataFile.print(rotation, 8); dataFile.print("\n"); dataFile.flush();
////    state matrix begins
//    dataFile.print(x[0][0], 8); dataFile.print(","); dataFile.flush();
//    dataFile.print(x[1][0], 8); dataFile.print(","); dataFile.flush();
//    dataFile.print(x[2][0], 8); dataFile.print(","); dataFile.flush();
//    covariance begins
//    dataFile.print(P[0][0], 8); dataFile.print(","); dataFile.flush();
//    dataFile.print(P[0][1], 8); dataFile.print(","); dataFile.flush();
//    dataFile.print(P[0][2], 8); dataFile.print(","); dataFile.flush();
//    dataFile.print(P[1][0], 8); dataFile.print(","); dataFile.flush();
//    dataFile.print(P[1][1], 8); dataFile.print(","); dataFile.flush();
//    dataFile.print(P[1][2], 8); dataFile.print(","); dataFile.flush();
//    dataFile.print(P[2][0], 8); dataFile.print(","); dataFile.flush();
//    dataFile.print(P[2][1], 8); dataFile.print(","); dataFile.flush();
//    dataFile.print(P[2][2], 8); dataFile.print("\n"); dataFile.flush();
    
    dataFile.close();
  } else {
    //LEDWRITING = false;
  }

  switch(flightstate){
    case ARMED:
      if ( x[2][0] > accelLiftoffThreshold || ( x[0][0] - launchA) > baroLiftoffThreshold){
        flightstate = LAUNCHED;
      }
    break;
    case LAUNCHED:
      if ( x[2][0] < accelBurnoutThreshold){
        flightstate = BURNOUT;
      }
    break;
    case BURNOUT:
      if ( maxA > x[0][0] ){
        flightstate = APOGEE;
      }
    break;
    case APOGEE:
      if ( x[0][0] < launchA + 10 && x[1][0] < 1 ){
        flightstate = LANDED;
      }
    break;
    case LANDED:
    break;
  }

  delay(DELAY_TIME);
}

void Print_Header() {

  dataFile = SD.open(filename, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    
    dataFile.print("Flight State,"); dataFile.flush();
    dataFile.print("Time ms,"); dataFile.flush();
    dataFile.print("BNO Temperature C,"); dataFile.flush();
    dataFile.print("MPL Temperature C,"); dataFile.flush();
    dataFile.print("X Acceleration m/s^2,"); dataFile.flush();
    dataFile.print("Y Acceleration m/s^2,"); dataFile.flush();
    dataFile.print("Z Acceleration m/s^2,"); dataFile.flush();
    dataFile.print("BNO Gyro X rad/s,"); dataFile.flush();
    dataFile.print("BNO Gyro Y rad/s,"); dataFile.flush();
    dataFile.print("BNO Gyro Z rad/s,"); dataFile.flush();
    dataFile.print("Altitude m,"); dataFile.flush();
    dataFile.print("Pressure Pa,"); dataFile.flush();
    dataFile.print("Potentiometer\n"); dataFile.flush();
    

    
    dataFile.close();
  } else {
    // if the file didn't open, print an error:
    //Serial.println("this boi don;t open");
  }

}

void Kalman(float altitude,float zAccel) {
  dT = (millis() - lastT)/1000;
  float kTabs = 0; // UPDATE THIS WITH ACTUAL TAB DRAG

  // Start Kalman filter code
  mtx_type z[3][1] = {{altitude}, {x[1][0]}, {zAccel}};
  //Matrix.Print((mtx_type *)z, 3, 1, "Current readings:");
  mtx_type K[3][3];
  mtx_type tempM[3][3];
  mtx_type temp_2_M[3][3];
  mtx_type tempV[3][1];

  
  // Calculate Kalman gain
  Matrix.Add((mtx_type *)P, (mtx_type *)R, 3, 3, (mtx_type *)tempM);
  Matrix.Invert((mtx_type *)tempM, 3);
  Matrix.Multiply((mtx_type *)P, (mtx_type *)tempM, 3, 3, 3, (mtx_type *)K);

  // Update estimate
  Matrix.Subtract((mtx_type *)z, (mtx_type *)x, 3, 1, (mtx_type *)tempV);
  Matrix.Multiply((mtx_type *)K, (mtx_type *)tempV, 3, 3, 1, (mtx_type *)z);
  Matrix.Add((mtx_type *)x, (mtx_type *)z, 3, 1, (mtx_type *)x);

  // Update covariance
  Matrix.Copy((mtx_type *)P, 3, 3, (mtx_type *)temp_2_M);
  Matrix.Subtract((mtx_type *)I3, (mtx_type *)K, 3, 3, (mtx_type *)tempM);
  Matrix.Multiply((mtx_type *)tempM, (mtx_type *)temp_2_M, 3, 3, 3, (mtx_type *)P);

  
  // Project into next time step
  mtx_type tempTheta[3][3];
  Theta[0][1] = dT;
  Theta[0][2] = 0.5*dT*dT;
  Theta[1][2] = dT;
  Theta[2][1] = k + kTabs;
  Matrix.Copy((mtx_type *)x, 3, 1, (mtx_type *)tempV);
  Matrix.Multiply((mtx_type *)Theta, (mtx_type *)tempV, 3, 3, 1, (mtx_type *)x);
  Matrix.Transpose((mtx_type *)Theta, 3, 3, (mtx_type *)tempTheta);
  Matrix.Multiply((mtx_type *)Theta, (mtx_type *)P, 3, 3, 3, (mtx_type *)tempM);
  Matrix.Multiply((mtx_type *)tempM, (mtx_type *)tempTheta, 3, 3, 3, (mtx_type *)temp_2_M);
  Matrix.Add((mtx_type *)temp_2_M, (mtx_type *)Q, 3, 3, (mtx_type *)P);

  lastT = millis();

  //Matrix.Print((mtx_type *)x, 3, 1, "State of the world:");
  //Serial.println("\n");
  
}
