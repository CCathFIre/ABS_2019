/* BEFORE FLIGHT CHECKLIST
 *    make sure R/W on SD card
 *    comment out:
 *    - while(!Serial)
 *    - all Serial.print statements
 *    - all Serial.println statements
 *    - all Matrix.print statements
 *    update:
 *    - pAir
 *    - aRocket
 *    - mRocket
 *    - DELAY_TIME (if we find a lower value that the sensors can still handle)
 *    
 * AFTER FLIGHT CHECKLIST
 * - inspect SD card for data
 * - copy data file to computer
 * - update data file name on SD card
 * - fire handguns wildly into the sky in celebratory gesture
 * 
 */

// libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "SparkFunMPL3115A2.h"
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <SPI.h>
#include <MatrixMath.h>

#define DELAY_TIME 10
#define ARMED 0
#define LAUNCHED 1
#define BURNOUT 2
#define APOGEE 3
#define LANDED 4

float lastT, dT;
float k;
const float Cd = 0.25; // UPDATE THESE VALUES!!
const float pAir = 1.225;
const float aRocket = 0.1;
const float mRocket = 10;
// ^^^ UPDATE THESE VALUES!!

// matrices for kalman filter, continually updated
mtx_type x[3][1] = {{0}, {0}, {0}}; // State (position, velocity, accel), varies with time
mtx_type P[3][3] = {{0.005, 0, 0}, {0, 0.0122, 0}, {0, 0, 0.0176}}; // Covariance, varies with time
mtx_type R[3][3] = {{0.1, 0, 0}, {0, 0.1, 0}, {0, 0, 0.4}};   // Distrust of sensors
mtx_type Theta[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // physics transformation matrix
mtx_type I3[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // Identity (for maths)
mtx_type Q[3][3] = {{0, 0, 0}, {0, 0.001, 0}, {0, 0, 0.001}}; // Distrust of predictions

const int chipSelect = SDCARD_SS_PIN;

int flightstate = ARMED;
bool LEDINITIAL = false;
bool LEDWRITING = false;

bool BNOINIT = false;
bool L3GINIT = false;
bool SDINIT = false;

const float accelLiftoffThreshold = 50; //m/s^2  50
const float baroLiftoffThreshold = 10; //m   10
const float accelBurnoutThreshold = -5; //m/s^2 -5
//const float baroApogeeThreshold = 5; //m    5
//const float baroLandedThreshold = 0; //m    5
//const float accelFreefallThreshold = 1; //m/s^2  30

float launchA;
float maxA = -100;

File dataFile;

// BNO instantiation
Adafruit_BNO055 bno = Adafruit_BNO055();

// MPL instantiation
MPL3115A2 MPLPressure;

//L3G instantiation?
/* Assign a unique ID to this sensor at the same time */
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

char filename[9] = "data.txt";

void setup() {

  //Serial.begin(9600);   // printing to screen // TESTING

  //while(!Serial) ;  // TESTING
  
  Wire.begin();        // Join i2c bus

  if (!SD.begin(chipSelect)) {
    //Serial.println("The SD card has not exploded!!!");
    return;
  } else{
    SDINIT = true;
  }

  /* Initialise the sensors */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    // Record failure in SD card
    while(1);
  } else
  {
    BNOINIT = true;
  }
   /* Enable auto-ranging */
  gyro.enableAutoRange(true);
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    //Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
  else
  {
    L3GINIT = true;
  }

  // check if that init returned something // MPLPressure.begin();
  MPLPressure.begin();
  

  if ( BNOINIT && L3GINIT && SDINIT )
  {
    LEDINITIAL = true;
  }
  
  bno.setExtCrystalUse(true);

  MPLPressure.setModeAltimeter(); // Measure altitude above sea level in meters (MPL)
  MPLPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  MPLPressure.enableEventFlags(); // Enable all three pressure and temp event flags 
  
  k = -1*Cd*pAir*aRocket / (2*mRocket);
  
  x[0][0] = MPLPressure.readAltitude(); //Set initial altitude based on sensor reading
  lastT = millis();

  launchA = x[0][0];

  delay(DELAY_TIME); // Delay time for sensor sampling rate

  Print_Header();

  pinMode(6, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(6, LEDINITIAL);
}

void loop() {

  //Serial.println("We are looping");

  // Temperature variables
  int8_t bno_temp = bno.getTemp();
  float mpl_temp = MPLPressure.readTemp();

  // vector creation
  // acceleration units in m/s^2
  // gyroscope units in rad/s
  imu::Vector<3> bno_accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> bno_linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> bno_gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  /* Get a new sensor event */ 
  sensors_event_t L3G_event; 
  gyro.getEvent(&L3G_event);

  double accel_x = bno_linearAccel.x();
  double accel_y = bno_linearAccel.y();
  double accel_z = bno_linearAccel.z();

  double bno_gyro_x = bno_gyro.x();
  double bno_gyro_y = bno_gyro.y();
  double bno_gyro_z = bno_gyro.z();

  double l3g_gyro_x = L3G_event.gyro.x;
  double l3g_gyro_y = L3G_event.gyro.y;
  double l3g_gyro_z = L3G_event.gyro.z;

  // Altitude in m
  // Pressure in Pa
  float mpl_alt = MPLPressure.readAltitude();
  float mpl_pres = MPLPressure.readPressure();

  if (maxA < x[0][0]) {
    maxA = x[0][0];
  }

  Kalman(mpl_alt, accel_z);

  dataFile = SD.open(filename, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    
    LEDWRITING = true;

    //Serial.println("WITNESS ME!!!");
    //Serial.print("accelx");
    //Serial.print(accel_x, 8); Serial.print(","); Serial.flush(); Serial.println(" ");
    //Serial.print("accely");
    //Serial.print(accel_y, 8); Serial.print(","); Serial.flush(); Serial.println(" ");
    //Serial.print("accelz");
    //Serial.print(accel_z, 8); Serial.print(","); Serial.flush(); Serial.println(" ");
    //Serial.print("bnogyrox");
    //Serial.print(bno_gyro_x, 8); Serial.print(","); Serial.flush(); Serial.println(" ");
    //Serial.print("bnogyroy");
    //Serial.print(bno_gyro_y, 8); Serial.print(","); Serial.flush(); Serial.println(" ");
    //Serial.print("bnogyroz");
    //Serial.print(bno_gyro_z, 8); Serial.print(","); Serial.flush(); Serial.println(" ");
    //Serial.print("l3ggyrox");
    //Serial.print(l3g_gyro_x, 8); Serial.print(","); Serial.flush(); Serial.println(" ");
    //Serial.print("l3ggyroy");
    //Serial.print(l3g_gyro_y, 8); Serial.print(","); Serial.flush(); Serial.println(" ");
    //Serial.print("l3ggyroz");
    //Serial.print(l3g_gyro_z, 8); Serial.print(","); Serial.flush(); Serial.println(" "); Serial.flush();

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
    dataFile.print(l3g_gyro_x, 8); dataFile.print(","); dataFile.flush();
    dataFile.print(l3g_gyro_y, 8); dataFile.print(","); dataFile.flush();
    dataFile.print(l3g_gyro_z, 8); dataFile.print(","); dataFile.flush();
    dataFile.print(mpl_alt, 8); dataFile.print(","); dataFile.flush();
    dataFile.print(mpl_pres, 8); dataFile.print(","); dataFile.flush();
    // state matrix begins
    dataFile.print(x[0][0], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(x[1][0], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(x[2][0], 8); dataFile.print(","); dataFile.flush();
    // covariance begins
    dataFile.print(P[0][0], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(P[0][1], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(P[0][2], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(P[1][0], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(P[1][1], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(P[1][2], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(P[2][0], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(P[2][1], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(P[2][2], 8); dataFile.print("\n"); dataFile.flush();
    
    dataFile.close();
  } else {
    LEDWRITING = false;
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

  digitalWrite(8, LEDWRITING);

  delay(DELAY_TIME);
}

void Print_Header() {

  dataFile = SD.open(filename, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    //Serial.println("The header opened");
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
    dataFile.print("L3G Gyro X rad/s,"); dataFile.flush();
    dataFile.print("L3G Gyro Y rad/s,"); dataFile.flush();
    dataFile.print("L3G Gyro Z rad/s,"); dataFile.flush();
    dataFile.print("Altitude m,"); dataFile.flush();
    dataFile.print("Pressure Pa,"); dataFile.flush();
    dataFile.print("Kalman Altitude,"); dataFile.flush();
    dataFile.print("Kalman Velocity,"); dataFile.flush();
    dataFile.print("Kalman Z Acceleration,"); dataFile.flush();
    dataFile.print("Covariance (3x3 Matrix)\n"); dataFile.flush();
    
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
  Matrix.Print((mtx_type *)z, 3, 1, "Current readings:");
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
