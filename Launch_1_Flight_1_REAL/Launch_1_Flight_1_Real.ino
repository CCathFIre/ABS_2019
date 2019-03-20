// libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "SparkFunMPL3115A2.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <SPI.h>
#include <MatrixMath.h>

#define DELAY_TIME 10        // in milliseconds
#define ARMED 0
#define LAUNCHED 1
#define BURNOUT 2
#define APOGEE 3
#define LANDED 4

int testBNO = 0;
int testMPL = 0;
int MPLBAD = 0;
float mpl_alt = 300;

float lastT, dT;
float k;
const float Cd = 0.25; // UPDATE THESE VALUES!!
const float pAir = 1.225;
const float aRocket = 0.1;
const float mRocket = 10;
int potPin = A6;
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
bool SDINIT = false;
bool MPLINIT = false;
bool POTENTINIT = false;

const float accelLiftoffThreshold = 50; //m/s^2  50
const float baroLiftoffThreshold = 10; //m   10
const float accelBurnoutThreshold = -5; //m/s^2 -5
//const float baroApogeeThreshold = 5; //m    5
//const float baroLandedThreshold = 0; //m    5
//const float accelFreefallThreshold = 1; //m/s^2  30

float launchA;
float maxA = -100;

int rotation = 0;

File dataFile;

// BNO instantiation
Adafruit_BNO055 bno = Adafruit_BNO055();

// MPL instantiation
MPL3115A2 MPLPressure;

char filename[9] = "data.txt";

// data buffers
const int BUFFERSIZE = 1;
int BUFFINC = 0;

int flightstateBUFF[BUFFERSIZE];
float timeBUFF[BUFFERSIZE];
float bno_tempBUFF[BUFFERSIZE];
float mpl_tempBUFF[BUFFERSIZE];
float accel_xBUFF[BUFFERSIZE];
float accel_yBUFF[BUFFERSIZE];
float accel_zBUFF[BUFFERSIZE];
float bno_gyro_xBUFF[BUFFERSIZE];
float bno_gyro_yBUFF[BUFFERSIZE];
float bno_gyro_zBUFF[BUFFERSIZE];
float mpl_altBUFF[BUFFERSIZE];
float mpl_presBUFF[BUFFERSIZE];
float rotationBUFF[BUFFERSIZE];

void setup() {

  Serial.begin(9600);
  while (!Serial);
  
  Wire.begin();        // Join i2c bus

  analogReadResolution(12);     // for potentiometer
  pinMode(potPin,INPUT);        // potPin
 
  if (!SD.begin(chipSelect)) {
    return;
  } else{
    SDINIT = true;
  }

  // Initialize BNO
  if(bno.begin())
  {
    BNOINIT = true;
  }

  // check if that init returned something // MPLPressure.begin();
  MPLPressure.begin();
  
  bno.setExtCrystalUse(true);

  MPLPressure.setModeAltimeter(); // Measure altitude above sea level in meters (MPL)
  MPLPressure.setOversampleRate(0); // Fuck the police
  MPLPressure.enableEventFlags(); // Enable all three pressure and temp event flags 
  
  k = -1*Cd*pAir*aRocket / (2*mRocket);
  
  x[0][0] = MPLPressure.readAltitude(); //Set initial altitude based on sensor reading

  // Initialize MPL
  if(x[0][0] >= 0 && x[0][0] <= 2500)
  {
    MPLINIT = true;
  }

  rotation = analogRead(potPin);
  if(rotation >= 0 && rotation <= 5000)
  {
    POTENTINIT = true;
  }
  
  lastT = millis();

  launchA = x[0][0];

//  Serial.println("WITNESS ME2");

  delay(DELAY_TIME); // Delay time for sensor sampling rate

  Print_Header();

  if ( BNOINIT && SDINIT && MPLINIT && POTENTINIT)
  {
    LEDINITIAL = true;
  }

  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT); 
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT); 
  // what do pins 6 & 8 do??
  pinMode(6, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(0, LEDINITIAL);  // all initialized
  digitalWrite(1, BNOINIT);     // BNO good
  digitalWrite(2, MPLINIT);     // MPL good
  digitalWrite(3, POTENTINIT);  // Potent "is very potent"
  digitalWrite(4, SDINIT);      // SD good
}

void loop() {

  dataFile = SD.open(filename, FILE_WRITE);

  //  Serial.println("We are looping");
  
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

  testBNO = millis();

  double accel_x = bno_linearAccel.x();
  double accel_y = bno_linearAccel.y();
  double accel_z = bno_linearAccel.z();

  double bno_gyro_x = bno_gyro.x();
  double bno_gyro_y = bno_gyro.y();
  double bno_gyro_z = bno_gyro.z();

  Serial.print("BNO:    ");
  Serial.print(millis() - testBNO);
  Serial.print("\n");
  // Altitude in m
  // Pressure in Pa

  testMPL = millis();
  

  mpl_alt = MPLPressure.readAltitude();

//  float mpl_pres = MPLPressure.readPressure();

  Serial.print("MPL:    ");
  Serial.print(millis() - testMPL);
  Serial.print("\n");

  if (maxA < x[0][0]) {
    maxA = x[0][0];
  }

  //Kalman(mpl_alt, accel_z);




  /* 
   *  BEGIN DATA WRITING
   */

  // if the file is available, write to it:
  if (true) {
    
    LEDWRITING = true;
   
    //fillBuffers();
    
    /*
    //Serial.println("WITNESS ME!!!");
    //Serial.print("accelx");
    //Serial.print(accel_x, 8); Serial.print(","); Serial.flush(); Serial.println(" ");
    //Serial.print("accely");
    //Serial.print(accel_y, 8); Serial.print(","); Serial.flush(); Serial.println(" ");
    //Serial.print("accelz");
    //Serial.print(accel_z, 8); Serial.print(","); Serial.flush(); Serial.println(" ");
    Serial.print("bnogyrox");
    Serial.print(bno_gyro_x, 8); Serial.print(","); Serial.flush(); Serial.println(" ");
    Serial.print("bnogyroy");
    Serial.print(bno_gyro_y, 8); Serial.print(","); Serial.flush(); Serial.println(" ");
    Serial.print("bnogyroz");
    Serial.print(bno_gyro_z, 8); Serial.print(","); Serial.flush(); Serial.println(" ");
    Serial.print("mpl_alt");
    Serial.print(mpl_alt, 8); Serial.print(","); Serial.flush(); Serial.println(" ");
    Serial.print("mpl_pres");
    Serial.print(mpl_pres, 8); Serial.print(","); Serial.flush(); Serial.println(" ");
    Serial.flush();
    */

    flightstateBUFF[BUFFINC] = flightstate;
    timeBUFF[BUFFINC] = millis();
    bno_tempBUFF[BUFFINC] = bno_temp;
    mpl_tempBUFF[BUFFINC] = mpl_temp;
    accel_xBUFF[BUFFINC] = accel_x;
    accel_yBUFF[BUFFINC] = accel_y;
    accel_zBUFF[BUFFINC] = accel_z;
    bno_gyro_xBUFF[BUFFINC] = bno_gyro_x;
    bno_gyro_yBUFF[BUFFINC] = bno_gyro_y;
    bno_gyro_zBUFF[BUFFINC] = bno_gyro_z;
    mpl_altBUFF[BUFFINC] = mpl_alt;
    //mpl_presBUFF[BUFFINC] = mpl_pres;
    rotationBUFF[BUFFINC] = rotation;

    BUFFINC++;
    if (BUFFINC == BUFFERSIZE){
      dataFile = SD.open(filename, FILE_WRITE);
      for (int i = 0; i < BUFFERSIZE; i++){
        dataFile.print(flightstateBUFF[i]); dataFile.print(","); dataFile.flush();
        dataFile.print(timeBUFF[i]); dataFile.print(","); dataFile.flush();
        dataFile.print(bno_tempBUFF[i]); dataFile.print(","); dataFile.flush();
        dataFile.print(mpl_tempBUFF[i], 8); dataFile.print(","); dataFile.flush();
        dataFile.print(accel_xBUFF[i], 8); dataFile.print(","); dataFile.flush();
        dataFile.print(accel_yBUFF[i], 8); dataFile.print(","); dataFile.flush();
        dataFile.print(accel_zBUFF[i], 8); dataFile.print(","); dataFile.flush();
        dataFile.print(bno_gyro_xBUFF[i], 8); dataFile.print(","); dataFile.flush();
        dataFile.print(bno_gyro_yBUFF[i], 8); dataFile.print(","); dataFile.flush();
        dataFile.print(bno_gyro_zBUFF[i], 8); dataFile.print(","); dataFile.flush();
        dataFile.print(mpl_altBUFF[i], 8); dataFile.print(","); dataFile.flush();
        //dataFile.print(mpl_presBUFF[i], 8); dataFile.print(","); dataFile.flush();
        dataFile.print("Bullshit"); dataFile.print(","); dataFile.flush();
        dataFile.print(rotationBUFF[i], 8); dataFile.print("\n"); dataFile.flush();
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
      }
    
      dataFile.close();       // is this the issue?
      BUFFINC = 0;
    }
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
