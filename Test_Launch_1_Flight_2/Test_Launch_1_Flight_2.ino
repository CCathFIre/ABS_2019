//bool RETRACTONCE = false;     // YOU ADDED THIS AND changed in the functions
// myservo.write(actualTheta);   // YOU ADDED START RETRACTED
/*
  if (maxA < altREAD[READINC]) {      // FIX FOR KALMAN
    maxA = altREAD[READINC];
  }

*/
// libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "SparkFunMPL3115A2.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <SPI.h>
#include <MatrixMath.h>
#include <Servo.h>

#define DELAY_TIME 5        // in milliseconds
#define ARMED 0
#define LAUNCHED 1
#define BURNOUT 2
#define APOGEE 3
#define LANDED 4

float timeREAD[1000], acelREAD[1000], altREAD[1000];
String inFileName = "test.txt"; // filenames are limited to 8 characters (?)
int READINC = 0;

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

bool EXTENDONCE = false;
bool RETRACTONCE = false;     // YOU ADDED THIS AND 

const float accelLiftoffThreshold = 50; //m/s^2  50
const float baroLiftoffThreshold = 10; //m   10
const float accelBurnoutThreshold = -5; //m/s^2 -5
//const float baroApogeeThreshold = 5; //m    5
//const float baroLandedThreshold = 0; //m    5
//const float accelFreefallThreshold = 1; //m/s^2  30

float launchA;
float maxA = -100;

int rotation = 0;

int actualTheta = 20;   // starts at 20 full retracted
int minTheta = 20;
int maxTheta = 90;

int morePotent = 2500;
int lessPotent = 1800;

File dataFile;
Servo myservo;

// BNO instantiation
Adafruit_BNO055 bno = Adafruit_BNO055();

// MPL instantiation
MPL3115A2 MPLPressure;

char filename[9] = "data.txt";

void setup() {

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  Wire.begin();        // Join i2c bus

  myservo.attach(7);
  analogReadResolution(12);     // for potentiometer

  myservo.write(actualTheta);   // YOU ADDED START RETRACTED
 
  if (!SD.begin(chipSelect)) {
    return;
  } else{
    SDINIT = true;
  }

  ReadBestFlight();

  // Initialize BNO
  if(bno.begin())
  {
    BNOINIT = true;
  }

  // check if that init returned something // MPLPressure.begin();
  MPLPressure.begin();
  
  bno.setExtCrystalUse(true);

  MPLPressure.setModeAltimeter(); // Measure altitude above sea level in meters (MPL)
  MPLPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  MPLPressure.enableEventFlags(); // Enable all three pressure and temp event flags 
  
  k = -1*Cd*pAir*aRocket / (2*mRocket);
  
  x[0][0] = altREAD[READINC]; //Set initial altitude based on sensor reading

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

  launchA = MPLPressure.readAltitude();

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

//  Serial.println("We are looping");



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
  //double accel_z = bno_linearAccel.z();
  double accel_z = acelREAD[READINC];
  
  double bno_gyro_x = bno_gyro.x();
  double bno_gyro_y = bno_gyro.y();
  double bno_gyro_z = bno_gyro.z();

  // Altitude in m
  // Pressure in Pa
  //float mpl_alt = MPLPressure.readAltitude();
  float mpl_alt = altREAD[READINC];
  
  float mpl_pres = MPLPressure.readPressure();

  if (maxA < altREAD[READINC]) {      // FIX FOR KALMAN
    maxA = altREAD[READINC];
  }

  //Kalman(mpl_alt, accel_z);




  /* 
   *  BEGIN DATA WRITING
   */

  dataFile = SD.open(filename, FILE_WRITE);

  // if the file is available, write to it:
  if (true) {
    
    LEDWRITING = true;

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
    LEDWRITING = false;
  }

  switch(flightstate){
    case ARMED:
      if ( accel_z > accelLiftoffThreshold || ( mpl_alt - launchA) > baroLiftoffThreshold){
        flightstate = LAUNCHED;
      }
    break;
    case LAUNCHED:
      if ( accel_z < accelBurnoutThreshold){
        flightstate = BURNOUT;
      }
    break;
    case BURNOUT:
      fullExtension();
      if ( maxA > mpl_alt ){
        flightstate = APOGEE;
      }
    break;
    case APOGEE:
      fullRetraction();
      if ( mpl_alt < launchA + 10 ){                  //&& x[1][0] < 1 ){
        flightstate = LANDED;
      }
    break;
    case LANDED:
    break;
  }

  digitalWrite(8, LEDWRITING);

  delay(DELAY_TIME);
  READINC += 1;
  Serial.print("ALT: ");
  Serial.println(mpl_alt);
  Serial.print("ACCEL Z: ");
  Serial.println(accel_z);
  Serial.println("");

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

bool mplWorking(){
  
}

bool ptmrWorking(){
  
}

void fullExtension(){  

  if (! EXTENDONCE ){
    EXTENDONCE = true;
    for (actualTheta = minTheta; actualTheta <= maxTheta; actualTheta += 5) {  // MAXIMIZE
      myservo.write(actualTheta);                         // tell servo to go to position in variable 'pos'
      delay(15);                                          // MINIMIZE
      rotation = analogRead(potPin);
    }
  }
  if( rotation < morePotent ){              // If tabs didn't full extend, will continue to extend
    actualTheta += 1;
    myservo.write(actualTheta);
  }
}


void fullRetraction(){  

  if (! RETRACTONCE ){
    RETRACTONCE = true;
    for (actualTheta = maxTheta; actualTheta >= minTheta; actualTheta -= 5) {  // MAXIMIZE
      myservo.write(actualTheta);                         // tell servo to go to position in variable 'pos'
      delay(15);                                          // MINIMIZE
      rotation = analogRead(potPin);
    }
  }
  if( rotation < lessPotent ){              // If tabs didn't full retract, will continue to try
    actualTheta -= 1;
    myservo.write(actualTheta);
  }
}

void ReadBestFlight(){
  File inFile = SD.open(inFileName);
  int c = 0; //Counter variable
  if(inFile){
    while(inFile.available()){
      //bestVel[c] = inFile.parseFloat();
      //bestAlt[c] = inFile.parseFloat();
      timeREAD[c] = inFile.parseFloat();
      acelREAD[c] = (inFile.parseFloat()) * 0.3048;
      altREAD[c] = (inFile.parseFloat()) * 0.3048 ;
      c++;
    }
    inFile.close();
  }
  else{
    Serial.println("Error: Unable to open comparison datafile.");
    while(1); //Freeze code if comparison dataset cannot be read
  }
}
