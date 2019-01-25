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

void ReadBestFlight();

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

float timet[1000], acel[1000], alt[1000]; // dummy data read into these arrays, make them large enough

int indx = 0; // This is used to iterate through the arrays

// file dummy data is read into
String inFileName = "test.txt"; // filenames are limited to 8 characters (?)

File dataFile;

// BNO instantiation
//Adafruit_BNO055 bno = Adafruit_BNO055();

// MPL instantiation
//MPL3115A2 MPLPressure;

//L3G instantiation?
/* Assign a unique ID to this sensor at the same time */
//Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

char filename[9] = "data.txt"; // file written to

void setup() {

  Serial.begin(9600);   // printing to screen // TESTING

  while(!Serial) ;  // TESTING
  
  Wire.begin();        // Join i2c bus

  if (!SD.begin(chipSelect)) {
    //Serial.println("The SD card has not exploded!!!");
    return;
  } else{
    SDINIT = true;
  }

  /* Initialise the sensors */
  //if(!bno.begin())
  //{
  //  /* There was a problem detecting the BNO055 ... check your connections */
  //  //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //  // Record failure in SD card
  //  while(1);
  //} else
  //{
    BNOINIT = true; // sensors are not used so don't need to be initialized
  //}
   /* Enable auto-ranging */
  //gyro.enableAutoRange(true);
  //if(!gyro.begin())
  //{
  //  /* There was a problem detecting the L3GD20 ... check your connections */
  //  //Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
  //  while(1);
  //}
  //else
  //{
    L3GINIT = true;
  //}

  // check if that init returned something // MPLPressure.begin();
  //MPLPressure.begin();
  

  //if ( BNOINIT && L3GINIT && SDINIT )
  //{
    LEDINITIAL = true;
  //}
  
  
  k = -1*Cd*pAir*aRocket / (2*mRocket);
  
  //x[0][0] = MPLPressure.readAltitude(); //Set initial altitude based on sensor reading
  x[0][0] = 347; //Set initial altitude to something that makes sense
  lastT = 519000; // Set this to a reasonable value time value soon before recorded start

  launchA = x[0][0];

  ReadBestFlight(); // reads in dummy data

  Print_Header();

  //pinMode(6, OUTPUT);
  //pinMode(8, OUTPUT);
  //digitalWrite(6, LEDINITIAL);
}

void loop() {
  /* Get a new sensor event */ 

  if (maxA < x[0][0]) {
    maxA = x[0][0];
  }

  Kalman(alt[indx], acel[indx]);

  dataFile = SD.open(filename, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    
    LEDWRITING = true;

    dataFile.print(flightstate); dataFile.print(","); dataFile.flush();
    dataFile.print(timet[indx]); dataFile.print(","); dataFile.flush();
    dataFile.print(acel[indx], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(alt[indx], 8); dataFile.print(","); dataFile.flush();
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

  //digitalWrite(8, LEDWRITING);
  if (indx < 1000){
    indx++; // increments index
  } else {
    Serial.println("Simulation Over Asshole");
  }
}

void Print_Header() {

  dataFile = SD.open(filename, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    //Serial.println("The header opened");
    dataFile.print("Flight State,"); dataFile.flush();
    dataFile.print("Time ms,"); dataFile.flush();
    dataFile.print("Z Acceleration m/s^2,"); dataFile.flush();
    dataFile.print("Altitude m,"); dataFile.flush();
    dataFile.print("Kalman Altitude,"); dataFile.flush();
    dataFile.print("Kalman Velocity,"); dataFile.flush();
    dataFile.print("Kalman Z Acceleration"); dataFile.flush();
    dataFile.print("Covariance (3x3 Matrix)\n"); dataFile.flush();
      
    dataFile.close();
  } else {
    // if the file didn't open, print an error:
    //Serial.println("this boi don;t open");
  }

}

void Kalman(float altitude,float zAccel) {
  dT = (timet[indx] - lastT)/1000;
  float kTabs = 0; // UPDATE THIS WITH ACTUAL TAB DRAG

  // Start Kalman filter code
  mtx_type z[3][1] = {{altitude}, {x[1][0]}, {zAccel}};
  //Matrix.Print((mtx_type *)z, 3, 1, "Current readings:"); // I'm not really sure if this is important
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

  lastT = timet[indx];

  //Matrix.Print((mtx_type *)x, 3, 1, "State of the world:");
  //Serial.println("\n");
  
}

// Reads in dummy data (time, acceleration, altitude) from a sample flight
void ReadBestFlight(){
  File inFile = SD.open(inFileName);
  int c = 0; //Counter variable
  if(inFile){
    while(inFile.available()){
      // .parseFloat() will read the file and record the next float it hits, how the file is delimited is irrelevant
      timet[c] = inFile.parseFloat();
      acel[c] = -1 * (inFile.parseFloat()); // our acel data was negative, sensor was oriented wrong in the test flight
      alt[c] = inFile.parseFloat();
      c++;
    }
    inFile.close();
  }
  else{
    Serial.println("Error: Unable to open comparison datafile.");
    while(1); //Freeze code if comparison dataset cannot be read
  }
}
