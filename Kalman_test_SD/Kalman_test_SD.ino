///Look into extended Kalman filters for modeling the controls

#include <MatrixMath.h>
#include <SPI.h>
//#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

const int chipSelect = 28;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_BMP280 bmp; // I2C

//const float p0 = 0.1;
//const float r = 0.15;

mtx_type x[3][1] = {{0}, {0}, {0}};
mtx_type P[3][3] = {{0.005, 0, 0}, {0, 0.0122, 0}, {0, 0, 0.0176}};
mtx_type R[3][3] = {{0.1, 0, 0}, {0, 0.1, 0}, {0, 0, 0.4}};
mtx_type Theta[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
mtx_type I3[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
mtx_type Q[3][3] = {{0, 0, 0}, {0, 0.001, 0}, {0, 0, 0.001}};

float lastT, dT;
float k;
float Cd = 0.25; // UPDATE THESE VALUES!!
float pAir = 1.225;
float aRocket = 0.1;
float mRocket = 10;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Initializing hardware...");

  /*if (!SD.begin(chipSelect)) {
    return;
  }*/

  if (!bmp.begin() || !accel.begin()) {
    return;
  }

  Serial.println("Hardware initialized");

  accel.setRange(ADXL345_RANGE_16_G);

  k = -1*Cd*pAir*aRocket / (2*mRocket);
}

void loop() {
  sensors_event_t event; 
  accel.getEvent(&event);
  float xAccel = event.acceleration.x;
  float yAccel = event.acceleration.y;
  float zAccel = event.acceleration.z;
  float altitude = bmp.readAltitude(1013.25);
  dT = (millis() - lastT)/1000;
  float kTabs = 0; // UPDATE THIS WITH ACTUAL TAB DRAG

  // Start Kalman filter code
  mtx_type z[3][1] = {{altitude}, {x[1][0]}, {yAccel}};
  mtx_type K[3][3];
  mtx_type tempM[3][3];
  mtx_type tempV[3][1];

  // Calculate Kalman gain
  Matrix.Add((mtx_type *)P, (mtx_type *)R, 3, 3, (mtx_type *)tempM);
  Matrix.Invert((mtx_type *)tempM, 3);
  Matrix.Multiply((mtx_type *)P, (mtx_type *)tempM, 3, 3, 3, (mtx_type *)K);

  // Update estimate
  Matrix.Subtract((mtx_type *)z, (mtx_type *)x, 3, 1, (mtx_type *)tempV);
  Matrix.Multiply((mtx_type *)K, (mtx_type *)tempV, 3, 3, 1, (mtx_type *)tempV);
  Matrix.Add((mtx_type *)x, (mtx_type *)tempV, 3, 1, (mtx_type *)x);

  // Update covariance
  Matrix.Subtract((mtx_type *)I3, (mtx_type *)K, 3, 3, (mtx_type *)tempM);
  Matrix.Multiply((mtx_type *)tempM, (mtx_type *)P, 3, 3, 3, (mtx_type *)P);

  // Project into next time step
  mtx_type tempTheta[3][3];
  Theta[0][1] = dT;
  Theta[0][2] = 0.5*dT*dT;
  Theta[1][2] = dT;
  Theta[2][1] = k + kTabs;
  Matrix.Multiply((mtx_type *)Theta, (mtx_type *)x, 3, 3, 1, (mtx_type *)x);
  Matrix.Transpose((mtx_type *)Theta, 3, 3, (mtx_type *)tempTheta);
  Matrix.Multiply((mtx_type *)Theta, (mtx_type *)P, 3, 3, 3, (mtx_type *)tempM);
  Matrix.Multiply((mtx_type *)tempM, (mtx_type *)tempTheta, 3, 3, 3, (mtx_type *)tempM);
  Matrix.Add((mtx_type *)tempM, (mtx_type *)Q, 3, 3, (mtx_type *)P);
  
  lastT = millis();
  // End Kalman filter code


  delay(50); // Wait 50 ms to get next sensor reading
  /*
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println("");
    dataFile.close();
  }
  */
  
}









