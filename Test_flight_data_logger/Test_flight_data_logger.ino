// libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "SparkFunMPL3115A2.h"
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// BNO instantiation
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();

// MPL instantiation
MPL3115A2 myPressure;

void setup() {

  Serial.begin(9600);



  // BNO SETUP CODE

  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t bno_temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(bno_temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");



  // MPL SETUP CODE
  
  Wire.begin();        // Join i2c bus

  myPressure.begin(); // Get sensor online
  
  myPressure.setModeAltimeter(); // Measure altitude above sea level in meters
  
  
}

void loop() {


  // BNO LOOP 

  // vector creation
  // acceleration units in m/s^2
  // gyroscope units in rad/s
  imu::Vector<3> bno_accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> bno_linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> bno_gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // BNO data display
  Serial.print("BNO Acceleration");
  Serial.print("X: ");
  Serial.print(bno_accelerometer.x());
  Serial.print(" Y: ");
  Serial.print(bno_accelerometer.y());
  Serial.print(" Z: ");
  Serial.print(bno_accelerometer.z());
  Serial.print("\t\t");

  Serial.print("BNO Linear Acceleration");
  Serial.print("X: ");
  Serial.print(bno_linearAccel.x());
  Serial.print(" Y: ");
  Serial.print(bno_linearAccel.y());
  Serial.print(" Z: ");
  Serial.print(bno_linearAccel.z());
  Serial.print("\t\t");

  Serial.print("BNO Gyroscope");
  Serial.print("X: ");
  Serial.print(bno_gyro.x());
  Serial.print(" Y: ");
  Serial.print(bno_gyro.y());
  Serial.print(" Z: ");
  Serial.print(bno_gyro.z());
  Serial.print("\t\t");



  // MPL LOOP
  














  // delay for data sampling
  delay(BNO055_SAMPLERATE_DELAY_MS);


  
}
