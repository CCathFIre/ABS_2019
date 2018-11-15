// libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "SparkFunMPL3115A2.h"
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <SPI.h>

#define DELAY_TIME 50
// 50
const int chipSelect = SDCARD_SS_PIN;
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

  Serial.begin(9600);   // printing to screen

  // DELETE THIS BEFORE FLIGHT
  // DELETE
  // DELETE

  while(!Serial) ;      

  // ^^^ DELETE THIS BEFORE FLIGHT
  // ^^^
  // ^^^
  
  Wire.begin();        // Join i2c bus

  if (!SD.begin(chipSelect)) {
    Serial.println("The SD card has not exploded!!!");
    return;
  }

  /* Initialise the sensors */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    // Record failure in SD card
    while(1);
  }
   /* Enable auto-ranging */
  gyro.enableAutoRange(true);
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
  
  MPLPressure.begin();
  
  bno.setExtCrystalUse(true);

  MPLPressure.setModeAltimeter(); // Measure altitude above sea level in meters (MPL)
  MPLPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  MPLPressure.enableEventFlags(); // Enable all three pressure and temp event flags 

  Print_Header();
}

void loop() {

  Serial.println("We are looping");

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

  dataFile = SD.open(filename, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {

    Serial.println("WITNESS ME!!!");
    
    dataFile.print(millis()); dataFile.print(","); dataFile.flush();
    dataFile.print(bno_temp); dataFile.print(","); dataFile.flush();
    dataFile.print(mpl_temp); dataFile.print(","); dataFile.flush();
    dataFile.print(accel_x); dataFile.print(","); dataFile.flush();
    dataFile.print(accel_y); dataFile.print(","); dataFile.flush();
    dataFile.print(accel_z); dataFile.print(","); dataFile.flush();
    dataFile.print(bno_gyro_x); dataFile.print(","); dataFile.flush();
    dataFile.print(bno_gyro_y); dataFile.print(","); dataFile.flush();
    dataFile.print(bno_gyro_z); dataFile.print(","); dataFile.flush();
    dataFile.print(l3g_gyro_x); dataFile.print(","); dataFile.flush();
    dataFile.print(l3g_gyro_y); dataFile.print(","); dataFile.flush();
    dataFile.print(l3g_gyro_z); dataFile.print(","); dataFile.flush();
    dataFile.print(mpl_alt); dataFile.print(","); dataFile.flush();
    dataFile.print(mpl_pres); dataFile.print("\n"); dataFile.flush();
    
    dataFile.close();
  } else {
    Serial.print("whoops\n");
  }

  delay(DELAY_TIME);
}

void Print_Header() {

  Serial.println("We header");

  dataFile = SD.open(filename, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    Serial.println("The header opened");
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
    dataFile.print("Pressure Pa\n"); dataFile.flush();
    
    
    dataFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("this boi don;t open");
  }

}
