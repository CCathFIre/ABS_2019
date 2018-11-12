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
MPL3115A2 MPLPressure;

//L3G instantiation?
/* Assign a unique ID to this sensor at the same time */
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);


// L3G startup function seperate from setup() and loop()
void displaySensorDetails(void)
{
  sensor_t L3G_sensor;
  gyro.getSensor(&L3G_sensor);
  Serial.print  ("L3G startup data");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(L3G_sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(L3G_sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(L3G_sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(L3G_sensor.max_value); Serial.println(" rad/s");
  Serial.print  ("Min Value:    "); Serial.print(L3G_sensor.min_value); Serial.println(" rad/s");
  Serial.print  ("Resolution:   "); Serial.print(L3G_sensor.resolution); Serial.println(" rad/s");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


void setup() {

  Serial.begin(9600);



  // BNO SETUP CODE

  // this line seems unncecessary
  //Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

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
  Serial.print("BNO Current Temperature: ");
  Serial.print(bno_temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");



  // MPL SETUP CODE
  
  Wire.begin();        // Join i2c bus

  MPLPressure.begin(); // Get MPL sensor online
  
  MPLPressure.setModeAltimeter(); // Measure altitude above sea level in meters (MPL)
  
  MPLPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  MPLPressure.enableEventFlags(); // Enable all three pressure and temp event flags 

  /* Display the current temperature in degrees C */
  float mpl_temp = MPLPressure.readTemp();
  Serial.print("MPL Temp(c):");
  Serial.print(mpl_temp, 2);


  // L3G SETUP CODE

  Serial.println("L3G Gyroscope Test"); Serial.println("");
  
  /* Enable auto-ranging */
  gyro.enableAutoRange(true);
  
  /* Initialise the sensor */
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
  
  // This is code for the L3G that I don't understand, maybe it prints stuff?
  /* Display some basic information on this sensor */
  displaySensorDetails();



  
  
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

  // What number should go in here? Should this be removed
  // delay for data sampling
  delay(BNO055_SAMPLERATE_DELAY_MS);

  Serial.println();
  

  // MPL LOOP

  // Altitude in m
  float mpl_alt = MPLPressure.readAltitude();
  Serial.print("MPL Altitude(m):");
  Serial.print(mpl_alt, 2);

  // Pressure in Pa
  float mpl_pres = MPLPressure.readPressure();
  Serial.print("MPL Pressure(Pa):");
  Serial.print(mpl_pres, 2);

  Serial.println();



  // L3G LOOP

   /* Get a new sensor event */ 
  sensors_event_t L3G_event; 
  gyro.getEvent(&L3G_event);
 
  /* Display the results (speed is measured in rad/s) */
  Serial.print("L3G Gyroscope Data");
  Serial.print("X: "); Serial.print(L3G_event.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(L3G_event.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(L3G_event.gyro.z); Serial.print("  ");
  Serial.println("rad/s ");
  delay(500);   // Issue: how does this work with the other delay() funtions like the one below


  









  
}
