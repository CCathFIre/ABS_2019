/*
  SD card datalogger

 This example shows how to log data from three analog sensors
 to an SD card using the SD library.

 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4

 created  24 Nov 2010
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */

#include <SPI.h>
#include <SD.h>

void ReadBestFlight();

const int chipSelect = SDCARD_SS_PIN; // 28  SDCARD_SS_PIN
//float bestAlt[2351], bestVel[2351];
float timet[1000], acel[1000], alt[1000];

String inFileName = "test.txt"; // filenames are limited to 8 characters (?)

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  ReadBestFlight();

  for(int c=0;c<999;c++){
    Serial.print(timet[c]); Serial.print(" "); Serial.print(acel[c]); Serial.print(" "); Serial.println(alt[c]);
  }
}

void loop() {
  /*// make a string for assembling the data to log:
  String dataString = "";

  // read three sensors and append to the string:
  for (int analogPin = 0; analogPin < 3; analogPin++) {
    int sensor = analogRead(analogPin);
    dataString += String(sensor);
    if (analogPin < 2) {
      dataString += ",";
    }
  }

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }*/
  // ignore all above loop code, thats for putting to card
  
}

void ReadBestFlight(){
  File inFile = SD.open(inFileName);
  int c = 0; //Counter variable
  if(inFile){
    while(inFile.available()){
      //bestVel[c] = inFile.parseFloat();
      //bestAlt[c] = inFile.parseFloat();
      timet[c] = inFile.parseFloat();
      acel[c] = inFile.parseFloat();
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
