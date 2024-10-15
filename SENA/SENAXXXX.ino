// SEN55 with SD and RTC, approx 10 second measurements. Script version 0.0.0

/* 
 * SEN55........MEGA
 * VCC.........5V
 * GND.........GND
 * SDA.........SDA
 * SCL.........SCL
 * SEL.........GND
 * NC..........dont connect
 *
 * SD module...Mega
 * GND.........GND
 * VCC.........5V
 * MISO........50
 * MOSI........51
 * SCK.........52
 * CS..........53
 *
 * DS3231......Mega //sketch parts from DS3231_TEST
 * GND.........GND
 * VCC.........5V
 * SDA.........SDA
 * SCL.........SCL
 *
 * Green LED....MEGA
 * long.........8
 * Short..220R..GND
*/

#include <SPI.h>
#include "SdFat.h"
#include <Wire.h>
#include "RTClib.h"
#include <Arduino.h>
#include <SensirionI2CSen5x.h>

#define MAXBUF_REQUIREMENT 48

#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#define USE_PRODUCT_INFO
#endif

SensirionI2CSen5x sen5x;
SdFat SD;
RTC_DS3231 rtc;

#define SD_CS_PIN SS

// ######### SET THE DEVICEID ####################
String deviceID = "SENA0001";
String softwareVersion = "V000";
String sensors = "SEN55";
int ledposition = 8;

File metaFile;
String meta_filename = "metadata_"+deviceID+".txt";
String metaheader = "datetime;software;deviceID;SEN55serial;sensors";
File dataFile;
String data_filename= "";
String month_cov = "";
String dataheader = "counter;datetime;PM1;PM2_5;PM4;PM10;tempC;humidity;VOC;NOX";
String data ="";
String serialdata = "";
String senString = "";
int count = 1; //counter to keep track of minute averages
unsigned long previousMillis = 0;
unsigned long interval = 60000UL;
bool sdFailure = false; //variable to keep track of sdFailure. If 1: try to reinitialize during loop.

long counter = 0; // For every restart, a counter is included


void setup() {
  // put your setup code here, to run once:
  pinMode(ledposition,OUTPUT);
  Serial.begin(9600);
  Serial.print("Initializing RTC module...");
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  DateTime now = rtc.now();
  Serial.println("Current time according to RTC: "+String(now.timestamp(DateTime::TIMESTAMP_FULL)));
  
  Serial.print("Initializing SD card...");

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("initialization failed!");
    return;
  }

  Wire.begin();

  sen5x.begin(Wire);
  uint16_t error;
  char errorMessage[256];
  error = sen5x.deviceReset();
  if (error) {
      Serial.print("Error trying to execute deviceReset(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  }

  Serial.println("initialization done.");

  // Write some device info to serial
  char serialNumber[32];
  sen5x.getSerialNumber(serialNumber, 32);
  Serial.println("Device ID: "+deviceID+"\tSEN55 Serial: "+String(serialNumber));

  // Write metadata to the SD card
  bool metafile_exists = SD.exists(meta_filename);
  metaFile = SD.open(meta_filename, FILE_WRITE);
  // if the file opened okay, write to it:
  if (metaFile) {
    if(!metafile_exists) {
      metaFile.println(metaheader);
    }
    String metadata = String(now.timestamp(DateTime::TIMESTAMP_FULL))+';' + softwareVersion + ';' + deviceID + ';' + String(serialNumber)+';'+sensors;
    metaFile.println(metadata);
    // close the file:
    metaFile.close();          
  } else {
    // if the file didn't open, print an error
    Serial.println("error opening metadata file");      
  } 

  error = sen5x.startMeasurement();
  if (error) {
      Serial.print("Error trying to execute startMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  }

  Serial.println(F("Measurements starting..."));

  digitalWrite(ledposition, HIGH);
  delay(200);
  digitalWrite(ledposition, LOW);
  delay(200); 
  digitalWrite(ledposition, HIGH);
  delay(200);
  digitalWrite(ledposition, LOW);
  delay(200);
  digitalWrite(ledposition, HIGH);
  delay(200);
  digitalWrite(ledposition, LOW);
     
  delay(10000);

}

void loop() {
  uint16_t error;
  char errorMessage[256];

  delay(1000);

  // Get the data
  float pm1;
  float pm25;
  float pm4;
  float pm10;
  float RH;
  float T;
  float VOC;
  float NOX;

  error = sen5x.readMeasuredValues(
    pm1, pm25, pm4, pm10, RH, T, VOC, NOX);

  if (error) {
    Serial.print("Error trying to execute readMeasuredValues(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    senString = "nan;nan;nan;nan;nan;nan;nan;nan";
  } else {
    senString = String(pm1)+";"+String(pm25)+";"+String(pm4)+";"+String(pm10)+';'+String(T)+';'+String(RH)+';'+String(VOC)+';'+String(NOX);
  }

  // Write the data
  DateTime now = rtc.now();

  month_cov = String(now.month());
  if(now.month() <10) month_cov = "0"+ month_cov;
  data_filename = deviceID + "_" + String(now.year())+month_cov+".txt";

  data = String(String(counter)+';'+now.timestamp(DateTime::TIMESTAMP_FULL))+";"+senString;
  serialdata = String(String(counter)+'\t'+now.timestamp(DateTime::TIMESTAMP_FULL))+'\t'+senString;
  Serial.println(serialdata);

  // Decide whether headers need to be included    
  bool datafile_exists = SD.exists(data_filename);

  dataFile = SD.open(data_filename, FILE_WRITE);
  // if the file opened okay, write to it:
  if (dataFile) {
    if(!datafile_exists) {
      dataFile.println(dataheader);
    }

    dataFile.println(data);
    // close the file:
    dataFile.close();
    digitalWrite(ledposition, HIGH);
    delay(500);
    digitalWrite(ledposition, LOW);     
  } else {
    // if the file didn't open, print an error and blink 10 times:
    Serial.println("error opening datafile");
  }      
  
  ++counter;
  delay(8500);
}
