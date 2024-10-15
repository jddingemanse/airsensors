// SPSA: SPS30 Sensirion Arduino sensor

// ######### SET THE DEVICEID ####################
String deviceID = "SPSAXXXX"; #Enter the deviceID

// ######### SET THE LED PIN ##################### Normally pin 8
int ledposition = 8;

//taking 1 minute measurements
/*
 * SPS030......Mega //sketch parts from sps30 Basic Readings example
 * 1 VCC.......5V
 * 2 SDA.......SDA (nb: with 10k pull up resistor, or other I2C sensor)
 * 3 SCL.......SCL (nb: with 10k pull up resistor, or other I2C sensor)
 * 4 Select....GND (select I2c)
 * 5 GND.......GND
 *
 * BME280......Mega //sketch parts from SparkFun BME280
 * VIN.........5V
 * GND.........GND
 * SDA.........SDA
 * SCL.........SCL

 * SD module...Mega //sketch parts from SdFat SoftwareSPI
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
 * Green LED....Mega
 * +............8 [+: longer leg]
 * -..resistor..GND [-: smaller leg]
 */

String sensors = "SPS30,BME280";
String softwareVersion = "V03-60S";

#include <SPI.h>
#include "SdFat.h"
#include <Wire.h>
#include "RTClib.h"
#include "sps30.h"
#include <LowPower.h>
#include "SparkFunBME280.h"

SPS30 sps30;
SdFat SD;
RTC_DS3231 rtc;
BME280 bmeSensor;

#define SD_CS_PIN SS
#define SP30_COMMS I2C_COMMS
#define DEBUG 0
#define PERFORMCLEANNOW 1 //SPS030 cleaning
File metaFile;
String meta_filename = "metadata_"+deviceID+".txt";
String metaheader = "datetime;software;deviceID;spsSerial;sensors";
File dataFile;
String data_filename= "";
String month_cov = "";
String dataheader = "counter;datetime;PM1;PM2_5;PM4;PM10;NumPM1;NumPM2_5;NumPM4;NumPM10;PartSize;tempC;humidity;pressure;meta";
String data ="";
String serialdata = "";
struct sps_values val;
struct sps_values valAvg;
String pmString = "";
String bmeString = "";
String metaString = "";
int count = 1; //counter to keep track of minute averages
unsigned long previousMillis = 0;
unsigned long interval = 60000UL;
bool sdFailure = false; //variable to keep track of sdFailure. If 1: try to reinitialize during loop.

long counter = 0; // For every restart, a counter is included

void setup() {
  // put your setup code here, to run once:
  pinMode(ledposition,OUTPUT);
  Serial.begin(9600);
  Serial.println("Device ID: "+deviceID);
  Serial.print("Initializing RTC module...");
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  DateTime now = rtc.now();
  Serial.println("Current time according to RTC: "+String(now.timestamp(DateTime::TIMESTAMP_FULL)));
  Serial.println("Starting low-power to let battery charge...");
  delay(1000);
  for(int i=0;i<6;i++){
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER5_OFF, TIMER4_OFF, TIMER3_OFF, 
          TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART3_OFF, 
          USART2_OFF, USART1_OFF, USART0_OFF, TWI_OFF);  
  }

  Serial.print("Initializing SD card...");

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("initialization failed!");
    return;
  }

  Wire.begin();
  bmeSensor.setI2CAddress(0x76);
  if (bmeSensor.beginI2C() == false)
  {
    Serial.println("The BME280 sensor did not respond. Please check wiring.");
    while(1); //Freeze
  }

  Serial.println("initialization done.");
// set driver debug level
  sps30.EnableDebugging(DEBUG);
  // Begin communication channel TO sp30;
  if (!sps30.begin(SP30_COMMS)) 
    {
   while(1);
    }
  
  // check for SPS30 connection
 if (!sps30.probe()) 
   {
    while(1);
   }
  // reset SPS30 connection
  if (!sps30.reset()){
    
    while(1);
  }
  // Write some device info to serial
  char serialNumber[32];
  sps30.GetSerialNumber(serialNumber,32);
  Serial.println("Device ID: "+deviceID+"\tSPS30 Serial: "+String(serialNumber));
  metaString = deviceID + "_"+String(serialNumber);
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

  // start measurement
  sps30.start();

  // wait a moment until the system stablize
  //Clean for 15 seconds
  if (PERFORMCLEANNOW) {
  // clean now
    if (sps30.clean() == true) {
      Serial.println(F("fan-cleaning started"));
    }
    else {
      Serial.println(F("Could NOT start fan-cleaning"));
    }
    delay(30000);
  }
  
  Serial.println(F("Measurements starting..."));
  
  digitalWrite(ledposition, HIGH);
  delay(200);
  digitalWrite(ledposition, LOW);
  delay(200); 
  digitalWrite(ledposition, HIGH);
  delay(200);
  digitalWrite(ledposition, LOW);
    
  delay(10);
}

void loop() 
{
  if (previousMillis == 0) previousMillis = millis();
  
  sps30.GetValues(&val);

  // Building the average
  if (count == 1) {
    valAvg = val;
    count++;
  }
  else {
    valAvg.MassPM1 = (valAvg.MassPM1 * (count - 1) + val.MassPM1) / count;
    valAvg.MassPM2 = (valAvg.MassPM2 * (count - 1) + val.MassPM2) / count;
    valAvg.MassPM4 = (valAvg.MassPM4 * (count - 1) + val.MassPM4) / count;
    valAvg.MassPM10 = (valAvg.MassPM10*(count - 1) + val.MassPM10)/ count;
    valAvg.NumPM1 = (valAvg.NumPM1 * (count - 1) + val.NumPM1) / count;
    valAvg.NumPM2 = (valAvg.NumPM2 * (count - 1) + val.NumPM2) / count;
    valAvg.NumPM4 = (valAvg.NumPM4 * (count - 1) + val.NumPM4) / count;
    valAvg.NumPM10 = (valAvg.NumPM10 * (count - 1) + val.NumPM10) / count;
    valAvg.PartSize = (valAvg.PartSize * (count - 1) + val.PartSize) / count;
    count++;
  }

  // If it is time to write the minute average
  if (millis() - previousMillis > interval)
  {
    previousMillis = millis();
    count = 0;
    DateTime now = rtc.now();

    long pm1 = static_cast<long>(floor(valAvg.MassPM1+0.5));
    long pm2 = static_cast<long>(floor(valAvg.MassPM2+0.5));
    long pm4 = static_cast<long>(floor(valAvg.MassPM4+0.5));
    long pm10 = static_cast<long>(floor(valAvg.MassPM10+0.5));
    long num1 = static_cast<long>(floor(valAvg.NumPM1+0.5));
    long num2 = static_cast<long>(floor(valAvg.NumPM2+0.5));
    long num4 = static_cast<long>(floor(valAvg.NumPM4+0.5));
    long num10 = static_cast<long>(floor(valAvg.NumPM10+0.5));

    float temp = bmeSensor.readTempC();
    int humidity = static_cast<int>(round(bmeSensor.readFloatHumidity()));
    long int pressure = static_cast<long int>(round(bmeSensor.readFloatPressure()));

    month_cov = String(now.month());
    if(now.month() <10) month_cov = "0"+ month_cov;
    data_filename = deviceID + "_" + String(now.year())+month_cov+".txt";

    pmString = String(pm1)+";"+String(pm2)+";"+String(pm4)+";"+String(pm10)+';'+String(num1)+';'+String(num2)+';'+String(num4)+';'+String(num10)+';'+String(valAvg.PartSize);
    bmeString = String(temp)+";"+String(humidity)+";"+String(pressure);
    data = String(String(counter)+';'+now.timestamp(DateTime::TIMESTAMP_FULL))+";"+pmString+";"+bmeString+";"+metaString;
    serialdata = String(String(counter)+'\t'+now.timestamp(DateTime::TIMESTAMP_FULL))+'\t'+pmString+'\t'+bmeString;
    Serial.println(serialdata);

    if (sdFailure) {  //Try to reinitialize SD card
      if (!SD.begin(SD_CS_PIN)) {
        Serial.println("reinitialization failed!");
      }
      else {
        sdFailure = false;
      }
    }

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
      delay(500);
      digitalWrite(ledposition, HIGH);
      delay(500);
      digitalWrite(ledposition, LOW);            
    } else {
      // if the file didn't open, print an error and blink 10 times:
      Serial.println("error opening datafile");
      sdFailure = true;
      for(int i=0;i<10;i++){
        digitalWrite(ledposition, HIGH);
        delay(200);
        digitalWrite(ledposition, LOW);
        delay(200);  
      }      
    }
    metaString = "";
    ++counter;
  }
  
  
  digitalWrite(ledposition, HIGH);
  delay(1000);
  digitalWrite(ledposition, LOW);
  delay(7500);
}
