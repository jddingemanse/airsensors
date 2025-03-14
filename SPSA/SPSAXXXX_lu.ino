// SPSA: SPS30 Sensirion Arduino sensor
/// V03_lu: Special code for Lund Lab testing: continuous screen inclusion, no low-power start (since no lithium battery)
/// Change versus V01: added numberconcentration 0.5
/// Change versus V02: added cleaning every 9000 counters, and switched to one reading per minute (instead of average)

// ######### SET THE DEVICEID ####################
String deviceID = "SPSAXXXX";

//taking 1 minute measurements
/*
 * SPS030......Mega //sketch parts from sps30 Basic Readings example
 * 1 VCC.......5V
 * 2 SDA.......SDA (nb: with 10k pull up resistor, or other I2C sensor)
 * 3 SCL.......SCL (nb: with 10k pull up resistor, or other I2C sensor)
 * 4 Select....GND (select I2c)
 * 5 GND.......GND
 *
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
 * SSD1306.....Mega //sketch parts from Adafruit SSD1306
 * GND.........GND
 * VCC.........5V
 * SDA.........SDA
 * SCL.........SCL
 *
 * Green LED....MEGA
 * long.........8
 * Short..220R..GND
 */

String sensors = "SPS30";
String softwareVersion = "V_03lu";

#include <SPI.h>
#include "SdFat.h"
#include <Wire.h>
#include "RTClib.h"
#include "sps30.h"
#include <LowPower.h>
#include "SparkFunBME280.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

SPS30 sps30;
SdFat SD;
RTC_DS3231 rtc;
BME280 bmeSensor;

int ledposition = 8;
int screenposition = 9;
#define SD_CS_PIN SS
#define SP30_COMMS I2C_COMMS
#define DEBUG 0
#define PERFORMCLEANNOW 1 //SPS030 cleaning
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };

File metaFile;
String meta_filename = "metadata_"+deviceID+".txt";
String metaheader = "datetime;software;deviceID;spsSerial;sensors";
File dataFile;
String data_filename= "";
String month_cov = "";
String dataheader = "counter;datetime;PM1;PM2_5;PM4;PM10;NumPM0_5;NumPM1;NumPM2_5;NumPM4;NumPM10;PartSize;tempC;humidity;pressure;meta";
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
bool bmeFailure = false;
long next_clean_count = 9000;

long counter = 0; // For every restart, a counter is included

void setup() {
  // put your setup code here, to run once:
  pinMode(ledposition, OUTPUT);
  pinMode(screenposition, OUTPUT);
  digitalWrite(screenposition, HIGH); // Give power to the screen during setup.
  Serial.begin(9600);
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  Serial.println("Device ID: "+deviceID);
  Serial.print("Initializing RTC module...");
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  DateTime now = rtc.now();
  Serial.println("Current time according to RTC: "+String(now.timestamp(DateTime::TIMESTAMP_FULL)));

  screentext(deviceID.c_str());
  addscreentext(String(now.timestamp(DateTime::TIMESTAMP_FULL)).c_str());
  addscreentext("Starting sequence...");

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
    bmeFailure = true;
    //while(1); //Freeze
  }

  Serial.println("initialization done.");
// set driver debug level
  sps30.EnableDebugging(DEBUG);
  // Begin communication channel TO sp30;
  if (!sps30.begin(SP30_COMMS)) {
   Serial.println("SPS connection problem 1.");
   while(1);
  }
  
  // check for SPS30 connection
 if (!sps30.probe()) {
    Serial.println("SPS connection problem 2.");
    while(1);
  }
  // reset SPS30 connection
  if (!sps30.reset()){
    Serial.println("SPS connection problem 3.");
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
  
  if (counter > next_clean_count) {
    next_clean_count = next_clean_count + 9000;
    if (sps30.clean() == true) {
      Serial.println(F("fan-cleaning started"));
    }
    else {
      Serial.println(F("Could NOT start fan-cleaning"));
    }
    delay(30000);

  }

  // Try to connect the screen
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  // If it is time to write the minute average
  if (millis() - previousMillis > interval)
  {
    previousMillis = millis();
    DateTime now = rtc.now();

    sps30.GetValues(&val);

    long pm1 = static_cast<long>(floor(val.MassPM1+0.5));
    long pm2 = static_cast<long>(floor(val.MassPM2+0.5));
    long pm4 = static_cast<long>(floor(val.MassPM4+0.5));
    long pm10 = static_cast<long>(floor(val.MassPM10+0.5));
    long num0 = static_cast<long>(floor(val.NumPM0+0.5));
    long num1 = static_cast<long>(floor(val.NumPM1+0.5));
    long num2 = static_cast<long>(floor(val.NumPM2+0.5));
    long num4 = static_cast<long>(floor(val.NumPM4+0.5));
    long num10 = static_cast<long>(floor(val.NumPM10+0.5));

    float temp = bmeSensor.readTempC();
    int humidity = static_cast<int>(round(bmeSensor.readFloatHumidity()));
    long int pressure = static_cast<long int>(round(bmeSensor.readFloatPressure()));

    month_cov = String(now.month());
    if(now.month() <10) month_cov = "0"+ month_cov;
    data_filename = deviceID + "_" + String(now.year())+month_cov+".txt";

    pmString = String(pm1)+";"+String(pm2)+";"+String(pm4)+";"+String(pm10)+';'+String(num0)+';'+String(num1)+';'+String(num2)+';'+String(num4)+';'+String(num10)+';'+String(val.PartSize);
    bmeString = String(temp)+";"+String(humidity)+";"+String(pressure);
    if(bmeFailure) bmeString = ";;";
    
    Serial.print(String(String(counter)+'\t'+now.timestamp(DateTime::TIMESTAMP_FULL))+'\t');
    Serial.print(pmString + '\t');
    Serial.println(bmeString);

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
      screentext("Writing to SD");
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
      screentext("SD error");
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
    screentext("");
  }

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  
  sps30.GetValues(&val);
  
  DateTime now = rtc.now();
  screentext(deviceID.c_str());
  addscreentext(String(now.timestamp(DateTime::TIMESTAMP_FULL)).c_str());
  String pmtext = "PM2.5: " + String(val.MassPM2);
  addscreentext(pmtext.c_str());
  if (bmeFailure) {
    String bmefailuretext = "BME failure";
    addscreentext(bmefailuretext.c_str());
  }
  else {
    String bmetext = "T;RH;P " + bmeString;
    addscreentext(bmetext.c_str());
  }
  if (sdFailure) {
    String sdfailuretext = "SD failure";
    addscreentext(sdfailuretext.c_str());
  }

  digitalWrite(ledposition, HIGH);
  delay(1000);
  digitalWrite(ledposition, LOW);
  delay(7500);
  screentext("");
}

void screentext(const char* input) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println((input));
  display.display();
}

void addscreentext(const char* input) {
  display.println(input);
  display.display();
}
