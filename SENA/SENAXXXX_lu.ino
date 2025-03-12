// SENA: SEN55 Sensirion Arduino sensor
/// V03_lu: Special code for Lund Lab testing: continuous screen inclusion, 1 minute measurements
/// V02: included number concentrations and fan cleaning
/// V03: single reading per minute, fan cleaning every 9000

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

#include <SPI.h>
#include "SdFat.h"
#include <Wire.h>
#include "RTClib.h"
#include <Arduino.h>
#include <SensirionI2CSen5x.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ######### SET THE DEVICEID ####################
String deviceID = "SENAXXXX";
String softwareVersion = "V_03lu";
String sensors = "SEN55";
int ledposition = 8;

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
String metaheader = "datetime;software;deviceID;SEN55serial;sensors";
File dataFile;
String data_filename= "";
String month_cov = "";
String dataheader = "counter;datetime;PM1;PM2_5;PM4;PM10;NumPM05;NumPM1;NumPM2_5;NumPM4;NumPM10;PartSize;tempC;humidity;VOC;NOX";
String data ="";
String serialdata = "";
String pmString = "";
String otherString = "";
String metaString = "";
int count = 1; //counter to keep track of minute averages
unsigned long previousMillis = 0;
unsigned long interval = 60000UL;
bool sdFailure = false; //variable to keep track of sdFailure. If 1: try to reinitialize during loop.
long next_clean_count = 9000;

float pm1;
float pm25;
float pm4;
float pm10;
float n05;
float n1;
float n25;
float n4;
float n10;
float ps; 
float RH;
float T;
float VOC;
float NOX;
float apm1;
float apm25;
float apm4;
float apm10;
float an05;
float an1;
float an25;
float an4;
float an10;
float aps; 

long counter = 0; // For every restart, a counter is included


void setup() {
  // put your setup code here, to run once:
  pinMode(ledposition,OUTPUT);
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

  error = sen5x.startMeasurement();
  if (error) {
      Serial.print("Error trying to execute startMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  }

  //Fan cleaning
  error = sen5x.startFanCleaning();
  if (error) {
    Serial.print("Error trying to start fan cleaning: ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  else {
    Serial.println("Fan cleaning.");
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
  delay(200);
  digitalWrite(ledposition, HIGH);
  delay(200);
  digitalWrite(ledposition, LOW);
     
  delay(10000);

}

void loop() {
  if (previousMillis == 0) previousMillis = millis();
  uint16_t error;
  char errorMessage[256];

  if (counter > next_clean_count) {
    next_clean_count = next_clean_count + 9000;
    error = sen5x.startFanCleaning();
    if (error) {
      Serial.print("Error trying to start fan cleaning: ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
    }
    else {
      Serial.println("Fan cleaning.");
      delay(30000);
    }
  }

  // Get the data
  sen5x.readMeasuredValues(
    pm1, pm25, pm4, pm10, RH, T, VOC, NOX);
  sen5x.readMeasuredPmValues(
    pm1, pm25, pm4, pm10, n05, n1, n25, n4, n10, ps);

  // Try to connect the screen
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  // If it is time to write the minute average
  if (millis() - previousMillis > interval)
  {
    previousMillis = millis();
    count = 0;
    DateTime now = rtc.now();

    long wpm1 = static_cast<long>(floor(pm1+0.5));
    long wpm25 = static_cast<long>(floor(pm25+0.5));
    long wpm4 = static_cast<long>(floor(pm4+0.5));
    long wpm10 = static_cast<long>(floor(pm10+0.5));
    long wn05 = static_cast<long>(floor(n05+0.5));
    long wn1 = static_cast<long>(floor(n1+0.5));
    long wn25 = static_cast<long>(floor(n25+0.5));
    long wn4 = static_cast<long>(floor(n4+0.5));
    long wn10 = static_cast<long>(floor(n10+0.5));

    month_cov = String(now.month());
    if(now.month() <10) month_cov = "0"+ month_cov;
    data_filename = deviceID + "_" + String(now.year())+month_cov+".txt";

    pmString = String(wpm1)+";"+String(wpm25)+";"+String(wpm4)+";"+String(wpm10)+';'+String(wn05)+';'+String(wn1)+';'+String(wn25)+';'+String(wn4)+';'+String(wn10)+';'+String(ps);
    otherString = String(T)+";"+String(RH)+";"+String(VOC)+";"+String(NOX);

    data = String(String(counter)+';'+now.timestamp(DateTime::TIMESTAMP_FULL))+";"+pmString+";"+otherString+";"+metaString;
    serialdata = String(String(counter)+'\t'+now.timestamp(DateTime::TIMESTAMP_FULL))+'\t'+pmString+'\t'+otherString;
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
  
  DateTime now = rtc.now();
  screentext(deviceID.c_str());
  addscreentext(String(now.timestamp(DateTime::TIMESTAMP_FULL)).c_str());
  String pmtext = "PM2.5: " + String(pm25);
  addscreentext(pmtext.c_str());
  String trhtext = "T;RH: " + String(T) + ";" + String(RH);
  addscreentext(trhtext.c_str());
  String gastext = "VOC;NOX: " + String(VOC) + ";" + String(NOX);
  addscreentext(gastext.c_str());
  if (sdFailure) {
    String sdfailuretext = "SD failure";
    addscreentext(sdfailuretext.c_str());
  }

  digitalWrite(ledposition, HIGH);
  delay(1000);
  digitalWrite(ledposition, LOW);
  delay(7500);
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
