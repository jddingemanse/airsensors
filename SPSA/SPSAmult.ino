/************************************************************************************
 * SPSAmult: 3 SPSA sensors connected to one Arduino Mega board
 * Taking a point measurement every minute
 */

//// User settings ////
String deviceID = "SPSAmultXXXX"; //Set deviceID

//// Builder settings ////
String softwareVersion = "V_01";

// CONNECTION INSTRUCTIONS: THREE SPS30 (SPS30A, SPS30B, SPS30C),
//   RTC DS3231, SD MODULE, 0.96IN SCREEN, LED, PUSHBUTTON
/*
 * SPS30A......Mega 
 * 1 VCC.......5V
 * 2 RX........TX1 (PIN 18) 
 * 3 TX........RX1 (PIN 19)
 * 4 Select....DONT CONNECT (select UART communication)
 * 5 GND.......GND
 *
 * SPS30A......Mega 
 * 1 VCC.......5V
 * 2 RX........TX2 (PIN 16) 
 * 3 TX........RX2 (PIN 17)
 * 4 Select....DONT CONNECT (select UART communication)
 * 5 GND.......GND
 *
 * SPS30A......Mega 
 * 1 VCC.......5V
 * 2 RX........TX3 (PIN 14) 
 * 3 TX........RX3 (PIN 15)
 * 4 Select....DONT CONNECT (select UART communication)
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
 * long...220R..8 //Somewhere in the circuit there should be a 220ohm resistor
 * Short........GND
 * 
 * Pushbutton..MEGA
 * Leg1........5V // It does not matter which of the button-legs is leg1 or leg2
 * Leg2........9
 * Leg2..>1kR..GND // Leg connected to pin 9 should ALSO over a resistor (1Kohm or higher) be connected to GND
 */


#include <SPI.h>
#include "SdFat.h"
#include <Wire.h>
#include "RTClib.h"

#include "sps30.h"
#include <LowPower.h>
#include "SparkFunBME280.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/////////////////////////////////////////////////////////////
#define SP30_COMMSA SERIALPORT1
#define SP30_COMMSB SERIALPORT2
#define SP30_COMMSC SERIALPORT3
#define TX_A 18
#define RX_A 19
#define TX_B 16
#define RX_B 17
#define TX_C 14
#define RX_C 15
#define DEBUG 0
#define PERFORMCLEANNOW 1

#define SD_CS_PIN SS

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

// create constructor
SPS30 sps30A;
SPS30 sps30B;
SPS30 sps30C;
SdFat SD;
RTC_DS3231 rtc;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// create variables
int ledposition = 8;
const int buttonPin = 9; // pushbutton
int buttonState = 0;
int screenTimes = 7;

void ErrtoMess(char *mess, uint8_t r, uint8_t s);
void Errorloop(char *mess, uint8_t r, uint8_t s);

struct sps_values valA;
struct sps_values valB;
struct sps_values valC;
String pmStringA = "";
String pmStringB = "";
String pmStringC = "";

File metaFile;
String meta_filename = "metadata_" + deviceID + ".txt";
String metaheader = "datetime;software;deviceID;spsSerialA;spsSerialB;spsSerialC";
File dataFile;
String data_filename = "";
String month_cov = "";
String dataheader = "sensor;counter;datetime;PM1;PM2_5;PM4;PM10;NumPM0;NumPM1;NumPM2_5;NumPM4;NumPM10;PartSize;meta";
String dataA = "";
String dataB = "";
String dataC = "";
String serialdata = "";
String metaStringA = "";
String metaStringB = "";
String metaStringC = "";
int count = 1; //counter to keep track of minute averages
unsigned long previousMillis = 0;
unsigned long interval = 60000UL;
bool sdFailure = false; //variable to keep track of sdFailure. If 1: try to reinitialize during loop. 

long counter = 0; // For every restart, a counter is included

void setup() {

  pinMode(ledposition, OUTPUT);
  pinMode(buttonPin, INPUT); // Set the button as input
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

  Serial.println("initialization done.");

  // set driver debug level
  sps30A.EnableDebugging(DEBUG);
  sps30B.EnableDebugging(DEBUG);
  sps30C.EnableDebugging(DEBUG);

  // set pins
  if (TX_A != 0 && RX_A != 0) sps30A.SetSerialPin(RX_A,TX_A);
  if (TX_B != 0 && RX_B != 0) sps30B.SetSerialPin(RX_B,TX_B);
  if (TX_C != 0 && RX_C != 0) sps30C.SetSerialPin(RX_C,TX_C);
  

  // Begin communication channel;
  if (sps30A.begin(SP30_COMMSA) == false) {
    Errorloop("could not initialize communication channel A.", 0, 1);
  }
  if (sps30B.begin(SP30_COMMSB) == false) {
    Errorloop("could not initialize communication channel B.", 0, 2);
  }
  if (sps30C.begin(SP30_COMMSC) == false) {
    Errorloop("could not initialize communication channel C.", 0, 3);
  }

  // check for SPS30 connection
  if (sps30A.probe() == false) Errorloop("could not probe / connect with SPS30A.", 0, 1);
  else Serial.println(F("Detected SPS30 A."));
  if (sps30B.probe() == false) Errorloop("could not probe / connect with SPS30B.", 0, 2);
  else Serial.println(F("Detected SPS30B."));
  if (sps30C.probe() == false) Errorloop("could not probe / connect with SPS30C.", 0, 3);
  else Serial.println(F("Detected SPS30C."));


  // reset SPS30 connection
  if (sps30A.reset() == false) Errorloop("could not reset A.", 0, 1);
  if (sps30B.reset() == false) Errorloop("could not reset B.", 0, 2);
  if (sps30C.reset() == false) Errorloop("could not reset C.", 0, 3);

  // read device info
  char serialNumberA[32];
  sps30A.GetSerialNumber(serialNumberA,32);
  char serialNumberB[32];
  sps30B.GetSerialNumber(serialNumberB,32);
  char serialNumberC[32];
  sps30C.GetSerialNumber(serialNumberC,32);
  Serial.println("Device ID: " + deviceID +"\tSPS30 Serial numbers: "+String(serialNumberA)+", "+String(serialNumberB)+", "+String(serialNumberC));
  metaStringA = deviceID + "_" + String(serialNumberA);
  metaStringB = deviceID + "_" + String(serialNumberB);
  metaStringC = deviceID + "_" + String(serialNumberC);
  
  // Write metadata to SD card
  bool metafile_exists = SD.exists(meta_filename);
  metaFile = SD.open(meta_filename, FILE_WRITE);
  // if the file opened okay, write to it:
  if (metaFile) {
    if(!metafile_exists) {
      metaFile.println(metaheader);
    }
    String metadata = String(now.timestamp(DateTime::TIMESTAMP_FULL))+';' + softwareVersion + ';' + deviceID + ';' + String(serialNumberA) + ';' + String(serialNumberB) + ';' + String(serialNumberC);
    metaFile.println(metadata);
    // close the file:
    metaFile.close();          
  } else {
    // if the file didn't open, print an error
    Serial.println("error opening metadata file");      
  }   

  // start measurement
  if (sps30A.start() == true) Serial.println(F("Measurement started A"));
  else Errorloop("Could NOT start measurement A", 0, 1);
  if (sps30B.start() == true) Serial.println(F("Measurement started B"));
  else Errorloop("Could NOT start measurement B", 0, 2);
  if (sps30C.start() == true) Serial.println(F("Measurement started C"));
  else Errorloop("Could NOT start measurement C", 0, 3);

  if (PERFORMCLEANNOW) {
  // clean now
    if (sps30A.clean() == true) Serial.println(F("fan-cleaning started A"));
    else Serial.println(F("Could NOT start fan-cleaning A"));
    if (sps30B.clean() == true) Serial.println(F("fan-cleaning started B"));
    else Serial.println(F("Could NOT start fan-cleaning B"));
    if (sps30C.clean() == true) Serial.println(F("fan-cleaning started C"));
    else Serial.println(F("Could NOT start fan-cleaning C"));

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

void loop() {
  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH) screenTimes = 7;
  
  if (previousMillis == 0) previousMillis = millis();

  // Take only first time a reading for the screen
  if (counter == 0 && screenTimes == 7) {
    sps30A.GetValues(&valA);
    sps30B.GetValues(&valB);
    sps30C.GetValues(&valC);
  }

  // If it is time to take the minute measurement
  if (millis() - previousMillis > interval)
  {
    previousMillis = millis();
    DateTime now = rtc.now();
    
    sps30A.GetValues(&valA);
    sps30B.GetValues(&valB);
    sps30C.GetValues(&valC);

    pmStringA = String(String(valA.MassPM1)+";"+String(valA.MassPM2)+";"+String(valA.MassPM4)+";"+String(valA.MassPM10)+';'+String(valA.NumPM0)+';'+String(valA.NumPM1)+';'+String(valA.NumPM2)+';'+String(valA.NumPM4)+';'+String(valA.NumPM10)+';'+String(valA.PartSize));
    pmStringB = String(String(valB.MassPM1)+";"+String(valB.MassPM2)+";"+String(valB.MassPM4)+";"+String(valB.MassPM10)+';'+String(valB.NumPM0)+';'+String(valB.NumPM1)+';'+String(valB.NumPM2)+';'+String(valB.NumPM4)+';'+String(valB.NumPM10)+';'+String(valB.PartSize));
    pmStringC = String(String(valC.MassPM1)+";"+String(valC.MassPM2)+";"+String(valC.MassPM4)+";"+String(valC.MassPM10)+';'+String(valC.NumPM0)+';'+String(valC.NumPM1)+';'+String(valC.NumPM2)+';'+String(valC.NumPM4)+';'+String(valC.NumPM10)+';'+String(valC.PartSize));
  
    month_cov = String(now.month());
    if(now.month() <10) month_cov = "0"+ month_cov;
    data_filename = String(deviceID + "_" + String(now.year()) + month_cov + ".txt");

    dataA = String("A;" + String(counter) + ";" + now.timestamp(DateTime::TIMESTAMP_FULL) + ";" + pmStringA + ";" + metaStringA);
    dataB = String("B;" + String(counter) + ";" + now.timestamp(DateTime::TIMESTAMP_FULL) + ";" + pmStringB + ";" + metaStringB);
    dataC = String("C;" + String(counter) + ";" + now.timestamp(DateTime::TIMESTAMP_FULL) + ";" + pmStringC + ";" + metaStringC);

    serialdata = String("A " + String(counter)+'\t'+now.timestamp(DateTime::TIMESTAMP_FULL)+'\t'+pmStringA + "\n" + "B " + String(counter)+'\t'+now.timestamp(DateTime::TIMESTAMP_FULL)+'\t'+pmStringB + "\n" + "C " + String(counter)+'\t'+now.timestamp(DateTime::TIMESTAMP_FULL)+'\t'+pmStringC);
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
      if (screenTimes > 0) screentext("Writing to SD");
      if(!datafile_exists) {
        dataFile.println(dataheader);
      }

      dataFile.println(dataA);
      dataFile.println(dataB);
      dataFile.println(dataC);
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
      if (screenTimes > 0) screentext("SD error");
      Serial.println("error opening datafile");
      sdFailure = true;
      for(int i=0;i<10;i++){
        digitalWrite(ledposition, HIGH);
        delay(200);
        digitalWrite(ledposition, LOW);
        delay(200);  
      }  
    }
    metaStringA = "";
    metaStringB = "";
    metaStringC = "";
    ++counter;
    if (screenTimes > 0) screentext("");
  }
  
  if (screenTimes > 0) {
    DateTime now = rtc.now();
    screentext(deviceID.c_str());
    addscreentext(String(now.timestamp(DateTime::TIMESTAMP_FULL)).c_str());
    String pmtextA = "A PM2.5: " + String(valA.MassPM2);
    addscreentext(pmtextA.c_str());
    String pmtextB = "B PM2.5: " + String(valB.MassPM2);
    addscreentext(pmtextB.c_str());
    String pmtextC = "C PM2.5: " + String(valC.MassPM2);
    addscreentext(pmtextC.c_str());
    if (sdFailure) {
      String sdfailuretext = "SD failure";
      addscreentext(sdfailuretext.c_str());
    }
    String screenTimesInfo = "Screen on " + String(screenTimes) + ".";
    addscreentext(screenTimesInfo.c_str());
  }
  

  digitalWrite(ledposition, HIGH);
  delay(1000);
  digitalWrite(ledposition, LOW);
  delay(7500);
  if (screenTimes > 0) {
    screenTimes--;
    screentext("");
  }
}

/**
 *  @brief : continued loop after fatal error
 *  @param mess : message to display
 *  @param r : error code
 *  @param s : sensor (A: 1, B: 2, C: 3)
 *  if r is zero, it will only display the message
 */
void Errorloop(char *mess, uint8_t r, uint8_t s)
{
  if (r) ErrtoMess(mess, r, s);
  else Serial.println(mess);
  Serial.println(F("Program on hold"));
  for(;;) delay(100000);
}

/**
 *  @brief : display error message
 *  @param mess : message to display
 *  @param r : error code
 *  @param s : sensor (A: 1, B: 2, C: 3)
 */
void ErrtoMess(char *mess, uint8_t r, uint8_t s)
{
  char buf[80];

  Serial.print(mess);

  if (s==1) sps30A.GetErrDescription(r, buf, 80);
  if (s==2) sps30B.GetErrDescription(r, buf, 80);
  if (s==3) sps30C.GetErrDescription(r, buf, 80);
  Serial.println(buf);
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
