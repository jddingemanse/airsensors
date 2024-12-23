*This file provides Arduino code for SPSA, V0.5, with code explanations. It is
an exact copy of [SPSA](https://github.com/jddingemanse/airsensors/blob/main/SPSA/SPSAXXXX_1M.ino) V0.5 code, with in between edited text.
It is meant as an explanation of the different parts of the sketch - for easier
validation by others.*

# 1. General information
## 1.1 Hardware
This sketch version includes the following hardware:
- Arduino Mega 2560;
- SPS30 Sensirion sensor;
- BME280 RH/T sensor;
- Micro SD module (+ card);
- DS3231 real-time clock (RTC);
- SSD1306 0.96in OLED screen;
- Green LED.

## 1.2 Libraries
The SPSA code is mostly a combination of example sketches from the libraries
for the respective hardware components. The following libraries are used:
- [sps30](https://github.com/paulvha/sps30)
- [SparkFun_BME280](https://github.com/sparkfun/SparkFun_BME280_Arduino_Library)
- [SdFat](https://github.com/greiman/SdFat)
- [RTClib](https://github.com/adafruit/RTClib)
- [Adafruit_GFX](https://github.com/adafruit/Adafruit-GFX-Library) and [Adafruit_SSD1306](https://github.com/adafruit/Adafruit_SSD1306)

## 1.3 General sketch description
### 1.3.1 Start-up
The system starts with reading the time, and printing this to the serial monitor.
After that, it goes for approximately 50 seconds in lowpower mode. This is because
in the field the system is connected through a Lithium 18650 battery shield to power.
In case this battery is completely depleted, and power comes back, the lithium battery
is first charged a little bit before the system starts fully.
After that, the other components are initialized, and the SPS30 sensor takes 15 seconds
of cleaning mode, after which still 30 seconds of waiting is included. Then the green
LED blinks twice, and the measurement loop starts.
If the real-time clock, SD or SPS30 are not found, then the system does not start.
If the BME280 sensor or screen are not found, still the system starts.

At every successful start-up, a new line is printed to `metadata_[deviceID].txt`, that includes
the time, software version, deviceID, sps30 serial number, and the sensors on this system.

### 1.3.2 Measurement loop
During the measurement loop, every approximately 10 seconds the LED blinks once, SPS30 data is read,
and an average is recalculated of that reading and preceding values. If connected, the screen
data is also updated.
Every minute, the SPS30 average of preceding readings is written to SD, together with time, and
a point reading of the BME280. This data is also printed to the serial monitor. At the moment
of writing data to the SD, this is shown on the screen, and the green LED blinks two times.

If the SD is not found, at that moment the green LED blinks 10 times. Still, the measurement
loop continues (to make removing SD and putting it back while the system is on possible).

If connected, the screen prints:
- The time of the DS3231
- The PM2.5 value of the SPS30
- The T, RH and P values of the BME280 (if found), or `BME error` if not found
- `SD error` if the SD is not found

Both the SD and screen are reinitialized each loop, to make it possible to have them
temporarily disconnected and later connected again.

# 2. Sketch description
## 2.1 Before `setup()`
The only thing that needs to be changed system to system is the deviceID. This
is always 'SPSA' followed by four digits. Additionally, the LED pin can be set
(the LED position changed across different versions, but it is currently pin 8).
```
// SPSA: SPS30 Sensirion Arduino sensor. Script version 0.5.

// ######### SET THE DEVICEID ####################
String deviceID = "SPSA0013";

// ######### SET THE LED PIN ##################### -- 8 for [SPSA0002]
int ledposition = 8;
```

Inside the code, references and hardware connections are included. This shows
how each of the components need to be connected to the Mega.
For SPS30, BME280, DS3231 and the screen SSD1306, I2C (SDA/SCL) communication is used.
```
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
 * SSD1306.....Mega //sketch parts from Adafruit SSD1306
 * GND.........GND
 * VCC.........5V
 * SDA.........SDA
 * SCL.........SCL
 *
 * Green LED....Mega
 * +............8 [+: longer leg]
 * -..resistor..GND [-: smaller leg]
 */
```
Strings `sensors` and `softwareVersion` are used in the metadata. The string
`sensors` shows which sensors are part of this system.
```
String sensors = "SPS30,BME280";
String softwareVersion = "V05-60S";
```
Inclusion of all libraries:
```
#include <SPI.h>
#include "SdFat.h"
#include <Wire.h>
#include "RTClib.h"
#include "sps30.h"
#include <LowPower.h>
#include "SparkFunBME280.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
```
Creating constructors for the components: `sps30`, `SD`, `rtc`, `bmeSensor`
and `display`. Also, defining several settings.
```
SPS30 sps30;
SdFat SD;
RTC_DS3231 rtc;
BME280 bmeSensor;

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
```
Creating variables that will be used. *Explanation per variable behind `//` for each line*
```
File metaFile; //The file to which metadata is printed.
String meta_filename = "metadata_"+deviceID+".txt"; //The filename of the metadata file.
String metaheader = "datetime;software;deviceID;spsSerial;sensors"; //The header in the metadata file.
File dataFile; //The file to which data is printed.
String data_filename= ""; //To be used as the filename of the data file.
String month_cov = ""; //To be used as a two-digit month value (as part of the data file name).
String dataheader = "counter;datetime;PM1;PM2_5;PM4;PM10;NumPM0;NumPM1;NumPM2_5;NumPM4;NumPM10;PartSize;tempC;humidity;pressure;meta"; //The header in the data file.
String data =""; //To be used as data that will be printed in the SD file.
String serialdata = ""; //To be used as data that will be printed to the serial monitor.
struct sps_values val; //A structure for a single reading of the SPS30.
struct sps_values valAvg; //A structure for the averaged values of the SPS30.
String pmString = ""; //To be used for the SPS30 (PM) data part of the string 'data'
String bmeString = ""; //To be used for the BME280 data part of the string 'data'
String metaString = ""; //To be used as metadata (deviceID and serialnumber) that is included at the end of the datafile line every first dataline after start-up.
int count = 1; //counter to keep track of minute averages; to be used to construct the intermediate average.
unsigned long previousMillis = 0; //value to save the millisecond since start-up. Used to decide to take minute average.
unsigned long interval = 60000UL; //value giving the time interval after which the minute average needs to be taken.
bool sdFailure = false; //variable to keep track of sdFailure. If 1: try to reinitialize during loop.
bool bmeFailure = false; //variable to keep track of bme280 failure.

long counter = 0; // At the start of the data line, a counter is included, starting at 0 every restart, and counting up per minute average.
```

## 2.2 setup()
First, the LED pin is set, the serial monitor is started, and the current time is read and printed.
If for some reason the DS3231 is unresponsive, the system freezes.
```
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
```
The system goes into low-power, in case the Lithium battery or powerbank is depleted and it needs
some charge first. It goes into six cycles of 8-second sleepmode.
```
  Serial.println("Starting low-power to let battery charge...");
  delay(1000);
  for(int i=0;i<6;i++){
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER5_OFF, TIMER4_OFF, TIMER3_OFF, 
          TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART3_OFF, 
          USART2_OFF, USART1_OFF, USART0_OFF, TWI_OFF);  
  }
```
Initialization of SD card. if the SD card is not found, the setup restarts from top.
```
  Serial.print("Initializing SD card...");

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("initialization failed!");
    return;
  }

```
Start of BME280. The address of my sensors is `0x76`, while the default address
according to the library is `0x77`. Therefore, the address needs to be set.
If the BME280 is not found, still the system continues (`while(1)` is commented out).
This is because I find SPS30 values more important, and I want the system to continue
even if there is some BME280 wiring problem.
```
  Wire.begin();
  bmeSensor.setI2CAddress(0x76);
  if (bmeSensor.beginI2C() == false)
  {
    Serial.println("The BME280 sensor did not respond. Please check wiring.");
    bmeFailure = true;
    //while(1); //Freeze
  }

  Serial.println("initialization done.");
```
Start of the SPS30 sensor. If it is not found, the system freezes.
```
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
```
Collecting metadata, and writing it to the serial monitor and the file metadata_[deviceID].txt. 
To the serial monitor, the device ID and SPS30 serial number is printed.
For the file a single line is created (`metadata`), that contains the current time,
the softwareversion, deviceID, SPS30 serial number, and the sensors.
Furthermore, the string `metaString` is created (deviceID and SPS30 serial number);
that string will be added to the first data line.
```
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
```
The sps30 is started, and then first set to cleaning mode. 
Around cleaning, there is 30 seconds of waiting time to let
the SPS30 return to normal readings before starting the measurement
cycle.
```
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
```
To indicate end of setup (and imminent start of measurement cycle),
the green LED blinks twice for 200 miliseconds.
```
  digitalWrite(ledposition, HIGH);
  delay(200);
  digitalWrite(ledposition, LOW);
  delay(200); 
  digitalWrite(ledposition, HIGH);
  delay(200);
  digitalWrite(ledposition, LOW);
    
  delay(10);
}
```
## 2.3 loop()
If arrived here, the system will continuously loop through the following commands.
In the very first loop, `previousMillis` is set to the current milisecond value of 
the arduino (number of miliseconds since start of the system).
```
void loop() 
{
  if (previousMillis == 0) previousMillis = millis();
```
### 2.3.1 Every ten seconds
The sps30 values are read and stored in `val`.
```  
  sps30.GetValues(&val);
```
If this is the first reading of a new minute average (`count == 1`), `valAvg` is set
to the first reading (`val`). If not, a new average is calculated for each of the SPS30
values. After either of these, count is incremented with 1 (`count++`). 
For example, for the mass concentration of PM1 (`.MassPM1`), the new average is calculated by 
- taking the previous average (`valAvg.MassPM1`);
- multiplying it with the previous `count` (`count - 1`);
- adding the newest reading to it (`val.MassPM1`);
- dividing that all by the new `count`.
This new average overwrites the earlier average:
`valAvg.MassPM1 = (valAvg.MassPM1 * (count - 1) + val.MassPM1) / count;`

When count is 2 (the second cycle), and assuming the previous average PM1 mass concentration (`valAvg.MassPM1`) was
3 microgram/m3, and the newest reading (`val.MassPM1`) is 5 microgram/m3, the new average becomes:
`(3 * (2 - 1) + 5) / 2 = 4`. This is of course the average of 3 and 5. `valAvg.MassPM1` hence becomes 4.
The cycle after this, count is 3. If the newest reading is 7 microgram/m3, the new average becomes:
`(4 * (3 - 1) + 7) / 3 = 5`. This is the average of 3, 5 and 7.
```
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
    valAvg.NumPM0 = (valAvg.NumPM0 * (count - 1) + val.NumPM0) / count;
    valAvg.NumPM1 = (valAvg.NumPM1 * (count - 1) + val.NumPM1) / count;
    valAvg.NumPM2 = (valAvg.NumPM2 * (count - 1) + val.NumPM2) / count;
    valAvg.NumPM4 = (valAvg.NumPM4 * (count - 1) + val.NumPM4) / count;
    valAvg.NumPM10 = (valAvg.NumPM10 * (count - 1) + val.NumPM10) / count;
    valAvg.PartSize = (valAvg.PartSize * (count - 1) + val.PartSize) / count;
    count++;
  }
```
In case the screen was disconnected at some point, it restarts here.
```
  // Try to connect the screen
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

```
### 2.3.2 Once per (approximate) minute
If the current arduino milisecond reading is more than `previousMillis` + `interval`, it is
time for the minute average (since `interval` is set to 60000 miliseconds). At that moment,
`previousMillis` is updated to this new milisecond reading, and the variable `count` is reset to
0. *NB: while creating this read-through, I realize this might be a mistake: it might need to be
reset to 1 instead... It is only a small mistake: this will result in one moment where count is
0, and above valAvg is updated with this, resulting in bullshit values. However, as soon as
count becomes 1, valAvg is overwritten with the actual readings. Hence, the result of this error
is that out of 6-7 readings per minute, the first reading is ignored entirely. This is just as arbitrary
as the additional time not used every minute while writing to SD.*
```
  // If it is time to write the minute average
  if (millis() - previousMillis > interval)
  {
    previousMillis = millis();
    count = 0;
```
The current time is read from the RTC DS3231.
```
    DateTime now = rtc.now();
```
Long integer variables are created for the SPS30 values, based on the 
rounded float values of the running averages (`valAvg`).
```
    long pm1 = static_cast<long>(floor(valAvg.MassPM1+0.5));
    long pm2 = static_cast<long>(floor(valAvg.MassPM2+0.5));
    long pm4 = static_cast<long>(floor(valAvg.MassPM4+0.5));
    long pm10 = static_cast<long>(floor(valAvg.MassPM10+0.5));
    long num0 = static_cast<long>(floor(valAvg.NumPM0+0.5));
    long num1 = static_cast<long>(floor(valAvg.NumPM1+0.5));
    long num2 = static_cast<long>(floor(valAvg.NumPM2+0.5));
    long num4 = static_cast<long>(floor(valAvg.NumPM4+0.5));
    long num10 = static_cast<long>(floor(valAvg.NumPM10+0.5));
```
Temperature (as float), relative humidity (cast to integer) and pressure (cast to
long integer) are read from the BME280 sensor.
```
    float temp = bmeSensor.readTempC();
    int humidity = static_cast<int>(round(bmeSensor.readFloatHumidity()));
    long int pressure = static_cast<long int>(round(bmeSensor.readFloatPressure()));
```
The data filename is created. For this, the month value is taken (and turned into
two digits), in order to create a filename of `[deviceID]_[YYYYMM].txt`. In other words,
per month a new datafile is created. For the device `SPSA0021` in January 2024, the filename
would become `SPSA0021_202401.txt`.
```
    month_cov = String(now.month());
    if(now.month() <10) month_cov = "0"+ month_cov;
    data_filename = deviceID + "_" + String(now.year())+month_cov+".txt";
```
The `data` string is created, by first creating the substrings `pmString` (SPS30 data) and
`bmeString` (BME280 data), with values semi-colon separated (`x;y`). The `data` string will be
a single line of `counter` (starting at 0 per restart, counting up per minute average), date and time,
SPS30 data (PM1, PM2.5, PM4, PM10 mass concentrations, PM0.5, PM1, PM2.5, PM4, PM10 number concentrations,
size fraction), BME280 data (temperature, humidity, pressure), and metaString (deviceID + SPS30 serial number the first
line after startup, empty after that).
```
    pmString = String(pm1)+";"+String(pm2)+";"+String(pm4)+";"+String(pm10)+';'+String(num0)+';'+String(num1)+';'+String(num2)+';'+String(num4)+';'+String(num10)+';'+String(valAvg.PartSize);
    bmeString = String(temp)+";"+String(humidity)+";"+String(pressure);
    if(bmeFailure) bmeString = ";;";
    data = String(String(counter)+';'+now.timestamp(DateTime::TIMESTAMP_FULL))+";"+pmString+";"+bmeString+";"+metaString;
```
The string `serialdata` is created (combination of counter, time, pmString and bmeString), and printed to the serial
monitor.
```
    serialdata = String(String(counter)+'\t'+now.timestamp(DateTime::TIMESTAMP_FULL))+'\t'+pmString+'\t'+bmeString;
    Serial.println(serialdata);
```
In case the SD was missing an earlier cycle (sdFailure = true) and the SD card is back, the SD card is reinitialized.
```
    if (sdFailure) {  //Try to reinitialize SD card
      if (!SD.begin(SD_CS_PIN)) {
        Serial.println("reinitialization failed!");
      }
      else {
        sdFailure = false;
      }
    }
```
Creating a variable `datafile_exists`; if the file did not yet exist (first time for the instrument,
or every new month), this one will be `false` - in order to decide to include a header.
```
    // Decide whether headers need to be included    
    bool datafile_exists = SD.exists(data_filename);
```
Try to open the file. If it opened, and it was a new file (`datafile_exists == true`), the string 
`dataheader` is printed as new line. In any case, the string `data` is printed as new line.
After closing the file, the green LED blinks twice for 500 miliseconds.
If it did not open, this is printed to the screen and serial monitor, `sdFailure` is set to true,
and the green LED blinks ten times for 200 miliseconds.
```
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
```
At the end of the minute-average cycle, metaString is set empty (only the first data line includes
some metadata at the end of that line), and the minute-average counter is incremented.
```
    metaString = "";
    ++counter;
    screentext("");
  }

```
### 2.3.2 Back to every 10-seconds
In case the screen was missing, and it is connected now, it is restarted.
```
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
```
Some information is prepared to be put on the screen:
- time;
- SPS30 PM2.5 reading;
- BME280 T, RH and P reading (if available) or `BME failure`;
- if SD is not present, `SD failure`.
```
  DateTime now = rtc.now();
  screentext(String(now.timestamp(DateTime::TIMESTAMP_FULL)).c_str());
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
```
Finally, the green LED blinks for 1 second, and the system halts for 7.5 seconds,
to create with everything combined an approximate 10 second cycle.
```
  digitalWrite(ledposition, HIGH);
  delay(1000);
  digitalWrite(ledposition, LOW);
  delay(7500);
  screentext("");
}
```
## 2.4 Functions
The function `screentext()` is used to clear the screen, and write a first line.
```
void screentext(const char* input) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println((input));
  display.display();
}
```
The function `addscreentext()` is used to add a line to the screen (always to be used
after `screentext()`.
```
void addscreentext(const char* input) {
  display.println(input);
  display.display();
}
```
