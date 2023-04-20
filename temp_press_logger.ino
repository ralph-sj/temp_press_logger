#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include "Adafruit_MCP9600.h"

#define LOG_INTERVAL  100 // mills between entries (reduce to take more/faster data)
#define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL  1
#define WAIT_TO_START   0 
#define PRINT_TO_FILE   0 
#define SET_TIME        1
#define SET_TIME        1
#define READ_TEMPERATURE 0 

#define TC_ADDRESS_1 (0x60)
#define TC_ADDRESS_2 (0x67)

// the digital pins that connect to the LEDs
#define redLEDpin 2
#define greenLEDpin 3

// The analog pins that connect to the sensors
#define photocellPin 0           // analog 0
#define tempPin 1                // analog 1
#define BANDGAPREF 14            // special indicator that we want to measure the bandgap

#define aref_voltage 3.3         // we tie 3.3V to ARef and measure it with a multimeter!
#define bandgap_voltage 1.1      // this is not super guaranteed but its not -too- off

RTC_PCF8523 RTC; // define the Real Time Clock object
Adafruit_MCP9600 mcp;

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// the logging file
File logfile;
float temp_cold = 0;
float temp_hot =0;

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);

  while(1);
}

void setup(void)
{

  Serial.begin(9600);
  Serial.println();

  if (! RTC.begin()) 
  {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }
  
  #if SET_TIME
    RTC.adjust(DateTime(2023, 4, 20, 13, 41, 0));
  #endif //SET_TIME
  RTC.start();

  // use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  
  #if WAIT_TO_START
    Serial.println("Type any character to start");
    while (!Serial.available());
  #endif //WAIT_TO_START

  #if PRINT_TO_FILE
    // initialize the SD card
    Serial.print("Initializing SD card...");
    // make sure that the default chip select pin is set to
    // output, even if you don't use it:
    pinMode(10, OUTPUT);
  
    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
      error("Card failed, or not present");
    }
    Serial.println("card initialized.");
  
    // create a new file
    char filename[] = "LOGGER00.CSV";
    for (uint8_t i = 0; i < 100; i++) {
      filename[6] = i/10 + '0';
      filename[7] = i%10 + '0';
      if (! SD.exists(filename)) {
        // only open a new file if it doesn't exist
        logfile = SD.open(filename, FILE_WRITE); 
        break;  // leave the loop!
      }
    }
  
    if (! logfile) {
      error("couldnt create file");
    }

    Serial.print("Logging to: ");
    Serial.println(filename);
  #else
    Serial.println("WARNING: NOT PRINTING TO FILE");
    delay(1000);
  #endif //PRINT_TO_FILE

  // connect to RTC
  Wire.begin();  
  if (!RTC.begin()) {
    logfile.println("RTC failed");
    #if ECHO_TO_SERIAL
      Serial.println("RTC failed");
    #endif  //ECHO_TO_SERIAL
  } 

  logfile.println("datetime,temp_cold, temp_hot");    
  #if ECHO_TO_SERIAL
    Serial.println("datetime,temp_cold, temp_hot");
  #endif //ECHO_TO_SERIAL
 
  // If you want to set the aref to something other than 5v
    analogReference(EXTERNAL);

  // Thermocouple
  #if READ_TEMPERATURE
    Serial.println("MCP9600 HW test");
      
    // TC1
    /* Initialise the driver with I2C_ADDRESS and the default I2C bus. */
    if (! mcp.begin(TC_ADDRESS_1)) {
        Serial.println("Sensor 1 not found. Check wiring!");
        while (1);
    }
    mcp.setADCresolution(MCP9600_ADCRESOLUTION_18);
    Serial.print("ADC resolution set to ");
    switch (mcp.getADCresolution()) {
      case MCP9600_ADCRESOLUTION_18:   Serial.print("18"); break;
      case MCP9600_ADCRESOLUTION_16:   Serial.print("16"); break;
      case MCP9600_ADCRESOLUTION_14:   Serial.print("14"); break;
      case MCP9600_ADCRESOLUTION_12:   Serial.print("12"); break;
    }
    Serial.println(" bits");

    mcp.setThermocoupleType(MCP9600_TYPE_K);
    Serial.print("Thermocouple type set to ");
    switch (mcp.getThermocoupleType()) {
      case MCP9600_TYPE_K:  Serial.print("K"); break;
      case MCP9600_TYPE_J:  Serial.print("J"); break;
      case MCP9600_TYPE_T:  Serial.print("T"); break;
      case MCP9600_TYPE_N:  Serial.print("N"); break;
      case MCP9600_TYPE_S:  Serial.print("S"); break;
      case MCP9600_TYPE_E:  Serial.print("E"); break;
      case MCP9600_TYPE_B:  Serial.print("B"); break;
      case MCP9600_TYPE_R:  Serial.print("R"); break;
    }
    Serial.println(" type");

    mcp.setFilterCoefficient(3);
    Serial.print("Filter coefficient value set to: ");
    Serial.println(mcp.getFilterCoefficient());

    //  if (! mcp2.begin(TC_ADDRESS_2)) {
    //      Serial.println("Sensor 1 not found. Check wiring!");
    //      while (1);
    //  }
    //  mcp2.enable(false);

    Serial.println("Found MCP9600!");
  #endif //READ_TEMPERATURE
}

void loop(void){
  DateTime now;
  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
  digitalWrite(greenLEDpin, HIGH);

  // fetch the time
  now = RTC.now();
  #if READ_TEMPERATURE
    mcp.begin(TC_ADDRESS_1);
    temp_cold = mcp.readAmbient();
    temp_hot = mcp.readThermocouple();
  #endif //READ_TEMPERATURE
  // log time
  #if PRINT_TO_FILE    
    logfile.print(now.year(), DEC);
    logfile.print(now.month(), DEC);
    logfile.print(now.day(), DEC);
    logfile.print("_");
    logfile.print(now.hour(), DEC);
    logfile.print(now.minute(), DEC);
    logfile.print(now.second(), DEC);
    logfile.print(",");
    logfile.print(temp_cold);
    logfile.print(",");
    logfile.print(temp_hot);
    logfile.println();
  #endif //PRINT_TO_FILE
  #if ECHO_TO_SERIAL
    Serial.print(now.year(), DEC);
    if (now.month() < 10)
    {
      Serial.print("0");
    }
    Serial.print(now.month(), DEC);
    if (now.day() < 10)
    {
      Serial.print("0");
    }
    Serial.print(now.day(), DEC);
    Serial.print("_");
    if (now.hour() < 10)
    {
      Serial.print("0");
    }
    Serial.print(now.hour(), DEC);
    if (now.minute() < 10)
    {
      Serial.print("0");
    }
    Serial.print(now.minute(), DEC);
    if (now.second() < 10)
    {
      Serial.print("0");
    }
    Serial.print(now.second(), DEC);
    Serial.print(",");
    Serial.print(temp_cold);
    Serial.print(",");
    Serial.print(temp_hot);
    Serial.println();
  #endif //ECHO_TO_SERIAL


  digitalWrite(greenLEDpin, LOW);

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  
  // blink LED to show we are syncing data to the card & updating FAT!
  digitalWrite(redLEDpin, HIGH);
  logfile.flush();
  digitalWrite(redLEDpin, LOW);
  
}
