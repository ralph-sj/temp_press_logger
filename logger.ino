#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include "Adafruit_MCP9600.h"

#define LOG_INTERVAL  100 // mills between entries (reduce to take more/faster data)
#define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync(
uint32_t m;
#define ECHO_TO_SERIAL      1 
#define WAIT_TO_START       0 
#define PRINT_TO_FILE       1 
#define SET_TIME            0 
#define READ_TEMPERATURE    1 
#define READ_PRESSURE       1 
#define TC_ADDRESS_1        (0x60)
#define TC_ADDRESS_2        (0x67)
#define ADC_bits            10// for analogRead
#define AdcRange            1024 // for analogRead

const int PressureScale = 20; //5V = 100bar
const int PressurePin1 = A0;
const int PressurePin2 = A1;
float PressureADC1 = 0;
float PressureADC2 = 0;
float PressureVolt1 = 0;
float PressureVolt2 = 0;
float Pressure1 = 0;
float Pressure2 = 0;

// the digital pins that connect to the LEDs
#define redLEDpin   4 
#define greenLEDpin 3
#define extLEDpin 13


RTC_PCF8523 RTC; // define the Real Time Clock object
Adafruit_MCP9600 mcp, mcp2;

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// the logging file
File logfile;
float temp_cold = 0;
float temp_hot =0;
float temp_cold2 = 0;
float temp_hot2 =0;
DateTime starttime; 


void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);
  digitalWrite(extLEDpin, HIGH);

  while(1);
}

void setup(void)
{

  Serial.begin(115200);
  Serial.println();

  if (! RTC.begin()) 
  {
    error("RTC failed");
  }
  
  #if WAIT_TO_START
    Serial.println("Type any character to start");
    while (!Serial.available());
  #endif //WAIT_TO_START
  
  #if SET_TIME
    RTC.adjust(DateTime(2023, 9, 01, 9, 42, 00)); // takes about 13  seconds to save the time
  //  RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("Date set.");
  #endif //SET_TIME
  RTC.start();
  
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  pinMode(extLEDpin, OUTPUT);
  

  starttime  = RTC.now();
  Serial.print("Logger started at ");
  Serial.print(starttime.year(), DEC);
  if (starttime.month() < 10)
  {
    Serial.print("0");
  }
  Serial.print(starttime.month(), DEC);
  if (starttime.day() < 10)
  {
    Serial.print("0");
  }
  Serial.print(starttime.day(), DEC);
  Serial.print("_");
  if (starttime.hour() < 10)
  {
    Serial.print("0");
  }
  Serial.print(starttime.hour(), DEC);
  if (starttime.minute() < 10)
  {
    Serial.print("0");
  }
  Serial.print(starttime.minute(), DEC);
  if (starttime.second() < 10)
  {
    Serial.print("0");
  }
  Serial.print(starttime.second(), DEC);
  Serial.println();

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
    logfile.println("card initialized.");
  
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

    logfile.print(starttime.year(), DEC);
    if (starttime.month() < 10)
    {
      logfile.print("0");
    }
    logfile.print(starttime.month(), DEC);
    if (starttime.day() < 10)
    {
      logfile.print("0");
    }
    logfile.print(starttime.day(), DEC);
    logfile.print("_");
    if (starttime.hour() < 10)
    {
      logfile.print("0");
    }
    logfile.print(starttime.hour(), DEC);
    if (starttime.minute() < 10)
    {
      logfile.print("0");
    }
    logfile.print(starttime.minute(), DEC);
    if (starttime.second() < 10)
    {
      logfile.print("0");
    }
    logfile.print(starttime.second(), DEC);
    logfile.println();
    logfile.println("datetime,temp_cold,temp_hot,temp_cold2,temp_hot2,pressure1,pressure2,millis");
  #else //PRINT_TO_FILE
    Serial.println("WARNING: DATA NOT LOGGED TO FILE");
    Serial.println();
    delay(1000);
  #endif
  

  #if ECHO_TO_SERIAL
    Serial.println("datetime,temp_cold,temp_hot,temp_cold2,temp_hot2,pressure1,pressure2,millis");
  #endif //ECHO_TO_SERIAL
 
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

    mcp.enable(true);

    // TC2
    /* Initialise the driver with I2C_ADDRESS and the default I2C bus. */
    if (! mcp2.begin(TC_ADDRESS_2)) {
        Serial.println("Sensor 1 not found. Check wiring!");
        while (1);
    }
    mcp2.setADCresolution(MCP9600_ADCRESOLUTION_18);
    Serial.print("ADC resolution set to ");
    switch (mcp2.getADCresolution()) {
      case MCP9600_ADCRESOLUTION_18:   Serial.print("18"); break;
      case MCP9600_ADCRESOLUTION_16:   Serial.print("16"); break;
      case MCP9600_ADCRESOLUTION_14:   Serial.print("14"); break;
      case MCP9600_ADCRESOLUTION_12:   Serial.print("12"); break;
    }
    Serial.println(" bits");

    mcp2.setThermocoupleType(MCP9600_TYPE_K);
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

    mcp2.setFilterCoefficient(3);
    Serial.print("Filter coefficient value set to: ");
    Serial.println(mcp2.getFilterCoefficient());

    mcp2.enable(true);

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
    temp_cold = mcp.readAmbient();
    temp_hot = mcp.readThermocouple();
    temp_cold2 = mcp2.readAmbient();
    temp_hot2 = mcp2.readThermocouple();
  #endif //READ_TEMPERATURE 
  #if READ_PRESSURE
    PressureADC1 = analogRead(PressurePin1);  
    PressureADC2 = analogRead(PressurePin2);  
    PressureVolt1 = (PressureADC1 * 5)/AdcRange;  
    PressureVolt2 = (PressureADC2 * 5)/AdcRange;  
    Pressure1 = PressureVolt1 * PressureScale;
    Pressure2 = PressureVolt2 * PressureScale;
  #endif //READ_PRESSURE
  
  #if PRINT_TO_FILE    
    logfile.print(now.year(), DEC);
    if (now.month() < 10)
    {
      logfile.print("0");
    }
    logfile.print(now.month(), DEC);
    if (now.day() < 10)
    {
      logfile.print("0");
    }
    logfile.print(now.day(), DEC);
    logfile.print("_");
    if (now.hour() < 10)
    {
      logfile.print("0");
    }
    logfile.print(now.hour(), DEC);
    if (now.minute() < 10)
    {
      logfile.print("0");
    }
    logfile.print(now.minute(), DEC);
    if (now.second() < 10)
    {
      logfile.print("0");
    }
    logfile.print(now.second(), DEC);
    logfile.print(",");
    logfile.print(temp_cold);
    logfile.print(",");
    logfile.print(temp_hot);
    logfile.print(",");
    logfile.print(temp_cold2);
    logfile.print(",");
    logfile.print(temp_hot2);
    logfile.print(",");    
    logfile.print(Pressure1);
    logfile.print(",");    
    logfile.print(Pressure2);
    logfile.print(",");    
    m = millis();
    logfile.print(m);           // milliseconds since start
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
    Serial.print(",");
    Serial.print(temp_cold2);
    Serial.print(",");
    Serial.print(temp_hot2);
    Serial.print(",");     
    Serial.print(Pressure1);
    Serial.print(",");    
    Serial.print(Pressure2);
    Serial.print(",");    
    m = millis();
    Serial.print(m);           // milliseconds since start
    Serial.println();
  #endif //ECHO_TO_SERIAL


  digitalWrite(greenLEDpin, LOW);

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  
  // blink LED to show we are syncing data to the card & updating FAT!
  digitalWrite(redLEDpin, HIGH);
  digitalWrite(extLEDpin, HIGH);
  logfile.flush();
  digitalWrite(redLEDpin, LOW);
  digitalWrite(extLEDpin, LOW);
}
