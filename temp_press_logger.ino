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
#define PRINT_TO_FILE       0 
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
int PressureVolt1 = 0;
int PressureVolt2 = 0;
int Pressure1 = 0;
int Pressure2 = 0;

// the digital pins that connect to the LEDs
#define redLEDpin   4 
#define greenLEDpin 3


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

  while(1);
}

void setup(void)
{

  Serial.begin(9600);
  Serial.println();

  
}

void loop(void){
  
  #if READ_PRESSURE
    PressureVolt1 = analogRead(PressurePin1);  
    PressureVolt2 = analogRead(PressurePin2);  
    Pressure1 = (PressureVolt1 * PressureScale)/AdcRange;
    Pressure2 = (PressureVolt2 * PressureScale)/AdcRange;
  #endif //READ_PRESSURE

 
  #if ECHO_TO_SERIAL
   
    Serial.print(PressureVolt1);
    Serial.print(",");    
    Serial.print(PressureVolt2);
    Serial.println();
  #endif //ECHO_TO_SERIAL


  delay(1000);
}
