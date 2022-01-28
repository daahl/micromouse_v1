#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// define TOF sensor addresses
#define LOXLL_ADDR 0x2A // left
#define LOXCC_ADDR 0x2B // center
#define LOXRR_ADDR 0x2C // right

// sensor objects
Adafruit_VL53L0X loxLL = Adafruit_VL53L0X();
Adafruit_VL53L0X loxCC = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRR = Adafruit_VL53L0X();

void Scanner ()
{
  Serial.println ();
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (Wire.endTransmission () == 0)  // Receive 0 = success (ACK response) 
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);     // PCF8574 7 bit address
      Serial.println (")");
      count++;
    }
  }
  Serial.print ("Found ");      
  Serial.print (count, DEC);        // numbers of devices
  Serial.println (" device(s).");
}

void setup() {
  Serial.begin(9600);
  // set up the TOF sensors.
  // notice! you have to plug them in one-by-one when the setup is running...
  // thanks for making the address non-writable ST
  Wire.begin();

  // start with the left sensor
  Serial.println("Setup starting for LEFT sensor...");
  delay(5000);
  // wait for the left sensor to respond
  while(1){
    Serial.println("Waiting for sensor to respond...");
    loxLL.begin(); // default address is 0x29 for all lox sensors
    loxLL.setAddress(LOXLL_ADDR);
    Wire.beginTransmission(LOXLL_ADDR);
    if(Wire.endTransmission() == 0){ // Receive 0 = success
      break;
    }
    delay(500);
  }
  Serial.println("LEFT sensor setup successfully!");
  Serial.println("Please insert CENTER sensor within 5 seconds...");
  delay(7000);

  // then the CENTER sensor
  // wait for the center sensor to respond
  while(1){
    Serial.println("Waiting for sensor to respond...");
    loxCC.begin(); // default address is 0x29 for all lox sensors
    loxCC.setAddress(LOXCC_ADDR);
    Wire.beginTransmission(LOXCC_ADDR);
    if(Wire.endTransmission() == 0){ // Receive 0 = success
      break;
    }
    delay(500);
  }
  Serial.println("CENTER sensor setup successfully!");
  Serial.println("Please insert RIGHT sensor within 5 seconds...");
  delay(7000);

  // finally the right sensor
  // wait for the left sensor to respond
  while(1){
    Serial.println("Waiting for sensor to respond...");
    loxRR.begin(); // default address is 0x29 for all lox sensors
    loxRR.setAddress(LOXRR_ADDR);
    Wire.beginTransmission(LOXRR_ADDR);
    if(Wire.endTransmission() == 0){ // Receive 0 = success
      break;
    }
    delay(500);
  }
  Serial.println("RIGHT sensor setup successfully!");
  Serial.println("Setup complete for ALL sensors!");
  delay(2000);

  Scanner();
  delay(5000);
}


void loop() {
  // range measurement variables
  VL53L0X_RangingMeasurementData_t measureLL;
  VL53L0X_RangingMeasurementData_t measureCC;
  VL53L0X_RangingMeasurementData_t measureRR;

  //Serial.print("Reading a measurement from LEFT... ");
  loxLL.rangingTest(&measureLL, false);
  //Serial.print("Reading a measurement from CENTER... ");
  loxCC.rangingTest(&measureCC, false);
  //Serial.print("Reading a measurement from RIGHT... ");
  loxRR.rangingTest(&measureRR, false);

  if(measureLL.RangeStatus != 4 && measureCC.RangeStatus != 4 && measureRR.RangeStatus != 4){
    Serial.print("Distance (mm): "); 
    Serial.print(measureLL.RangeMilliMeter);
    Serial.print(" ");
    Serial.print(measureCC.RangeMilliMeter);
    Serial.print(" ");
    Serial.println(measureRR.RangeMilliMeter);
  }
  else{
    Serial.println("One sensor is out of range...");
  }
  
  delay(1000);
}