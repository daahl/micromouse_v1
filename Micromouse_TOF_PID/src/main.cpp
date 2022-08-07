/*
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h" // TOF library
#include "SparkFun_TB6612.h"  // Motor driver library

//########## General setup ##########
#define LED1 32
#define LED2 33

//########## TOF sensor setup ##########
// define TOF sensor addresses
#define LOXLL_ADDR 0x2A // left
#define LOXCC_ADDR 0x2B // center
#define LOXRR_ADDR 0x2C // right

// sensor objects
Adafruit_VL53L0X loxLL = Adafruit_VL53L0X();
Adafruit_VL53L0X loxCC = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRR = Adafruit_VL53L0X();

//########## Motor setup ##########
// Motor driver
#define AIN1    17
#define AIN2    18
#define PWMA    19
#define LOFFSET -1
#define BIN1    4
#define BIN2    2
#define PWMB    15
#define ROFFSET -1
#define STBY    27

// Motor encoders
#define encoderLLAA  36
#define encoderLLBB  39
#define encoderRRAA  35
#define encoderRRBB  34

// Motor objects to control the motors
Motor motorLL = Motor(AIN1, AIN2, PWMA, LOFFSET, STBY);
Motor motorRR = Motor(BIN1, BIN2, PWMB, ROFFSET, STBY);

// Motor positions
int posLL = 0;
int posRR = 0;

//########## PID setup ##########
// PID variables
int prevT = 0;
float errprev = 0.0;
float errintegral = 0.0;
float speedLL = 0;
float speedRR = 0;

// define PID parameters
float kp = 3;
float kd = 0.05;
float ki = 0.0;

//
// Scans the i2c address domain and prints the found sensors
//
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

//
// Reads the left motor encoder and tracks its rotations
//
void readEncoderLL(){
  int b = digitalRead(encoderLLBB);
  // Add to pos on clockwise rotation
  if(b > 0){
    posLL--;
  }
  else{
    posLL++;
  }
}


//
// Reads the right motor encoder and tracks its rotations
//
void readEncoderRR(){
  int b = digitalRead(encoderRRBB);
  // Add to pos on counter clockwise rotation
  if(b > 0){
    posRR++;
  }
  else{
    posRR--;
  }
}

void setup() {
  //########## General setup ##########
  Serial.begin(9600);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  //########## TOF sensor setup ##########
  // notice! you have to plug them in one-by-one when the setup is running...
  // thanks for making the address non-writable ST
  Wire.begin();

  // start with the left sensor
  Serial.println("Setup starting for LEFT sensor...");
  delay(5000);
  // wait for the left sensor to respond
  while(1){
    Serial.println("Waiting for sensor to respond...");
    digitalWrite(LED1, HIGH);
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
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
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
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, HIGH);
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
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  delay(250);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  delay(250);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  delay(250);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  delay(250);
  Scanner();
  //########## Motor setup ##########
  // Left motor encoder
  pinMode(encoderLLAA, INPUT);
  pinMode(encoderLLBB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderLLAA), readEncoderLL, RISING);
  // Right motor encoder
  pinMode(encoderRRAA, INPUT);
  pinMode(encoderRRBB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderRRAA), readEncoderRR, RISING);
  Serial.println("Motor setup complete!");

  delay(2000);
}


void loop() {
  //########## Read the TOF sensors ##########
  // range measurement variables
  VL53L0X_RangingMeasurementData_t measureLL;
  VL53L0X_RangingMeasurementData_t measureCC;
  VL53L0X_RangingMeasurementData_t measureRR;

  
  loxLL.rangingTest(&measureLL, false);
  loxCC.rangingTest(&measureCC, false);
  loxRR.rangingTest(&measureRR, false);

  int distLL = measureLL.RangeMilliMeter;
  int distCC = measureCC.RangeMilliMeter;
  int distRR = measureRR.RangeMilliMeter;

  if(measureLL.RangeStatus != 4 && measureCC.RangeStatus != 4 && measureRR.RangeStatus != 4){
    Serial.print("Distance (mm): "); 
    Serial.print(distLL);
    Serial.print(" ");
    Serial.print(distCC);
    Serial.print(" ");
    Serial.println(distRR);
  }
  else{
    Serial.println("One sensor is out of range...");
  }
  
  //########## Do some PID stuff ##########
  // time difference
  float currT = micros();
  float deltaT = ((float)(currT - prevT)/1.0e6);
  prevT = currT;

  // current error
  int err = distLL - distRR; // negative = too far right, positive = too far left

  // derivative
  float dedt = (err - errprev)/deltaT;

  // integral
  errintegral = errintegral + err*deltaT;

  // control signal
  float u = kp*err + kd*dedt + ki*errintegral;


  // motor power
  if(err < 0){ // drive further left
    speedLL = fabs(u);
    speedRR = 0;
  }
  else if(err > 0){ // drive further right
    speedLL = 0;
    speedRR = fabs(u);
  }
  else{
    speedLL = 0;
    speedRR = 0;
  }

  // Limit the extra speed that error corection gives
  if(speedLL > 100){
    speedLL = 100;
  }
  if(speedRR > 100){
    speedRR = 100;
  }

  // store previous error
  errprev = err;

  // signal the motors
  motorLL.drive(speedLL + 50);
  motorRR.drive(speedRR + 50);

  Serial.println(err);
  Serial.println();
}*/
// Code above was the old one. Code below is with Blynk.app
/*************************************************************

  This is a simple demo of sending and receiving some data.
  Be sure to check out other examples!
 *************************************************************/

// Template ID, Device Name and Auth Token are provided by the Blynk.Cloud
// See the Device Info tab, or Template settings
#define BLYNK_TEMPLATE_ID           "TMPLONAKqwBH"
#define BLYNK_DEVICE_NAME           "Quickstart Device"
#define BLYNK_AUTH_TOKEN            "H0kLMv5MtTSdw-0jVQvkUl56ezXpYRFl"


// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial


#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

char auth[] = BLYNK_AUTH_TOKEN;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Mulldalen";
char pass[] = "vilseiskogen";

BlynkTimer timer;

// This function is called every time the Virtual Pin 0 state changes
BLYNK_WRITE(V0)
{
  // Set incoming value from pin V0 to a variable
  int value = param.asInt();

  // Update state
  Blynk.virtualWrite(V1, value);
}

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
  // Change Web Link Button message to "Congratulations!"
  Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
  Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
}

// This function sends Arduino's uptime every second to Virtual Pin 2.
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V2, millis() / 1000);
}

void setup()
{
  // Debug console
  Serial.begin(115200);

  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);

  // Setup a function to be called every second
  timer.setInterval(1000L, myTimerEvent);
}

void loop()
{
  Blynk.run();
  timer.run();
  // You can inject your own code or combine it with other sketches.
  // Check other examples on how to communicate with Blynk. Remember
  // to avoid delay() function!
}

