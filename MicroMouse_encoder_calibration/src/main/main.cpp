/*#include <Arduino.h>
#include "SparkFun_TB6612.h"

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

// "Absolute" motor positions are set to 0 on startup
int absPosLL = 0;
int absPosRR = 0;

// acceleration parameters
float accLL = 0.0;
float accRR = 0.0;
float accLim = 0.75;
float deaccLim = 0.40;
float speedLim = 0.0;
float accSpeed = 3.0;
float deaccSpeed = 30.0;
int whineLim = 15;

// state varaibles
bool parked = 1;
bool accelerating = 0;
bool driving = 0;
bool deaccelerating = 0;
float distanceLim = 3.0;

// PID variables
int prevT = 0;
float eprevLL = 0.0;
float eprevRR = 0.0;
float eintegralLL = 0.0;
float eintegralRR = 0.0;
float speedLL = 0.0;
float speedRR = 0.0;

// define PID parameters
float kp = 7; //7
float kd = 0.1; //0.1
float ki = 0.0; //0

void readEncoderLL(){
  int b = digitalRead(encoderLLBB);
  // Add to pos on clockwise rotation
  if(b > 0){
    absPosLL--;
  }
  else{
    absPosLL++;
  }
}

void readEncoderRR(){
  int b = digitalRead(encoderRRBB);
  // Add to pos on counter clockwise rotation
  if(b > 0){
    absPosRR++;
  }
  else{
    absPosRR--;
  }
}

//
//  Return the absolute distance to the target.
//  0 = left motor
//  1 = right motor
//
float absDist2Target(int target, bool leftOrRight){
  if(leftOrRight)
  {
    return fabs(target - absPosRR);
  }else{
    return fabs(target - absPosLL);
  }
  
}

//
//  Regulates the speed to the motors. Has to be constantly called to regulate.
//
void PID(int targetLL, int targetRR){
  // time difference
  float currT = micros();
  float deltaT = ((float)(currT - prevT)/1.0e6);
  prevT = currT;

  // current error
  int eLL = targetLL - absPosLL;
  int eRR = targetRR - absPosRR;

  // derivative
  float dedtLL = (eLL - eprevLL)/deltaT;
  float dedtRR = (eRR - eprevRR)/deltaT;

  // integral
  eintegralLL = eintegralLL + eLL*deltaT;
  eintegralRR = eintegralRR + eRR*deltaT;

  // control signal
  float uLL = kp*eLL + kd*dedtLL + ki*eintegralLL;
  float uRR = kp*eRR + kd*dedtRR + ki*eintegralRR;

  // set and limit motor power
  speedLL = fabs(uLL);
  
  if(speedLL > speedLim){
    speedLL = speedLim;
  }
  speedRR = fabs(uRR);
  if(speedRR > speedLim){
    speedRR = speedLim;
  }

  // motor directions
  int dirLL = 1;
  if(uLL < 0){
    dirLL = -1;
  }
  int dirRR = 1;
  if(uRR < 0){
    dirRR = -1;
  }
  
  // store previous error
  eprevLL = eLL;
  eprevRR = eRR;

  // signal the motor
  motorLL.drive(dirLL*(int)speedLL);
  motorRR.drive(dirRR*(int)speedRR);

  // debug prints
  Serial.print("posLL:");
  Serial.print(absPosLL);
  Serial.print(",");
  Serial.print("posRR:");
  Serial.print(absPosRR);
  Serial.print(",");
  Serial.print("targetLL:");
  Serial.print(targetLL);
  Serial.print(",");
  Serial.print("targetRR:");
  Serial.println(targetRR);
}

//
//  State handling function to give the micromouse acceleration and deacceleration
//
void driveTo(int distanceLL, int distanceRR){
  float targetLL = absPosLL + distanceLL;
  float targetRR = absPosRR + distanceRR;

  // ### Parked state ### //
  while(parked){
    // too far away from target, change from parked to accelerating
    if(absDist2Target(targetLL, 0) > distanceLim or absDist2Target(targetRR, 1) > distanceLim){
    parked = 0;
    accelerating = 1;
    Serial.println("##### Accelerating #####");
    }
  }

  // ### Accelerating state ### //
  while(accelerating){
    if(speedLim < 255){
      speedLim += accSpeed;
    }
    PID(targetLL, targetRR);

    // accelerate until accLim% distance remains
    if((absDist2Target(targetLL, 0) <= accLim*fabs(distanceLL)) and (absDist2Target(targetRR, 1) <= accLim*fabs(distanceRR))){
      accelerating = 0;
      driving = 1;
      Serial.println("##### Driving #####");

      // short disances causes the speedLim to be too low,
      // minimum speed to move forward is 50.
      if(speedLim < 50){
        speedLim = 50;
      }
    }
  }

  // ### Driving state ### //
  while(driving){
    PID(targetLL, targetRR);

    // begin deaccelerating when deaccLim% distance remains
    if((absDist2Target(targetLL, 0) <= deaccLim*fabs(distanceLL)) and (absDist2Target(targetRR, 1) <= deaccLim*fabs(distanceRR))){
      driving = 0;
      deaccelerating = 1;
      Serial.println("##### Deaccelerating #####");
    }
  }

  // ### Deaccelerating state ### //
  while(deaccelerating){
    if(speedLim >= 70){
      speedLim -= deaccSpeed;
    }
    PID(targetLL, targetRR);

    // stop deaccelerating when close enough to target
    if(absDist2Target(targetLL, 0) <= distanceLim and absDist2Target(targetRR, 1) <= distanceLim){
      deaccelerating = 0;
      parked = 1;
      speedLim = 0.0;
      motorLL.brake();
      motorRR.brake();
      Serial.println("##### Parked #####");
    }
  }
}

void setup() {
  Serial.begin(115200);
  // Left motor encoder
  pinMode(encoderLLAA, INPUT);
  pinMode(encoderLLBB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderLLAA), readEncoderLL, RISING);

  // Right motor encoder
  pinMode(encoderRRAA, INPUT);
  pinMode(encoderRRBB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderRRAA), readEncoderRR, RISING);
  delay(4000);
}

void loop(){
    driveTo(500, 500);
    delay(700);
    driveTo(-345, +345);
    delay(700);
}*/
/*************************************************************

  This is a simple demo of sending and receiving some data.
  Be sure to check out other examples!
 *************************************************************/
//#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
//#include "SparkFun_TB6612.h"

// Template ID, Device Name and Auth Token are provided by the Blynk.Cloud
// See the Device Info tab, or Template settings
#define BLYNK_TEMPLATE_ID "TMPLzIux-iXu"
#define BLYNK_DEVICE_NAME "Micromouse v2"
#define BLYNK_AUTH_TOKEN "2FeJeGJ-Ofr3sv2tWK5VPHD_gEaWwQE8"

// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial

// Hardware pins
#define BOARD_LED_1 32

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

// Motor objects and variables to control the motors
//Motor motorLL = Motor(AIN1, AIN2, PWMA, LOFFSET, STBY);
//Motor motorRR = Motor(BIN1, BIN2, PWMB, ROFFSET, STBY);
int speed = 0;
int turn = 0;

char auth[] = BLYNK_AUTH_TOKEN;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Mulldalen";
char pass[] = "vilseiskogen";

BlynkTimer timer;

void driver(){
  //motorLL.drive(speed + turn);
  //motorRR.drive(speed - turn);
}

// Speed controller
BLYNK_WRITE(V0){
  int value = param.asInt();
  speed = value;
  driver();
}

// Turn controller
BLYNK_WRITE(V1){
  int value = param.asInt();
  turn = value;
  driver();
}

// LED controller. Is called everytime virtual pin 3 changes value
BLYNK_WRITE(V2)
{
  // Set incoming value from pin V0 to a variable
  int value = param.asInt();
  if(value == 1){
    digitalWrite(BOARD_LED_1, HIGH);
  }else{
    digitalWrite(BOARD_LED_1, LOW);
  }
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
  // Debug console and Blynk setup
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);
  // Setup a function to be called every second
  timer.setInterval(1000L, myTimerEvent);

  // LED setup
  pinMode(BOARD_LED_1, OUTPUT);
}

void loop()
{
  Blynk.run();
  timer.run();
  // You can inject your own code or combine it with other sketches.
  // Check other examples on how to communicate with Blynk. Remember
  // to avoid delay() function!
}


