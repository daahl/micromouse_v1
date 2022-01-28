#include <Arduino.h>
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

// Motor positions
int posLL = 0;
int posRR = 0;

// PID variables
int prevT = 0;
float eprevLL = 0.0;
float eprevRR = 0.0;
float eintegralLL = 0.0;
float eintegralRR = 0.0;

// define PID parameters
float kp = 3;
float kd = 0.01;
float ki = 0.04;

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
  Serial.begin(9600);
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

void loop() {
  // set target pos
  int target = 1200;

  // time difference
  float currT = micros();
  float deltaT = ((float)(currT - prevT)/1.0e6);
  prevT = currT;

  // current error
  int eLL = target - posLL;
  int eRR = target - posLL;

  // derivative
  float dedtLL = (eLL - eprevLL)/deltaT;
  float dedtRR = (eRR - eprevRR)/deltaT;

  // integral
  eintegralLL = eintegralLL + eLL*deltaT;
  eintegralRR = eintegralRR + eRR*deltaT;

  // control signal
  float uLL = kp*eLL + kd*dedtLL + ki*eintegralLL;
  float uRR = kp*eRR + kd*dedtRR + ki*eintegralRR;

  // motor power
  float speedLL = fabs(uLL);
  float speedRR = fabs(uRR);
  if(speedLL > 255){
    speedLL = 255;
  }
  if(speedRR > 255){
    speedRR = 255;
  }
  

  // motor directions
  int dirLL = 1;
  int dirRR = 1;
  if(uLL < 0){
    dirLL = -1;
  }
  if(uRR < 0){
    dirRR = -1;
  }

  // remove super small errors, to remove motor noise
  /*if(abs(e) < 5){
    e = 0;
    dir = 0;
    pwr = 0;
  }*/
  
  // store previous error
  eprevLL = eLL;
  eprevRR = eRR;

  // signal the motor
  motorLL.drive(dirLL*speedLL);
  motorRR.drive(dirRR*speedRR);

  // debug prints
  Serial.print(posLL);
  Serial.print(",");
  //Serial.print(e);
  //Serial.print(",");
  Serial.println(target);
  //Serial.print(",");
  //Serial.print(pwr);
  //Serial.print(",");
  //Serial.println(posRR);
}
