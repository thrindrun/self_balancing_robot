#include <Arduino.h>
#include "lqr.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// pins, BUNLAR AYARLANCAK
const int leftEncA = 2; const int leftEncB = 4;
const int rightEncA = 3; const int rightEncB = 7;
const int ENA = 10; const int IN1 = 9; const int IN2 = 8;
const int ENB = 5; const int IN3 = 12; const int IN4 = 11;

// constants, BUNLAR DA AYARLANCAK
const float WHEEL_DIAMETER = .065; 
const float CPR = 360.0; // counts per rev
const float DISTANCE_PER_TICK = (M_PI*WHEEL_DIAMETER)/CPR;

// state variables
volatile long leftEncoderCount = 0, rightEncoderCount = 0;
float x = 0, x_dot = 0, last_x = 0;
float theta = 0, theta_dot = 0, last_theta = 0;
unsigned long lastTime = 0;

// lqr
float k1 = 1, k2 = 1, k3 = 1, k4 = 1;
float min = -255, max = 255;
lqr lqr1(k1,k2,k3,k4,min,max);

// MPU6050
MPU6050 mpu;
uint8_t fifobuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(ENA,OUTPUT); pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(ENB,OUTPUT); pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  pinMode(leftEncA,INPUT_PULLUP); pinMode(rightEncA,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncA), [](){(digitalRead(leftEncB)) ? leftEncoderCount++ : leftEncoderCount--;}, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncA), [](){(digitalRead(rightEncB)) ? rightEncoderCount++ : rightEncoderCount--;}, RISING);

  // MPU INIT
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime)/1000.0;

  if (dt >= 0.01) {
    // get states
    if (mpu.getFIFOCount()>=42){
      mpu.getFIFOBytes(fifobuffer,42);
      mpu.dmpGetQuaternion(&q,fifobuffer);
      mpu.dmpGetGravity(&gravity,&q);
      mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);

      theta = ypr[1] * 180/M_PI;
      theta_dot = (theta - last_theta) / dt;
      last_theta = theta;
    }

    x = ((leftEncoderCount+rightEncoderCount)/2.0)*DISTANCE_PER_TICK;
    x_dot = (x-last_x)/dt;
    last_x = x;
    // control loops
    if (abs(theta) > 45) {
      driveMotors(0);
    }
    else {
      float lqrOutput = lqr1.compute(x,x_dot,theta,theta_dot);
      driveMotors(lqrOutput);
    }
    
    lastTime = currentTime;
  }
}

void driveMotors(float pwm) {
  if (pwm > 0) pwm+=35;
  else if (pwm < 0) pwm-=35;
  
  if (pwm > 255) pwm = 255;
  else if (pwm < -255) pwm = -255;

  if (pwm > 0) {
    digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
    digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
  }
  else {
    digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
    digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);
  }
  analogWrite(ENA,abs(pwm)); analogWrite(ENB,abs(pwm));
}