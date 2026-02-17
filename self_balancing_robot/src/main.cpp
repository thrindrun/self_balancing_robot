#include <Arduino.h>
#include "pid.h"
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
float theta = 0, theta_dot = 0;
unsigned long lastTime = 0;

// pid
// position control
float posSetpoint = 0, posInput, posOutput;
float Kp_pos = 1, Ki_pos = 0.1, Kd_pos = 0.5; // BUNLAR DEĞİŞİCEK
float minVal_pos = -10, maxVal_pos = 10;
pid posPID(Kp_pos,Ki_pos,Kd_pos,minVal_pos,maxVal_pos);

// angle control
float angleSetpoint = 0, angleInput, angleOutput;
float Kp_angle = 1, Ki_angle = 1, Kd_angle = 1; // BUNLAR DA DEĞİŞİCEK
float minVal_angle = -255, maxVal_angle = 255;
pid anglePID(Kp_angle,Ki_angle,Kd_angle,minVal_angle,maxVal_angle);

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
    }

    x = ((leftEncoderCount+rightEncoderCount)/2.0)*DISTANCE_PER_TICK;
    x_dot = (x-last_x)/dt;

    // control loops
    angleSetpoint = posPID.compute(posSetpoint,x,dt); // desired angle
    angleOutput = anglePID.compute(angleSetpoint,theta,dt); // pwm

    // drive motors
    driveMotors(angleOutput);

    last_x = x;
    lastTime = currentTime;

  }
}

void driveMotors(float pwm) {
  if (pwm > 0) pwm+=35;
  else if (pwm < 0) pwm-=35;
  pwm = constrain(pwm,-255,255);

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