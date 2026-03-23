#include <Arduino.h>
#include "pid.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// pins, BUNLAR AYARLANCAK
const int leftEncA = 2; const int leftEncB = 4;
const int rightEncA = 3; const int rightEncB = 7;
// BTS7960 Pin Mapping
const int R_PWM_L = 9;  // Connect to RPWM of Left Driver
const int L_PWM_L = 8;  // Connect to LPWM of Left Driver
const int EN_L = 10;    // Connect to R_EN and L_EN of Left Driver

const int R_PWM_R = 12; // Connect to RPWM of Right Driver
const int L_PWM_R = 11; // Connect to LPWM of Right Driver
const int EN_R = 5;     // Connect to R_EN and L_EN of Right Driver

// constants, BUNLAR DA AYARLANCAK
const float WHEEL_DIAMETER = .088; 
const float CPR = 224.0; // counts per rev
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
//pid posPID(Kp_pos,Ki_pos,Kd_pos,minVal_pos,maxVal_pos);

// angle control
float angleSetpoint = 0, angleInput, angleOutput;
float Kp_angle = 5.0, Ki_angle = 0.0, Kd_angle = 0.5; // BUNLAR DA DEĞİŞİCEK
float minVal_angle = -255, maxVal_angle = 255;
pid anglePID(Kp_angle,Ki_angle,Kd_angle,minVal_angle,maxVal_angle);

// MPU6050
MPU6050 mpu;
uint8_t fifobuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// log 
unsigned long logCounter = 0;
const int logInterval = 5;

void driveMotors(float pwm);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock for faster communication

  pinMode(R_PWM_L,OUTPUT); pinMode(L_PWM_L,OUTPUT); pinMode(EN_L,OUTPUT);
  pinMode(R_PWM_R,OUTPUT); pinMode(L_PWM_R,OUTPUT); pinMode(EN_R,OUTPUT);
  pinMode(leftEncA,INPUT_PULLUP); pinMode(rightEncA,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncA), [](){(digitalRead(leftEncB)) ? leftEncoderCount-- : leftEncoderCount++;}, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncA), [](){(digitalRead(rightEncB)) ? rightEncoderCount++ : rightEncoderCount--;}, RISING);

  // MPU INIT
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);

  mpu.setXAccelOffset(-2435); 
  mpu.setYAccelOffset(729);   
  mpu.setZAccelOffset(1079);  
  mpu.setXGyroOffset(608);    
  mpu.setYGyroOffset(-750);   
  mpu.setZGyroOffset(124);
  Serial.println(F("Offsets applied to MPU6050."));

  Serial.println("time,theta,angleSetpoint,x,x_dot,posSetpoint,pwm");
  leftEncoderCount = 0;
  rightEncoderCount = 0;
}

void loop() {
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime)/1000000.0;

  if (dt >= 0.01) {
    // get states
    if (mpu.dmpGetCurrentFIFOPacket(fifobuffer)){
      mpu.dmpGetQuaternion(&q,fifobuffer);
      mpu.dmpGetGravity(&gravity,&q);
      mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);
      theta = ypr[1] * 180/M_PI;
    }

    x = ((leftEncoderCount+rightEncoderCount)/2.0)*DISTANCE_PER_TICK;
    x_dot = (x-last_x)/dt;

    // control loops
    if (abs(theta) > 45) {
      angleOutput = 0;
      driveMotors(0);
      //posPID.reset();
      anglePID.reset();
    }
    else {
      //angleSetpoint = posPID.compute(posSetpoint,x,dt); // desired angle
      angleOutput = anglePID.compute(angleSetpoint,theta,dt); // pwm
      driveMotors(angleOutput);
    }

    last_x = x;
    lastTime = currentTime;

    if (logCounter % logInterval == 0) {
      Serial.print(currentTime / 1000.0, 3); Serial.print(",");
      Serial.print(theta, 2); Serial.print(",");
      Serial.print(angleSetpoint, 2); Serial.print(",");
      Serial.print(x, 3); Serial.print(",");
      Serial.print(x_dot, 3); Serial.print(",");
      Serial.print(posSetpoint, 3); Serial.print(",");
      Serial.println(angleOutput, 0);
    }
    logCounter++;
  }
}

void driveMotors(float pwm) {
  int deadzone_fwd = 80;
  int deadzone_bwd = 20;
  if (pwm > 0) pwm+=deadzone_fwd;
  else if (pwm < 0) pwm-=deadzone_bwd;
  
  if (pwm > 255) pwm = 255;
  else if (pwm < -255) pwm = -255;

  if (pwm >= 0) {
    // Forward: RPWM gets signal, LPWM is 0
    analogWrite(R_PWM_L, 0);
    analogWrite(L_PWM_L, pwm);
    analogWrite(R_PWM_R, pwm);
    analogWrite(L_PWM_R, 0);
  } else {
    // Backward: LPWM gets signal, RPWM is 0
    analogWrite(R_PWM_L, abs(pwm));
    analogWrite(L_PWM_L, 0);
    analogWrite(R_PWM_R, 0);
    analogWrite(L_PWM_R, abs(pwm));
  }
  digitalWrite(EN_L, HIGH);
  digitalWrite(EN_R, HIGH);
}