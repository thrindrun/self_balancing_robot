#include <Arduino.h>
#include "pid.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h> // Added for Bluetooth

// HC-06 Bluetooth Setup
// Arduino Pin 12 (RX) -> HC-06 TX
// Arduino Pin 13 (TX) -> HC-06 RX (Use a voltage divider here if possible)
SoftwareSerial BTSerial(12, 13); 

// Pin Mapping
const int leftEncA = 2; const int leftEncB = 4;
const int rightEncA = 3; const int rightEncB = 7;

// BTS7960 Pin Mapping (Pins 5, 9, 10, 11 are all PWM)
const int R_PWM_L = 10; 
const int L_PWM_L = 9;  
const int EN_L = 8;     

const int R_PWM_R = 11; 
const int L_PWM_R = 5; 
const int EN_R = 6;     // Moved from 12 to 6 to make room for BTSerial

// Constants
const float WHEEL_DIAMETER = 0.088; 
const float CPR = 224.0; 
const float DISTANCE_PER_TICK = (PI * WHEEL_DIAMETER) / CPR;

// State variables
volatile long leftEncoderCount = 0, rightEncoderCount = 0;
float x = 0, x_dot = 0, last_x = 0;
float theta = 0;
unsigned long lastTime = 0;

// PID Setup
float angleSetpoint = 0, angleOutput;
float Kp_angle = 7.0, Ki_angle = 0.05, Kd_angle = 1.2; 
float minVal_angle = -255, maxVal_angle = 255;
pid anglePID(Kp_angle, Ki_angle, Kd_angle, minVal_angle, maxVal_angle);

// MPU6050
MPU6050 mpu;
uint8_t fifobuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// Logging
unsigned long logCounter = 0;
const int logInterval = 20; // Increased interval slightly for BT stability

void driveMotors(float pwm);

void setup() {
  Serial.begin(115200);   // USB Serial
  BTSerial.begin(9600);   // HC-06 Default Baud Rate
  Wire.begin();
  Wire.setClock(400000); 

  pinMode(R_PWM_L, OUTPUT); pinMode(L_PWM_L, OUTPUT); pinMode(EN_L, OUTPUT);
  pinMode(R_PWM_R, OUTPUT); pinMode(L_PWM_R, OUTPUT); pinMode(EN_R, OUTPUT);
  pinMode(leftEncA, INPUT_PULLUP); pinMode(rightEncA, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(leftEncA), [](){(digitalRead(leftEncB)) ? leftEncoderCount-- : leftEncoderCount++;}, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncA), [](){(digitalRead(rightEncB)) ? rightEncoderCount++ : rightEncoderCount--;}, RISING);

  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);

  // Your specific offsets
  mpu.setXAccelOffset(-2396); mpu.setYAccelOffset(715); mpu.setZAccelOffset(1084);
  mpu.setXGyroOffset(584); mpu.setYGyroOffset(-699); mpu.setZGyroOffset(122);

  BTSerial.println("Bluetooth Connected. time,theta,setpoint,x,pwm");
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  lastTime = micros();
}

void loop() {
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;

  if (dt >= 0.01) { // 100Hz Loop
    if (mpu.dmpGetCurrentFIFOPacket(fifobuffer)) {
      mpu.dmpGetQuaternion(&q, fifobuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      theta = ypr[1] * 180 / M_PI;
    }

    x = ((leftEncoderCount + rightEncoderCount) / 2.0) * DISTANCE_PER_TICK;

    if (abs(theta) > 45) {
      angleOutput = 0;
      driveMotors(0);
      anglePID.reset();
    } else {
      angleOutput = anglePID.compute(angleSetpoint, theta, dt);
      driveMotors(angleOutput);
    }

    // Wireless Logging via Bluetooth
    if (logCounter % logInterval == 0) {
      //BTSerial.print(currentTime / 1000000.0, 2); BTSerial.print(",");
      BTSerial.print(theta, 2); BTSerial.print(",");
      BTSerial.println(angleOutput, 0);
    }

    last_x = x;
    lastTime = currentTime;
    logCounter++;
  }
}

void driveMotors(float pwm) {
  int deadzone_fwd = 30;
  int deadzone_rev = 40;
  if (pwm > 0) pwm += deadzone_fwd;
  else if (pwm < 0) pwm -= deadzone_rev;
  
  if (pwm > 255) pwm = 255;
  else if (pwm < -255) pwm = -255;

  if (pwm >= 0) {
    analogWrite(R_PWM_L, 0);
    analogWrite(L_PWM_L, pwm);
    analogWrite(R_PWM_R, pwm);
    analogWrite(L_PWM_R, 0);
  } else {
    analogWrite(R_PWM_L, abs(pwm));
    analogWrite(L_PWM_L, 0);
    analogWrite(R_PWM_R, 0);
    analogWrite(L_PWM_R, abs(pwm));
  }
  digitalWrite(EN_L, HIGH);
  digitalWrite(EN_R, HIGH);
}