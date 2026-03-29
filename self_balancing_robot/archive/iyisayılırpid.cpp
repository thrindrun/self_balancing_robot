#include <Arduino.h>
#include "pid.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(12, 13); 

// Pin Mapping
const int leftEncA = 2; const int leftEncB = 4;
const int rightEncA = 3; const int rightEncB = 7;
const int R_PWM_L = 10; const int L_PWM_L = 9; const int EN_L = 8;    
const int R_PWM_R = 11; const int L_PWM_R = 5; const int EN_R = 6;    

// Constants
const float WHEEL_DIAMETER = 0.088; 
const float CPR = 224.0; 
const float DISTANCE_PER_TICK = (PI * WHEEL_DIAMETER) / CPR;

// State variables
volatile long leftEncoderCount = 0, rightEncoderCount = 0;
float x = 0, x_dot = 0, last_x = 0;
float theta = 0;
unsigned long lastTime = 0;

// PID Setup - INNER LOOP (Angle)
float angleSetpoint = 0, angleOutput;
float Kp_angle = 10.0, Ki_angle = 0.0, Kd_angle = 1.0; 
pid anglePID(Kp_angle, Ki_angle, Kd_angle, -255, 255);

// PID Setup - MIDDLE LOOP (Velocity)
float Kp_vel = 2.0, Ki_vel = 0.1; // Reduced Kp_vel slightly to favor position
float vel_i_term = 0;
float velocitySetpoint = 0; 

// PID Setup - OUTER LOOP (Position)
// This loop runs every 20-50ms typically, but we'll keep it in the 100Hz for simplicity
float Kp_pos = 0.8;  // Start very low!
float target_x = 0;  // This is the "Anchor" spot on the floor

// MPU6050
MPU6050 mpu;
uint8_t fifobuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

void driveMotors(float pwm);

void setup() {
  Serial.begin(115200);
  BTSerial.begin(9600);
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

  mpu.setXAccelOffset(-2396); mpu.setYAccelOffset(715); mpu.setZAccelOffset(1084);
  mpu.setXGyroOffset(584); mpu.setYGyroOffset(-699); mpu.setZGyroOffset(122);

  lastTime = micros();
}

void loop() {
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;

  if (dt >= 0.01) { // 100Hz
    if (mpu.dmpGetCurrentFIFOPacket(fifobuffer)) {
      mpu.dmpGetQuaternion(&q, fifobuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      theta = (ypr[1] * 180 / M_PI);
    }

    // Encoder Math
    x = ((leftEncoderCount + rightEncoderCount) / 2.0) * DISTANCE_PER_TICK;
    float raw_x_dot = (x - last_x) / dt;
    x_dot = (x_dot * 0.8) + (raw_x_dot * 0.2); // Heavy filter for smooth velocity

    if (abs(theta) > 45) {
      driveMotors(0);
      anglePID.reset();
      vel_i_term = 0;
      target_x = x; // Reset target to current position when fallen
    } else {
      
      // 1. POSITION LOOP (Outer)
      float x_error = target_x - x;
      velocitySetpoint = x_error * Kp_pos; 
      velocitySetpoint = constrain(velocitySetpoint, -0.5, 0.5); // Max speed 0.5m/s

      // 2. VELOCITY LOOP (Middle)
      float vel_error = velocitySetpoint - x_dot;
      vel_i_term += vel_error * dt;
      vel_i_term = constrain(vel_i_term, -5, 5); 
      
      angleSetpoint = (Kp_vel * vel_error) + (Ki_vel * vel_i_term);
      angleSetpoint = constrain(angleSetpoint, -10, 10); // Max lean 10 degrees

      // 3. ANGLE LOOP (Inner)
      angleOutput = anglePID.compute(angleSetpoint, theta, dt);
      driveMotors(angleOutput);
    }

    // Logging for Tuning
    static int logCount = 0;
    if (logCount++ % 15 == 0) {
      BTSerial.print("X:"); BTSerial.print(x, 2);
      BTSerial.print(" V:"); BTSerial.print(x_dot, 2);
      BTSerial.print(" S:"); BTSerial.println(angleSetpoint, 1);
    }

    last_x = x;
    lastTime = currentTime;
  }
}

void driveMotors(float pwm) {
  int deadzone = 30; // Unified deadzone for simplicity
  if (pwm > 0) pwm += deadzone;
  else if (pwm < 0) pwm -= deadzone;
  
  pwm = constrain(pwm, -255, 255);

  if (pwm >= 0) {
    analogWrite(R_PWM_L, 0); analogWrite(L_PWM_L, pwm);
    analogWrite(R_PWM_R, pwm); analogWrite(L_PWM_R, 0);
  } else {
    analogWrite(R_PWM_L, abs(pwm)); analogWrite(L_PWM_L, 0);
    analogWrite(R_PWM_R, 0); analogWrite(L_PWM_R, abs(pwm));
  }
  digitalWrite(EN_L, HIGH); digitalWrite(EN_R, HIGH);
}