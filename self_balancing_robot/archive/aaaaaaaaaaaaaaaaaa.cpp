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
float staticOffset = 0; // Found via calibration
unsigned long lastTime = 0;

// PID Setup - Inner Loop (Angle)
float angleSetpoint = 0, angleOutput;
float Kp_angle = 10.0, Ki_angle = 0.0, Kd_angle = 1.0; 
pid anglePID(Kp_angle, Ki_angle, Kd_angle, -255, 255);

// PID Setup - Outer Loop (Velocity)
// Note: These gains are usually MUCH smaller than angle gains
float Kp_vel = 5.0, Ki_vel = 0.2; 
float vel_i_term = 0;
float velocitySetpoint = 0; // We want to stay still (0 m/s)

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
  /*  
  // --- AUTO CALIBRATION ---
  BTSerial.println("CALIBRATING... Hold robot at balance point!");
  delay(2000);
  float sumTheta = 0;
  int samples = 200;
  for (int i = 0; i < samples; i++) {
    if (mpu.dmpGetCurrentFIFOPacket(fifobuffer)) {
      mpu.dmpGetQuaternion(&q, fifobuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      sumTheta += (ypr[1] * 180 / M_PI);
    }
    delay(10);
  }
  staticOffset = sumTheta / samples;
  BTSerial.print("Offset Found: "); BTSerial.println(staticOffset);
  */
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
      theta = (ypr[1] * 180 / M_PI); //- staticOffset;
    }

    // 1. Calculate Velocity (m/s)
    x = ((leftEncoderCount + rightEncoderCount) / 2.0) * DISTANCE_PER_TICK;
    float raw_x_dot = (x - last_x) / dt;
    x_dot = (x_dot * 0.7) + (raw_x_dot * 0.3); // Low-pass filter for stability

    if (abs(theta) > 45) {
      driveMotors(0);
      anglePID.reset();
      vel_i_term = 0; // Reset velocity integral
    } else {
      // 2. OUTER LOOP: Velocity control decides the target angle
      float vel_error = velocitySetpoint - x_dot;
      vel_i_term += vel_error * dt;
      vel_i_term = constrain(vel_i_term, -5, 5); // Prevent integral windup
      
      // The output of velocity loop is our new angle setpoint
      angleSetpoint = (Kp_vel * vel_error) + (Ki_vel * vel_i_term);
      angleSetpoint = constrain(angleSetpoint, -10, 10); // Limit lean angle to 10 degrees

      // 3. INNER LOOP: Standard angle control
      angleOutput = anglePID.compute(angleSetpoint, theta, dt);
      driveMotors(angleOutput);
    }

    // Logging
    static int logCount = 0;
    if (logCount++ % 10 == 0) {
      BTSerial.print("T:"); BTSerial.print(theta, 1);
      BTSerial.print(" S:"); BTSerial.print(angleSetpoint, 1);
      BTSerial.print(" V:"); BTSerial.println(x_dot, 2);
    }

    last_x = x;
    lastTime = currentTime;
  }
}

void driveMotors(float pwm) {
  int deadzone_fwd = 30;
  int deadzone_rev = 30;
  if (pwm > 0) pwm += deadzone_fwd;
  else if (pwm < 0) pwm -= deadzone_rev;
  
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