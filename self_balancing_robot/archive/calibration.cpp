#include <Arduino.h>

// Pins for BTS7960 (Matching your previous wiring)
const int leftEncA = 2;   const int leftEncB = 4;
const int rightEncA = 3;  const int rightEncB = 7;

// Left Driver
const int R_PWM_L = 9;  const int L_PWM_L = 8;  const int EN_L = 10;
// Right Driver
const int R_PWM_R = 12; const int L_PWM_R = 11; const int EN_R = 5;

volatile long leftCount = 0;
volatile long rightCount = 0;

void setup() {
  Serial.begin(115200);
  
  // Encoder Setup
  pinMode(leftEncA, INPUT_PULLUP); pinMode(leftEncB, INPUT_PULLUP);
  pinMode(rightEncA, INPUT_PULLUP); pinMode(rightEncB, INPUT_PULLUP);
  
  // BTS7960 Setup
  pinMode(R_PWM_L, OUTPUT); pinMode(L_PWM_L, OUTPUT); pinMode(EN_L, OUTPUT);
  pinMode(R_PWM_R, OUTPUT); pinMode(L_PWM_R, OUTPUT); pinMode(EN_R, OUTPUT);

  // Enable Drivers (Must be HIGH for BTS7960 to work)
  digitalWrite(EN_L, HIGH);
  digitalWrite(EN_R, HIGH);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(leftEncA), [](){(digitalRead(leftEncB)) ? leftCount++ : leftCount--;}, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncA), [](){(digitalRead(rightEncB)) ? rightCount++ : rightCount--;}, RISING);

  Serial.println("--- BTS7960 HARDWARE CALIBRATION ---");
  Serial.println("1. ENCODERS: Rotate wheels 1 full turn forward to verify counts.");
  Serial.println("2. DEADZONE: Watch PWM. Note the number when wheels START spinning.");
  delay(3000);
}

void loop() {
  static int testPWM = 0;
  static unsigned long lastRamp = 0;
  
  // Ramping PWM every 1 second for easier observation
  if (millis() - lastRamp > 1000) {
    testPWM += 2; // Slow ramp (+2) to find the exact deadzone
    if (testPWM > 80) testPWM = 0; // Reset after 80 (should spin by then)
    lastRamp = millis();
    
    // Drive Forward: RPWM active, LPWM 0
    analogWrite(R_PWM_L, testPWM);
    analogWrite(L_PWM_L, 0);
    analogWrite(R_PWM_R, testPWM);
    analogWrite(L_PWM_R, 0);
  }

  // Debugging Output
  Serial.print("L_Ticks: "); Serial.print(leftCount);
  Serial.print(" | R_Ticks: "); Serial.print(rightCount);
  Serial.print(" | Current_PWM: "); Serial.println(testPWM);
  
  delay(100);
}