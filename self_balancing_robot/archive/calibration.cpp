#include <Arduino.h>

// Pins for the SINGLE driver/motor being tested
const int encA = 2;   // Encoder A (Hardware Interrupt Pin)
const int encB = 4;   // Encoder B

// BTS7960 Pins for the single motor
const int R_PWM = 10;  
const int L_PWM = 9;  
const int EN = 8;    // Connect to BOTH R_EN and L_EN

volatile long count = 0;

void setup() {
  Serial.begin(115200);
  
  // Encoder Pins
  pinMode(encA, INPUT_PULLUP); 
  pinMode(encB, INPUT_PULLUP);
  
  // BTS7960 Pins
  pinMode(R_PWM, OUTPUT); 
  pinMode(L_PWM, OUTPUT); 
  pinMode(EN, OUTPUT);

  // Activate the driver
  digitalWrite(EN, HIGH);

  // Hardware Interrupt for accurate counting
  attachInterrupt(digitalPinToInterrupt(encA), [](){
    (digitalRead(encB)) ? count++ : count--;
  }, RISING);

  Serial.println("--- SINGLE MOTOR CALIBRATION ---");
  Serial.println("1. ENCODER: Rotate wheel 1 full turn to find CPR.");
  Serial.println("2. DEADZONE: Note PWM when the wheel starts moving.");
  delay(3000);
}

void loop() {
  static int testPWM = 0;
  static unsigned long lastRamp = 0;
  
  // Slow ramp-up to find the exact starting voltage
  if (millis() - lastRamp > 1000) {
    testPWM += 10; 
    if (testPWM > 255) testPWM = 0; 
    lastRamp = millis();
    
    // Test RPWM direction
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, testPWM);
  }

  // Monitor Ticks and PWM
  Serial.print("Ticks: "); Serial.print(count);
  Serial.print(" | Current_PWM: "); Serial.println(testPWM);
  
  delay(100);
}