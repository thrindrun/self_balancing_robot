#include <Arduino.h>

// Pins (Matching your previous code)
const int leftEncA = 2;   const int leftEncB = 4;
const int rightEncA = 3;  const int rightEncB = 7;
const int ENA = 10; const int IN1 = 9; const int IN2 = 8;
const int ENB = 5;  const int IN3 = 12; const int IN4 = 11;

volatile long leftCount = 0;
volatile long rightCount = 0;

void setup() {
  Serial.begin(115200);
  pinMode(leftEncA, INPUT_PULLUP); pinMode(leftEncB, INPUT_PULLUP);
  pinMode(rightEncA, INPUT_PULLUP); pinMode(rightEncB, INPUT_PULLUP);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(leftEncA), [](){(digitalRead(leftEncB)) ? leftCount++ : leftCount--;}, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncA), [](){(digitalRead(rightEncB)) ? rightCount++ : rightCount--;}, RISING);

  Serial.println("--- HARDWARE CALIBRATION MODE ---");
  Serial.println("1. Rotate wheels 1 full turn to find CPR.");
  Serial.println("2. Testing motor deadzone in 5 seconds...");
  delay(5000);
}

void loop() {
  // TEST 1: Print Encoder Counts
  Serial.print("L_Ticks: "); Serial.print(leftCount);
  Serial.print(" | R_Ticks: "); Serial.print(rightCount);

  // TEST 2: Find Deadzone (Slowly ramp up speed)
  static int testPWM = 0;
  static unsigned long lastRamp = 0;
  
  if (millis() - lastRamp > 500) {
    testPWM += 5;
    if (testPWM > 100) testPWM = 0; // Reset after 100
    lastRamp = millis();
    
    // Drive only one direction for test
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, testPWM);
    analogWrite(ENB, testPWM);
  }

  Serial.print(" | Current_PWM: "); Serial.println(testPWM);
  delay(100);
}