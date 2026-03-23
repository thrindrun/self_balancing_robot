#include <Arduino.h>

// Pins (Matching your main code)
const int leftEncA = 2;   const int leftEncB = 4;
const int rightEncA = 3;  const int rightEncB = 7;
const int R_PWM_L = 9;    const int L_PWM_L = 8;  const int EN_L = 10;
const int R_PWM_R = 12;   const int L_PWM_R = 11; const int EN_R = 5;

volatile long leftCount = 0; volatile long rightCount = 0;

void setup() {
  Serial.begin(115200);
  pinMode(leftEncA, INPUT_PULLUP); pinMode(rightEncA, INPUT_PULLUP);
  pinMode(R_PWM_L, OUTPUT); pinMode(L_PWM_L, OUTPUT); pinMode(EN_L, OUTPUT);
  pinMode(R_PWM_R, OUTPUT); pinMode(L_PWM_R, OUTPUT); pinMode(EN_R, OUTPUT);
  
  digitalWrite(EN_L, HIGH); digitalWrite(EN_R, HIGH);

  attachInterrupt(digitalPinToInterrupt(leftEncA), [](){(digitalRead(leftEncB)) ? leftCount-- : leftCount++;}, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncA), [](){(digitalRead(rightEncB)) ? rightCount++ : rightCount--;}, RISING);

  Serial.println("--- SYSTEM CHECK MODE ---");
  Serial.println("1. PUSH robot FORWARD: Both Ticks should INCREASE.");
  Serial.println("2. AFTER 5s: Motors will spin FORWARD at PWM 40.");
}

void loop() {
  static unsigned long startTime = millis();
  
  // Phase 1: Manual Push Test (First 5 seconds)
  if (millis() - startTime < 5000) {
    Serial.print("MANUAL TEST -> L_Ticks: "); Serial.print(leftCount);
    Serial.print(" | R_Ticks: "); Serial.println(rightCount);
  } 
  // Phase 2: Motor Direction Test
  else {
    Serial.println("MOTOR TEST -> Sending FORWARD PWM...");
    // Forward Logic: Left(R_PWM), Right(L_PWM)
    analogWrite(R_PWM_L, 40); analogWrite(L_PWM_L, 0);
    analogWrite(R_PWM_R, 0);  analogWrite(L_PWM_R, 40);
    delay(2000);
    
    analogWrite(R_PWM_L, 0); analogWrite(L_PWM_R, 0);
    Serial.println("Test Done. Check if robot moved Forward.");
    while(1); // Stop here
  }
  delay(100);
}