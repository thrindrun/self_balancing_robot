#include <Arduino.h>

const int R_PWM_L = 4, L_PWM_L = 5, R_PWM_R = 6, L_PWM_R = 7;
const int leftEncA = 8, leftEncB = 9, rightEncA = 10, rightEncB = 11;
volatile long leftEncoderCount = 0, rightEncoderCount = 0;

void setup() {
    Serial.begin(115200);

    ledcSetup(0, 20000, 10); ledcSetup(1, 20000, 10);
    ledcSetup(2, 20000, 10); ledcSetup(3, 20000, 10);
    ledcAttachPin(R_PWM_L, 0); ledcAttachPin(L_PWM_L, 1);
    ledcAttachPin(R_PWM_R, 2); ledcAttachPin(L_PWM_R, 3);

    pinMode(leftEncA, INPUT_PULLUP); pinMode(leftEncB, INPUT_PULLUP);
    pinMode(rightEncA, INPUT_PULLUP); pinMode(rightEncB, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(leftEncA), [](){
        (digitalRead(leftEncB)) ? leftEncoderCount-- : leftEncoderCount++;
    }, RISING);
    attachInterrupt(digitalPinToInterrupt(rightEncA), [](){
        (digitalRead(rightEncB)) ? rightEncoderCount++ : rightEncoderCount--;
    }, RISING);

    Serial.println("Deadzone and direction test");
    delay(3000);
}

void loop() {
    static int test_pwm = 0;
    static unsigned long last_update = 0;

    // 1. HAND TEST PHASE (First 10 seconds)
    if (millis() < 10000) {
        if (millis() - last_update > 500) {
            last_update = millis();
            Serial.println("FREE SPIN MODE: Turn wheels forward by hand & check signs");
            Serial.printf("L: %ld | R: %ld\n", leftEncoderCount, rightEncoderCount);
        }
        return; // Skip deadzone test for now
    }

    // 2. DEADZONE TEST PHASE
    if (test_pwm <= 500) {
        if (millis() - last_update > 250) { // Slower ramp for better observation
            last_update = millis();
            ledcWrite(0, test_pwm); ledcWrite(1, 0);
            ledcWrite(2, 0); ledcWrite(3, test_pwm);
            
            Serial.printf("TESTING PWM: %d | L: %ld | R: %ld\n", 
                          test_pwm, leftEncoderCount, rightEncoderCount);
            test_pwm += 5;
        }
    } else {
        // Full Stop
        ledcWrite(0, 0); ledcWrite(1, 0);
        ledcWrite(2, 0); ledcWrite(3, 0);
    }
}