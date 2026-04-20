#include <Arduino.h>
#include <NimBLEDevice.h>
#include "pid.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

portMUX_TYPE myMux = portMUX_INITIALIZER_UNLOCKED;

NimBLEServer* pServer = NULL;
NimBLECharacteristic* pTxCharacteristic = NULL;
NimBLECharacteristic* pRxCharacteristic = NULL;

bool deviceConnected = false;
class ConnectionHandler: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(NimBLEServer* pServer) { deviceConnected = false; }
};

class TuningHandler: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            Serial.print("Received value: ");
            Serial.println(value.c_str()); }
    }
};

pid positionPID(0.05, 0.0, 0.002, -0.5,0.5); // PID for position control
pid velocityPID(0.6, 0.0, 0.02, -10, 10); // PID for velocity control
pid anglePID(12.0,0.2,1.2,-1023,1023);

MPU6050 mpu;

volatile long leftEncoderCount = 0, rightEncoderCount = 0;
volatile float v_theta = 0, v_position = 0, v_pwm = 0, v_velocity = 0;


const int R_PWM_L = 4, L_PWM_L = 5; 
const int R_PWM_R = 6, L_PWM_R = 7;
const int leftEncA = 8, leftEncB = 9, rightEncA = 10, rightEncB = 11;
const int SDA_PIN = 1, SCL_PIN = 2;

const float DIST_PER_TICK = (0.088f * M_PI) / 224.0f; // Wheel diameter 88mm, 224 ticks per revolution

void setupBLE();
void driveMotors(float pwm);
void controlTask(void *pvParameters);

void setup() {
    Serial.begin(115200);
    setupBLE();
    
    ledcSetup(0, 20000, 10); // Channel 0, 20 kHz, 10-bit resolution
    ledcSetup(1, 20000, 10); // Channel 1, 20 kHz, 10-bit resolution
    ledcSetup(2, 20000, 10); // Channel 2, 20 kHz, 10-bit resolution
    ledcSetup(3, 20000, 10); // Channel 3, 20 kHz, 10-bit resolution

    ledcAttachPin(R_PWM_L, 0); ledcAttachPin(L_PWM_L, 1);
    ledcAttachPin(R_PWM_R, 2); ledcAttachPin(L_PWM_R, 3);

    pinMode(leftEncA, INPUT_PULLUP);
    pinMode(leftEncB, INPUT_PULLUP);
    pinMode(rightEncA, INPUT_PULLUP);
    pinMode(rightEncB, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(leftEncA), []() {
        (digitalRead(leftEncB)) ? leftEncoderCount-- : leftEncoderCount++;
    }, RISING);
    attachInterrupt(digitalPinToInterrupt(rightEncA), []() {
        (digitalRead(rightEncB)) ? rightEncoderCount++ : rightEncoderCount--;
    }, RISING);

    Wire.begin(SDA_PIN, SCL_PIN, 400000); // Initialize I2C with specified pins and frequency

    xTaskCreatePinnedToCore(controlTask, "controlTask", 10000, NULL, 1, NULL, 0); // Create control task on core 0
}
void loop() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
        lastPrint = millis();
        if (deviceConnected) {
            portENTER_CRITICAL(&myMux);
            float t = v_theta;
            float p = v_pwm;
            float pos = v_position;
            float vel = v_velocity;
            portEXIT_CRITICAL(&myMux);
            char buffer[100];
            snprintf(buffer, sizeof(buffer), "Theta: %.2f, PWM: %.0f, Pos: %.2f, Vel: %.2f", t, p, pos, vel);
            pTxCharacteristic->setValue(buffer);
            pTxCharacteristic->notify();
        }
    }

    if (deviceConnected) {
        std::string rxValue = pRxCharacteristic->getValue();
        if (rxValue.length() > 0) {
            // command format "P15.5" for Kp, "I0.1" for Ki, "D1.2" for Kd
            char type = rxValue[0];
            float value = atof(rxValue.substr(1).c_str());
            if (type == 'P') anglePID.setKp(value);
            else if (type == 'I') anglePID.setKi(value);
            else if (type == 'D') anglePID.setKd(value);

            Serial.printf("Received PID update: %c = %.2f\n", type, value);
            pRxCharacteristic->setValue("");
        }
    }
}


void controlTask(void *pvParameters) {
    mpu.initialize();
    mpu.dmpInitialize();
    
    mpu.setXAccelOffset(-2396); mpu.setYAccelOffset(715); mpu.setZAccelOffset(1084);
    mpu.setXGyroOffset(584); mpu.setYGyroOffset(-699); mpu.setZGyroOffset(122);

    mpu.setDMPEnabled(true);
    Serial.println("MPU6050 DMP initialized and enabled!");
    vTaskDelay(pdMS_TO_TICKS(1000)); // Allow some time for the DMP to stabilize

    float target_position = 0;
    float target_speed = 0;
    float target_angle = 0;
    static long lastLeftCount = 0, lastRightCount = 0;

    uint8_t fifoBuffer[64];
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];
    unsigned long lastTime = micros();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 5 ms control loop

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        const float dt = 0.005f; // 5 ms loop time
        
        long left, right;
        noInterrupts();
        left = leftEncoderCount;
        right = rightEncoderCount;
        interrupts();
        long dleft = left - lastLeftCount;
        long dright = right - lastRightCount;
        lastLeftCount = left;
        lastRightCount = right;

        float dx = ((dleft + dright) / 2.0) * DIST_PER_TICK;
        v_position += dx; // Update position based on encoder counts
        float raw_velocity = dx / dt;
        v_velocity = 0.8 * v_velocity + 0.2 * raw_velocity; // Simple low-pass filter for velocity

        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                
            v_theta = ypr[1] * 180/M_PI; // Convert pitch to degrees
                
            if (abs(v_theta) > 45) {
            v_pwm = 0;
            driveMotors(0);
            positionPID.reset();
            velocityPID.reset();
            anglePID.reset();
            v_position = 0; // Reset position to prevent integral windup
            v_velocity = 0; // Reset velocity to prevent integral windup
            target_speed = 0;
            target_angle = 0;
            } else {
                //position loop
                target_speed = positionPID.compute(target_position, v_position, dt);
                //velocity loop
                target_angle = velocityPID.compute(target_speed, v_velocity, dt);
                //angle loop
                v_pwm = anglePID.compute(target_angle, v_theta, dt);
                driveMotors(v_pwm);
            }
        }
    }
}


void driveMotors(float pwm) {
  int deadzone = 100; // Unified deadzone for simplicity
  if (pwm > 0) pwm += deadzone;
  else if (pwm < 0) pwm -= deadzone;
  
  pwm = constrain(pwm, -1023, 1023);

  if (pwm >= 0) {
    ledcWrite(0, 0); ledcWrite(1, int(pwm));
    ledcWrite(2, int(pwm)); ledcWrite(3, 0);
  } else {
    ledcWrite(0, abs((int)pwm)); ledcWrite(1, 0);
    ledcWrite(2, 0); ledcWrite(3, abs((int)pwm));
  }
}

void setupBLE() {
    NimBLEDevice::init("ESP32_SelfBalancingBot");
    NimBLEDevice::setMTU(100);
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ConnectionHandler());
    NimBLEService* pService = pServer->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    // TX Characteristic (ESP32 -> Phone)
    pTxCharacteristic = pService->createCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", NIMBLE_PROPERTY::NOTIFY);
    // RX Characteristic (Phone -> ESP32)
    pRxCharacteristic = pService->createCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", NIMBLE_PROPERTY::WRITE);
    pRxCharacteristic->setCallbacks(new TuningHandler());
    pService->start();
    pServer->getAdvertising()->start();
}