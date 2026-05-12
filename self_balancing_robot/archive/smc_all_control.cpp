#include <Arduino.h>
#include <NimBLEDevice.h>
#include <math.h>
#include "smc_all.h" // Kendi yazdığın SMC kütüphanesi
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

portMUX_TYPE myMux = portMUX_INITIALIZER_UNLOCKED;

// BLE Tanımlamaları
NimBLEServer* pServer = NULL;
NimBLECharacteristic* pTxCharacteristic = NULL;
NimBLECharacteristic* pRxCharacteristic = NULL;
bool deviceConnected = false;

class ConnectionHandler: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(NimBLEServer* pServer) { deviceConnected = false; }
};

// SMC Tanımlaması (Başlangıç değerleri)
SMC smc(-1023,1023);
enum mode {Cls = 1, Hyb = 2, Hie = 3};
mode activeMode = Hie;

MPU6050 mpu;
volatile long leftEncoderCount = 0, rightEncoderCount = 0;
volatile float v_theta = 0, v_position = 0, v_pwm = 0, v_velocity = 0;

// Pin Tanımlamaları (Önceki PID kodunla aynı)
const int R_PWM_L = 2, L_PWM_L = 4; const int R_PWM_R = 5, L_PWM_R = 18;
const int leftEncA = 19, leftEncB = 23, rightEncA = 16, rightEncB = 17;
const int SDA_PIN = 21, SCL_PIN = 22;

const float DIST_PER_TICK = (0.088f * M_PI) / 224.0f; 

bool systemEnabled = false;

void setupBLE();
void driveMotors(float pwm);
void controlTask(void *pvParameters);

void setup() {
    Serial.begin(115200);
    setupBLE();

    smc.setParamsCls(10.0, 30.0, 1500.0, 60.0, 800.0, 2.0);
    smc.setParamsHyb(30.0, 60.0, 5.0, 12.0, 1.0/3.0, 1500.0/60.0);
    smc.setParamsHie(30.0, 60.0, 1.0/3.0, 1500.0/60.0, 800.0, 2.0);
    
    // PWM Ayarları (10-bit: 0-1023)
    ledcSetup(0, 20000, 10); ledcSetup(1, 20000, 10); 
    ledcSetup(2, 20000, 10); ledcSetup(3, 20000, 10);
    ledcAttachPin(R_PWM_L, 0); ledcAttachPin(L_PWM_L, 1);
    ledcAttachPin(R_PWM_R, 2); ledcAttachPin(L_PWM_R, 3);

    pinMode(leftEncA, INPUT_PULLUP); pinMode(leftEncB, INPUT_PULLUP);
    pinMode(rightEncA, INPUT_PULLUP); pinMode(rightEncB, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(leftEncA), []() {
        (digitalRead(leftEncB)) ? leftEncoderCount-- : leftEncoderCount++;
    }, RISING);
    attachInterrupt(digitalPinToInterrupt(rightEncA), []() {
        (digitalRead(rightEncB)) ? rightEncoderCount++ : rightEncoderCount--;
    }, RISING);

    Wire.begin(SDA_PIN, SCL_PIN, 400000); 
    xTaskCreatePinnedToCore(controlTask, "controlTask", 10000, NULL, 1, NULL, 0); 
}

void loop() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
        lastPrint = millis();
        if (deviceConnected) {
            portENTER_CRITICAL(&myMux);
            float t = v_theta * 180/M_PI;
            float p = v_pwm;
            float pos = v_position;
            float vel = v_velocity;
            portEXIT_CRITICAL(&myMux);

            char buffer[100];
            int len = snprintf(buffer, sizeof(buffer), "The:%.2f, PWM:%.0f, Pos:%.2f, Vel:%.2f\n", t, p, pos, vel);
            pTxCharacteristic->setValue((uint8_t*)buffer, len);
            pTxCharacteristic->notify();
        }
    }

    if (deviceConnected) {
        std::string rxValue = pRxCharacteristic->getValue();
        if (!rxValue.empty()) {
            char type = rxValue[0];
            float val = atof(rxValue.substr(1).c_str());

            if (type == 'S') { // start
                systemEnabled = true;
                char confirmBuf[64];
                int cLen = snprintf(confirmBuf, sizeof(confirmBuf), ">> System Enabled\n");
                pTxCharacteristic->setValue((uint8_t*)confirmBuf, cLen);
                pTxCharacteristic->notify();
            } else if (type == 'X') {  // stop
                systemEnabled = false; 
                driveMotors(0);
                char confirmBuf[64];
                int cLen = snprintf(confirmBuf, sizeof(confirmBuf), ">> System Disabled\n");
                pTxCharacteristic->setValue((uint8_t*)confirmBuf, cLen);
                pTxCharacteristic->notify();
            } else if (type == 'M') { // mode change
                activeMode = (mode)((int)val);
                char confirmBuf[64];
                int cLen = snprintf(confirmBuf, sizeof(confirmBuf), ">> Mode Set To %d\n", (int)activeMode);
                pTxCharacteristic->setValue((uint8_t*)confirmBuf, cLen);
                pTxCharacteristic->notify();
            }
            // Parameter update
            else if (type == '1') {
                if (activeMode == Cls) smc.setClsC1(val);
                else if (activeMode == Hyb) smc.setHybK1(val);
                else if (activeMode == Hie) smc.setHieK1(val);
            } else if (type == '2') {
                if (activeMode == Cls) smc.setClsC2(val);
                else if (activeMode == Hyb) smc.setHybK2(val);
                else if (activeMode == Hie) smc.setHieK2(val);
            } else if (type == '3') {
                if (activeMode == Cls) smc.setClsC3(val);
                else if (activeMode == Hyb) smc.setHybK3(val);
                else if (activeMode == Hie) smc.setHieLambda1(val);
            } else if (type == '4') {
                if (activeMode == Cls) smc.setClsC4(val);
                else if (activeMode == Hyb) smc.setHybK4(val);
                else if (activeMode == Hie) smc.setHieLambda2(val);
            } else if (type == '5') {
                if (activeMode == Cls) smc.setClsEta(val);
                else if (activeMode == Hyb) smc.setHybLambda1(val);
                else if (activeMode == Hie) smc.setHieEta(val);
            } else if (type == '6') {
                if (activeMode == Cls) smc.setClsPhi(val);
                else if (activeMode == Hyb) smc.setHybLambda2(val);
                else if (activeMode == Hie) smc.setHiePhi(val);
            }
            
            pRxCharacteristic->setValue(""); 
        }
    }
}

void controlTask(void *pvParameters) {
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setXAccelOffset(-2547); mpu.setYAccelOffset(857); mpu.setZAccelOffset(1103);
    mpu.setXGyroOffset(591); mpu.setYGyroOffset(-665); mpu.setZGyroOffset(139);
    mpu.setDMPEnabled(true);

    uint8_t fifoBuffer[64];
    Quaternion q; VectorFloat gravity; float ypr[3];
    float last_theta = 0; long lastLeftCount = 0, lastRightCount = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5); 

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        const float dt = 0.005f; 

        // Enkoder Okuma ve Hız Hesaplama
        noInterrupts();
        long left = leftEncoderCount; long right = rightEncoderCount;
        interrupts();
        
        float dx = (((left - lastLeftCount) + (right - lastRightCount)) / 2.0f) * DIST_PER_TICK;
        lastLeftCount = left; lastRightCount = right;
        
        v_position += dx;
        float raw_velocity = dx / dt;
        v_velocity = 0.8f * v_velocity + 0.2f * raw_velocity;

        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            v_theta = ypr[1]; // Radyan cinsinden pitch
            float theta_dot = (v_theta - last_theta) / dt;
            last_theta = v_theta;

            if (systemEnabled && abs(v_theta) < 45*M_PI/180) {
                switch (activeMode) {
                    case Cls: v_pwm = smc.computeCls(v_position, v_velocity, v_theta, theta_dot); break;
                    case Hyb: v_pwm = smc.computeHyb(v_position, v_velocity, v_theta, theta_dot); break;
                    case Hie: v_pwm = smc.computeHie(v_position, v_velocity, v_theta, theta_dot); break;
                }
                driveMotors(v_pwm);
            } else {
                driveMotors(0);
                v_position = 0; v_velocity = 0;
            }
        }
    }
}

void driveMotors(float pwm) {
    int deadzone = 180; // ESP32 PWM 1023 üzerinden
    if (abs(pwm) > 1.0) {
        pwm += (pwm > 0) ? deadzone : -deadzone;
    } else pwm = 0;

    pwm = constrain(pwm, -1023, 1023);
    int duty = abs((int)pwm);

    if (pwm > 0) {
        ledcWrite(0, 0); ledcWrite(1, duty);
        ledcWrite(2, duty); ledcWrite(3, 0);
    } else if (pwm < 0) {
        ledcWrite(0, duty); ledcWrite(1, 0);
        ledcWrite(2, 0); ledcWrite(3, duty);
    } else {
        ledcWrite(0, 0); ledcWrite(1, 0);
        ledcWrite(2, 0); ledcWrite(3, 0);
    }
}

void setupBLE() {
    NimBLEDevice::init("ESP32_SMC_Bot");
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ConnectionHandler());
    NimBLEService* pService = pServer->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    pTxCharacteristic = pService->createCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", NIMBLE_PROPERTY::NOTIFY);
    pRxCharacteristic = pService->createCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", NIMBLE_PROPERTY::WRITE);
    pService->start();
    pServer->getAdvertising()->start();
}