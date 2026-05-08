#include <Arduino.h>
#include <NimBLEDevice.h>
#include "lqr.h" 
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

// LQR Kurulumu (Katsayıları sistemine göre ayarlamalısın)
// Durumlar: [Mesafe (x), Hız (x_dot), Açı (theta), Açısal Hız (theta_dot)]
float k1 = -70.7107, k2 = -50.1869, k3 = -162.3399, k4 = -24.6664; 
lqr lqr1(k1, k2, k3, k4, -1023, 1023);

MPU6050 mpu;
volatile long leftEncoderCount = 0, rightEncoderCount = 0;
volatile float v_theta = 0, v_theta_dot = 0, v_position = 0, v_velocity = 0;
volatile float v_pwm = 0;
float target_x = 0; // LQR için hedef pozisyon

// Pin ve Fiziksel Sabitler
const int R_PWM_L = 2, L_PWM_L = 4; 
const int R_PWM_R = 5, L_PWM_R = 18;
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
            float t = v_theta;
            float p = v_pwm;
            float pos = v_position;
            float vel = v_velocity;
            portEXIT_CRITICAL(&myMux);

            char buffer[100];
            float t_deg = t * 180 / M_PI; // Radyanı dereceye çevir
            int len = snprintf(buffer, sizeof(buffer), "Ang:%.2f, PWM:%.0f, Pos:%.2f, Vel:%.2f\n", t_deg, p, pos, vel);
            if (deviceConnected && len>0){
            pTxCharacteristic->setValue((uint8_t*)buffer, len);
            pTxCharacteristic->notify();}
        }
    }

    if (deviceConnected) {
        std::string rxValue = pRxCharacteristic->getValue();
        if (!rxValue.empty()) {
            char type = rxValue[0];
            if (type == 'S') {
                systemEnabled = true;
                char confirmBuf[64];
                int cLen = snprintf(confirmBuf, sizeof(confirmBuf), ">> System Enabled\n");
                pTxCharacteristic->setValue((uint8_t*)confirmBuf, cLen);
                pTxCharacteristic->notify();
            }
            else if (type == 'X') {
                systemEnabled = false;
                driveMotors(0);
                v_position = 0;
                char confirmBuf[64];
                int cLen = snprintf(confirmBuf, sizeof(confirmBuf), ">> System Disabled\n");
                pTxCharacteristic->setValue((uint8_t*)confirmBuf, cLen);
                pTxCharacteristic->notify();
            }
            // LQR Katsayılarını BLE üzerinden güncelleme (Opsiyonel)
            if (type == 'P') k1 = atof(rxValue.substr(1).c_str()); // K1 günceller
            if (type == 'Q') k2 = atof(rxValue.substr(1).c_str()); // K2 günceller
            if (type == 'R') k3 = atof(rxValue.substr(1).c_str()); // K3 günceller
            if (type == 'S') k4 = atof(rxValue.substr(1).c_str()); // K4 günceller

            pRxCharacteristic->setValue(""); 
        }
    }
}

void controlTask(void *pvParameters) {
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setDMPEnabled(true);
    
    // Sensör Ofsetleri (Kendi robotuna göre kalibre etmelisin)
    mpu.setXAccelOffset(-2547); mpu.setYAccelOffset(857); mpu.setZAccelOffset(1103);
    mpu.setXGyroOffset(591); mpu.setYGyroOffset(-665); mpu.setZGyroOffset(139);

    uint8_t fifoBuffer[64];
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];
    float last_theta = 0;
    long lastLeftCount = 0, lastRightCount = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200Hz

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        const float dt = 0.005f; 
        
        // 1. Durum Tahmini (Enkoderler)
        long left, right;
        portENTER_CRITICAL(&myMux);
        left = leftEncoderCount;
        right = rightEncoderCount;
        portEXIT_CRITICAL(&myMux);

        long dleft = left - lastLeftCount;
        long dright = right - lastRightCount;
        lastLeftCount = left;
        lastRightCount = right;

        float dx = ((dleft + dright) / 2.0f) * DIST_PER_TICK;
        v_position += dx;
        float raw_velocity = dx / dt;
        v_velocity = 0.9f * v_velocity + 0.1f * raw_velocity; // LPF

        // 2. Durum Tahmini (IMU)
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            v_theta = ypr[1]; // Radyan cinsinden (LQR genelde radyan tercih eder)
            v_theta_dot = (v_theta - last_theta) / dt;
            last_theta = v_theta;

            // 3. LQR Hesaplama
            if (systemEnabled && abs(v_theta * 180/M_PI) < 45) {
                // LQR Formülü: u = -(K1*x + K2*x_dot + K3*theta + K4*theta_dot)
                // LQR nesneniz bu 4 parametreyi alacak şekilde güncellenmiştir.
                v_pwm = lqr1.compute(v_position - target_x, v_velocity, v_theta, v_theta_dot);
                driveMotors(v_pwm);
            } else {
                v_pwm = 0;
                driveMotors(0);
                v_position = 0; // Robot düştüğünde konumu sıfırla
            }
        }
    }
}

void driveMotors(float pwm) {
    int deadzone = 180; // Motorlarının kalkış eşiğine göre ayarla
    
    if (abs(pwm) > 1.0) {
        pwm = (pwm > 0) ? pwm + deadzone : pwm - deadzone;
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
    NimBLEDevice::init("ESP32_SelfBalancingBot");
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ConnectionHandler());
    NimBLEService* pService = pServer->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    pTxCharacteristic = pService->createCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", NIMBLE_PROPERTY::NOTIFY);
    pRxCharacteristic = pService->createCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", NIMBLE_PROPERTY::WRITE);
    pService->start();
    pServer->getAdvertising()->start();
}