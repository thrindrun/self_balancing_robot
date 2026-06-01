#include <Arduino.h>
#include <NimBLEDevice.h>
#include <math.h>
#include "smc.h" // Kendi yazdığın SMC kütüphanesi
#include "pid.h" // Kendi yazdığın PID kütüphanesi
#include "lqr.h" // Kendi yazdığın LQR kütüphanesi
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
lqr lqr1 = lqr(-10, -30, -2000, -60, -1023, 1023);
pid positionPID = pid(0.0018, 0, 0.0507, -4*M_PI/180, 4*M_PI/180);
pid anglePID = pid(2000.0, 114.6763, 10.0, -1023, 1023);
// SMC Tanımlaması (Başlangıç değerleri)
SMC smc(-1023,1023);
enum mode {Cls = 1, Hyb = 2, Hie = 3, PID = 4, LQR = 5};
mode activeMode = Hyb;
enum angVelocityType {Gyro = 1, Diff = 2};
angVelocityType angleVelocityType = Diff;

MPU6050 mpu;
volatile long leftEncoderCount = 0, rightEncoderCount = 0;
volatile float v_theta = 0, v_position = 0, v_pwm = 0, v_velocity = 0, v_theta_dot = 0;

// Pin Tanımlamaları (Önceki PID kodunla aynı)
const int R_PWM_L = 2, L_PWM_L = 4; const int R_PWM_R = 5, L_PWM_R = 18;
const int leftEncA = 19, leftEncB = 23, rightEncA = 16, rightEncB = 17;
const int SDA_PIN = 21, SCL_PIN = 22;

const float DIST_PER_TICK = (0.088f * M_PI) / 224.0f; 

bool systemEnabled = false;

void setupBLE();
void driveMotors(float pwm);
void controlTask(void *pvParameters);
void sendConfirmation(const char* message);

void setup() {
    Serial.begin(115200);
    setupBLE();

    smc.setParamsCls(10.0, 30.0, -1500.0, -60.0, 800.0, 2.0);
    smc.setParamsHyb(30.0, 60.0, 5.0, 10.0, 1.0/3.0, 1500.0/60.0);
    smc.setParamsHie(30.0, -40.0, 1.0/3.0, 1000.0/60.0, 600.0, 10.0);
    
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
            if (type == '\0' || type == '\n' || type == '\r') {
                pRxCharacteristic->setValue("");
                return;
            }
            float val = atof(rxValue.substr(1).c_str());

            switch (type) { 
                case 'S':
                    systemEnabled = true;
                    sendConfirmation(">> System enabled\n");
                    break;
                case 'X':
                    systemEnabled = false;
                    driveMotors(0);
                    sendConfirmation(">> System disabled\n");
                    break;
                case 'M': {
                    activeMode = (mode)((int)val);
                    char modeBuf[64];
                    snprintf(modeBuf, sizeof(modeBuf), ">> Mode set to %d\n", (int)activeMode);
                    sendConfirmation(modeBuf);
                    break;
                }
                case 'A': {
                    angleVelocityType = (angVelocityType)((int)val);
                    char typeBuf[64];
                    snprintf(typeBuf, sizeof(typeBuf), ">> Angle velocity type set to %d\n", (int)angleVelocityType);
                    sendConfirmation(typeBuf);
                    break;
                }
                case '1':
                    switch (activeMode) {
                        case Cls: smc.setClsC1(val); break;
                        case Hyb: smc.setHybK1(val); break;
                        case Hie: smc.setHieK1(val); break;
                        case LQR: lqr1.setK1(val); break;
                        case PID: anglePID.setKp(val); break;
                    }
                    break;
                case '2':
                    switch (activeMode) {
                        case Cls: smc.setClsC2(val); break;
                        case Hyb: smc.setHybK2(val); break;
                        case Hie: smc.setHieK2(val); break;
                        case LQR: lqr1.setK2(val); break;
                        case PID: anglePID.setKi(val); break;
                    }
                    break;
                case '3':
                    switch (activeMode) {
                        case Cls: smc.setClsC3(val); break;
                        case Hyb: smc.setHybK3(val); break;
                        case Hie: smc.setHieLambda1(val); break;
                        case LQR: lqr1.setK3(val); break;
                        case PID: anglePID.setKd(val); break;
                    }
                    break;
                case '4':
                    switch (activeMode) {
                        case Cls: smc.setClsC4(val); break;
                        case Hyb: smc.setHybK4(val); break;
                        case Hie: smc.setHieLambda2(val); break;
                        case LQR: lqr1.setK4(val); break;
                        case PID: positionPID.setKp(val); break;
                    }
                    break;
                case '5':
                    switch (activeMode) {
                        case Cls: smc.setClsEta(val); break;
                        case Hyb: smc.setHybLambda1(val); break;
                        case Hie: smc.setHieEta(val); break;
                        case LQR: break;
                        case PID: positionPID.setKi(val); break;
                    }
                    break;
                case '6':
                    switch (activeMode) {
                        case Cls: smc.setClsPhi(val); break;
                        case Hyb: smc.setHybLambda2(val); break;
                        case Hie: smc.setHiePhi(val); break;
                        case LQR: break;
                        case PID: positionPID.setKd(val); break;
                    }
                    break;
                default:
                    sendConfirmation(">> Unknown command\n");
                    break;
            }
            pRxCharacteristic->setValue(""); 
        }
    }
}

void controlTask(void *pvParameters) {
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setXAccelOffset(-2455); mpu.setYAccelOffset(742); mpu.setZAccelOffset(1111);
    mpu.setXGyroOffset(634); mpu.setYGyroOffset(-698); mpu.setZGyroOffset(134);
    mpu.setDMPEnabled(true);

    uint8_t fifoBuffer[64];
    Quaternion q; VectorFloat gravity; float ypr[3];
    float last_theta = 0; long lastLeftCount = 0, lastRightCount = 0;
    float target_position = 0; float target_angle = 0;
    VectorInt16 gyro;

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
            mpu.dmpGetGyro(&gyro, fifoBuffer);
            v_theta = ypr[1]; // Radyan cinsinden pitch
            switch (angleVelocityType) {
                case Gyro: v_theta_dot = (gyro.y / 131.0) * (M_PI / 180.0); break;
                case Diff: v_theta_dot = (v_theta - last_theta) / dt; break;
            }
            last_theta = v_theta;

            if (systemEnabled && abs(v_theta) < 45*M_PI/180) {
                switch (activeMode) {
                    case Cls: v_pwm = -smc.computeCls(v_position, v_velocity, v_theta, v_theta_dot); break;
                    case Hyb: v_pwm = -smc.computeHyb(v_position, v_velocity, v_theta, v_theta_dot); break;
                    case Hie: v_pwm = -smc.computeHie(v_position, v_velocity, v_theta, v_theta_dot); break;
                    case LQR: v_pwm = -lqr1.compute(v_position, v_velocity, v_theta, v_theta_dot); break;
                    case PID: {
                        target_angle = positionPID.compute(target_position, v_position, dt);
                        v_pwm = -anglePID.compute(target_angle, v_theta, dt);
                        break;
                    }
                }
                driveMotors(v_pwm);
            } else {
                driveMotors(0);
                v_position = 0; v_velocity = 0;
                positionPID.reset(); anglePID.reset();
                target_position = 0; target_angle = 0;
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
    NimBLEDevice::setMTU(100);
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ConnectionHandler());
    NimBLEService* pService = pServer->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    pTxCharacteristic = pService->createCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", NIMBLE_PROPERTY::NOTIFY);
    pRxCharacteristic = pService->createCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", NIMBLE_PROPERTY::WRITE);
    pService->start();
    pServer->getAdvertising()->start();
}

void sendConfirmation(const char* message) {
    if (deviceConnected) {
        int len = strlen(message);
        pTxCharacteristic->setValue((uint8_t*)message, len);
        pTxCharacteristic->notify();
    }
}