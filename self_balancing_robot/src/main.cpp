#include <Arduino.h>
#include <NimBLEDevice.h> // bluetooth
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

//pid positionPID(0.5, 0.0, 0.02, -2,2); // PID for position control
//pid velocityPID(0.6, 0.0, 0.02, -5, 5); // PID for velocity control
//pid anglePID(25.0,0.0,0.2,-1023,1023);
pid positionPID(0.0018, 0, 0.0507, -2, 2);
pid anglePID(1500, 114.6763, 10, -1023, 1023);

MPU6050 mpu;
volatile long leftEncoderCount = 0, rightEncoderCount = 0;
volatile float v_theta = 0, v_position = 0, v_pwm = 0, v_velocity = 0;
volatile float target_angle = 0;

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
            float theta_deg = t * 180/M_PI;
            char buffer[100];
            int len = snprintf(buffer, sizeof(buffer), "Theta: %.2f, PWM: %.0f, Pos: %.2f, Vel: %.2f\n", theta_deg, p, pos, vel);
            if (deviceConnected && len > 0) {
                pTxCharacteristic->setValue((uint8_t*)buffer, len);
                pTxCharacteristic->notify();
            }
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
        } else if (type == 'X') {
            systemEnabled = false;
            driveMotors(0);
            positionPID.reset();
            //velocityPID.reset();
            anglePID.reset();
            char confirmBuf[64];
            int cLen = snprintf(confirmBuf, sizeof(confirmBuf), ">> System Disabled\n");
            pTxCharacteristic->setValue((uint8_t*)confirmBuf, cLen);
            pTxCharacteristic->notify();
        }
        if (type == 'W') { 
        target_angle = -0.5; 
        char confirmBuf[64];
        int cLen = snprintf(confirmBuf, sizeof(confirmBuf), ">> Moving Forward\n");
        pTxCharacteristic->setValue((uint8_t*)confirmBuf, cLen);
        pTxCharacteristic->notify();
} 
        else if (type == 'K') { 
        target_angle = 0; 
        char confirmBuf[64];
        int cLen = snprintf(confirmBuf, sizeof(confirmBuf), ">> Stopping/Balancing\n");
        pTxCharacteristic->setValue((uint8_t*)confirmBuf, cLen);
        pTxCharacteristic->notify();
}
        else if (type == 'P' || type == 'I' || type == 'D') {
            float value = atof(rxValue.substr(1).c_str());
            
            if (type == 'P') anglePID.setKp(value);
            else if (type == 'I') anglePID.setKi(value);
            else if (type == 'D') anglePID.setKd(value);

            char confirmBuf[64];
            int cLen = snprintf(confirmBuf, sizeof(confirmBuf), ">> Update: %c set to %.2f\n", type, value);
            pTxCharacteristic->setValue((uint8_t*)confirmBuf, cLen);
            pTxCharacteristic->notify();
            
            Serial.printf("Received PID update: %c = %.2f\n", type, value);
        }

        // 2. IMPORTANT: Clear the characteristic so we don't process it again
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
    Serial.println("MPU6050 DMP initialized and enabled!");
    vTaskDelay(pdMS_TO_TICKS(1000)); // Allow some time for the DMP to stabilize

    float target_position = 0;
    float target_speed = 0;
    //float target_angle = 0;
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
                
            //v_theta = ypr[1] * 180/M_PI; // Convert pitch to degrees
            v_theta = ypr[1];

            if (systemEnabled && abs(v_theta) < 45*M_PI/180) {
                //position loop
                target_angle = positionPID.compute(target_position, v_position, dt);
                //velocity loop
                //target_angle = velocityPID.compute(target_speed, v_velocity, dt);
                //angle loop
                v_pwm = -anglePID.compute(target_angle, v_theta, dt);
                driveMotors(v_pwm);
            } else {
                v_pwm = 0;
                driveMotors(0);
                positionPID.reset();
                //velocityPID.reset();
                anglePID.reset();
                v_position = 0; // Reset position to prevent integral windup
                v_velocity = 0; // Reset velocity to prevent integral windup
                target_speed = 0;
                target_angle = 0;
            }
        }
    }
}


void driveMotors(float pwm) {
  int deadzone = 180; 

  if (abs(pwm) > 1.0) {
    if (pwm > 0) pwm += deadzone;
    else if (pwm < 0) pwm -= deadzone;
  } else {
    pwm = 0; // If within -1 to 1, consider it as zero to prevent jitter
  }

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