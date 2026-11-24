/**
 * LiteWing ESP32 Gesture Glove Controller Firmware
 *
 * Senses hand orientation via MPU6050, reads deadman sensor (LDR/Flex)
 * and emergency button, packages data into binary glovePacket_t, and
 * streams over Wi-Fi UDP port 2391 to the LiteWing drone.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "glove_protocol.h"
#include "config.h"

/* MPU6050 Registers */
#define MPU6050_ADDR         0x68
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

WiFiUDP udp;
IPAddress droneIP;

/* Sensor calibration offsets */
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;

/* Filtered angles */
float handRoll = 0.0f;
float handPitch = 0.0f;
float handYawRate = 0.0f;

/* Timing */
unsigned long lastLoopTime = 0;
unsigned long lastSendTime = 0;
uint8_t packetSequence = 0;

/* Operational flags */
bool flightModeAltHold = false;
bool emergencyAsserted = false;
bool isConnected = false;

/* Button debouncing */
unsigned long lastModeBtnPress = 0;

void writeMPU(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

bool initMPU6050() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    delay(50);

    /* Wake up MPU6050 */
    writeMPU(MPU6050_PWR_MGMT_1, 0x00);
    delay(20);

    /* Set sample rate divider = 4 (200Hz internal) */
    writeMPU(MPU6050_SMPLRT_DIV, 0x04);
    /* DLPF 44Hz */
    writeMPU(MPU6050_CONFIG, 0x03);
    /* Gyro scale ±500 deg/s */
    writeMPU(MPU6050_GYRO_CONFIG, 0x08);
    /* Accel scale ±4g */
    writeMPU(MPU6050_ACCEL_CONFIG, 0x08);

    /* Quick connection test */
    Wire.beginTransmission(MPU6050_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("[ERROR] MPU6050 connection failed!");
        return false;
    }
    Serial.println("[OK] MPU6050 initialized.");
    return true;
}

void calibrateMPU6050() {
    Serial.println("[INFO] Calibrating MPU6050... Keep hand level and steady!");
    digitalWrite(PIN_STATUS_LED, HIGH);

    const int SAMPLES = 300;
    long gX = 0, gY = 0, gZ = 0;
    long aX = 0, aY = 0, aZ = 0;

    for (int i = 0; i < SAMPLES; i++) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(MPU6050_ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU6050_ADDR, 14, true);

        if (Wire.available() >= 14) {
            int16_t rawAX = Wire.read() << 8 | Wire.read();
            int16_t rawAY = Wire.read() << 8 | Wire.read();
            int16_t rawAZ = Wire.read() << 8 | Wire.read();
            Wire.read(); Wire.read(); /* Skip temp */
            int16_t rawGX = Wire.read() << 8 | Wire.read();
            int16_t rawGY = Wire.read() << 8 | Wire.read();
            int16_t rawGZ = Wire.read() << 8 | Wire.read();

            aX += rawAX;
            aY += rawAY;
            aZ += rawAZ;
            gX += rawGX;
            gY += rawGY;
            gZ += rawGZ;
        }
        delay(4);
    }

    gyroBiasX = (float)gX / SAMPLES;
    gyroBiasY = (float)gY / SAMPLES;
    gyroBiasZ = (float)gZ / SAMPLES;
    accelBiasX = (float)aX / SAMPLES;
    accelBiasY = (float)aY / SAMPLES;
    /* Expected 1g at ±4g scale is 8192 LSB/g */
    accelBiasZ = ((float)aZ / SAMPLES) - 8192.0f;

    digitalWrite(PIN_STATUS_LED, LOW);
    Serial.println("[OK] Calibration complete.");
}

void connectToDrone() {
    Serial.println("[INFO] Searching for LiteWing drone AP...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    String targetSSID = "";
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++) {
        String ssid = WiFi.SSID(i);
        if (ssid.startsWith(DRONE_SSID_PREFIX)) {
            targetSSID = ssid;
            Serial.printf("[FOUND] Drone AP: %s (RSSI: %d)\n", targetSSID.c_str(), WiFi.RSSI(i));
            break;
        }
    }

    if (targetSSID.length() == 0) {
        Serial.println("[WARN] Specific LiteWing AP not found in scan, falling back to default prefix.");
        targetSSID = DRONE_SSID_PREFIX;
    }

    Serial.printf("[INFO] Connecting to: %s\n", targetSSID.c_str());
    WiFi.begin(targetSSID.c_str(), DRONE_WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 25) {
        delay(300);
        digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        isConnected = true;
        digitalWrite(PIN_STATUS_LED, HIGH);
        droneIP.fromString(DRONE_IP_ADDRESS);
        Serial.println("\n[OK] Connected to Drone!");
        Serial.printf("[INFO] Glove IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("[INFO] Target Drone IP: %s:%d\n", DRONE_IP_ADDRESS, DRONE_UDP_PORT);
    } else {
        Serial.println("\n[ERROR] Connection failed. Will retry in loop.");
        isConnected = false;
        digitalWrite(PIN_STATUS_LED, LOW);
    }
}

void updateOrientation(float dt) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);

    if (Wire.available() < 14) return;

    int16_t rawAX = Wire.read() << 8 | Wire.read();
    int16_t rawAY = Wire.read() << 8 | Wire.read();
    int16_t rawAZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); /* Skip temp */
    int16_t rawGX = Wire.read() << 8 | Wire.read();
    int16_t rawGY = Wire.read() << 8 | Wire.read();
    int16_t rawGZ = Wire.read() << 8 | Wire.read();

    /* Scale: Accel ±4g (8192 LSB/g), Gyro ±500 deg/s (65.5 LSB/(deg/s)) */
    float aX = (rawAX - accelBiasX) / 8192.0f;
    float aY = (rawAY - accelBiasY) / 8192.0f;
    float aZ = (rawAZ - accelBiasZ) / 8192.0f;

    float gX = (rawGX - gyroBiasX) / 65.5f;
    float gY = (rawGY - gyroBiasY) / 65.5f;
    float gZ = (rawGZ - gyroBiasZ) / 65.5f;

    /* Accel pitch and roll in degrees */
    float accRoll  = atan2(aY, aZ) * 180.0f / PI;
    float accPitch = atan2(-aX, sqrt(aY * aY + aZ * aZ)) * 180.0f / PI;

    /* Complementary filter */
    handRoll = COMP_FILTER_ALPHA * (handRoll + gX * dt) + (1.0f - COMP_FILTER_ALPHA) * accRoll;
    handPitch = COMP_FILTER_ALPHA * (handPitch + gY * dt) + (1.0f - COMP_FILTER_ALPHA) * accPitch;
    handYawRate = gZ;
}

bool readDeadmanSensor() {
    int raw = analogRead(PIN_DEADMAN_ADC);
#if DEADMAN_INVERT
    return (raw < DEADMAN_ADC_THRESHOLD);
#else
    return (raw > DEADMAN_ADC_THRESHOLD);
#endif
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== LiteWing Gesture Glove Starting ===");

    pinMode(PIN_STATUS_LED, OUTPUT);
    pinMode(PIN_EMERGENCY_BTN, INPUT_PULLUP);
    pinMode(PIN_MODE_BTN, INPUT_PULLUP);
    pinMode(PIN_DEADMAN_ADC, INPUT);

    digitalWrite(PIN_STATUS_LED, LOW);

    if (initMPU6050()) {
        calibrateMPU6050();
    }

    connectToDrone();
    lastLoopTime = millis();
    lastSendTime = millis();
}

void loop() {
    unsigned long now = millis();
    float dt = (now - lastLoopTime) * 0.001f;
    if (dt <= 0.0f) dt = 0.001f;
    lastLoopTime = now;

    /* Check Wi-Fi reconnection if lost */
    if (WiFi.status() != WL_CONNECTED) {
        isConnected = false;
        digitalWrite(PIN_STATUS_LED, LOW);
        if (now % 2000 < 50) {
            connectToDrone();
        }
        delay(10);
        return;
    }

    updateOrientation(dt);

    /* Read Emergency E-Stop button (Active LOW) */
    emergencyAsserted = (digitalRead(PIN_EMERGENCY_BTN) == LOW);

    /* Read Mode Button toggle (Active LOW with debounce) */
    if (digitalRead(PIN_MODE_BTN) == LOW && (now - lastModeBtnPress > 400)) {
        flightModeAltHold = !flightModeAltHold;
        lastModeBtnPress = now;
        Serial.printf("[MODE] Switched to %s mode\n", flightModeAltHold ? "Altitude-Assist" : "Manual");
    }

    /* Read Deadman switch */
    bool deadmanActive = readDeadmanSensor();

    /* Send packet at configured interval (50 Hz / 20ms) */
    if (now - lastSendTime >= GLOVE_SEND_INTERVAL_MS) {
        lastSendTime = now;

        glovePacket_t pkt;
        pkt.magic0 = GLOVE_MAGIC_BYTE_0;
        pkt.magic1 = GLOVE_MAGIC_BYTE_1;
        pkt.version = GLOVE_PROTOCOL_VERSION;
        pkt.sequence = packetSequence++;
        pkt.timestamp_ms = now;

        /* Apply neutral deadzones and clamp safe limits */
        float outRoll = handRoll;
        float outPitch = handPitch;

        if (fabs(outRoll) < GESTURE_DEADZONE_DEG) outRoll = 0.0f;
        if (fabs(outPitch) < GESTURE_DEADZONE_DEG) outPitch = 0.0f;

        if (outRoll > GESTURE_MAX_ANGLE_DEG) outRoll = GESTURE_MAX_ANGLE_DEG;
        if (outRoll < -GESTURE_MAX_ANGLE_DEG) outRoll = -GESTURE_MAX_ANGLE_DEG;
        if (outPitch > GESTURE_MAX_ANGLE_DEG) outPitch = GESTURE_MAX_ANGLE_DEG;
        if (outPitch < -GESTURE_MAX_ANGLE_DEG) outPitch = -GESTURE_MAX_ANGLE_DEG;

        /* Convert to binary units (0.01 deg) */
        pkt.roll = (int16_t)(outRoll * 100.0f);
        pkt.pitch = (int16_t)(outPitch * 100.0f);
        pkt.yaw = (int16_t)(handYawRate * 10.0f); /* 0.1 deg/s */

        /* Throttle generation:
         * If deadman is released: Throttle is strictly 0!
         * If deadman is engaged: Normal hover throttle base */
        if (!deadmanActive || emergencyAsserted) {
            pkt.thrust = 0;
        } else {
            /* Hand tilt up adds throttle, tilt down reduces throttle */
            float throttleAdjust = -outPitch * 800.0f; /* Pitch back = climb */
            long t = (long)THROTTLE_HOVER_EST + (long)throttleAdjust;
            if (t < THROTTLE_MIN) t = THROTTLE_MIN;
            if (t > THROTTLE_MAX) t = THROTTLE_MAX;
            pkt.thrust = (uint16_t)t;
        }

        pkt.deadman = deadmanActive ? 1 : 0;
        pkt.emergency = emergencyAsserted ? 1 : 0;
        pkt.mode = flightModeAltHold ? 1 : 0;
        pkt.crc = gloveCalculateCRC(&pkt);

        /* Transmit via UDP to Drone port 2391 */
        udp.beginPacket(droneIP, DRONE_UDP_PORT);
        udp.write((const uint8_t *)&pkt, sizeof(glovePacket_t));
        udp.endPacket();

        /* Visual feedback */
        if (emergencyAsserted) {
            digitalWrite(PIN_STATUS_LED, (now / 100) % 2); /* Fast flash */
        } else if (!deadmanActive) {
            digitalWrite(PIN_STATUS_LED, (now / 500) % 2); /* Slow flash */
        } else {
            digitalWrite(PIN_STATUS_LED, HIGH); /* Solid on */
        }
    }
}
