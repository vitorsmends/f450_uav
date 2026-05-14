#include <Arduino.h>

#include "RPMESCController.hpp"
#include "ESCManager.hpp"
#include "SerialMotorProtocol.hpp"
#include "GY91IMU.hpp"

#define ESC_1_PIN 18
#define ESC_2_PIN 19
#define ESC_3_PIN 25
#define ESC_4_PIN 26

#define IMU_SDA_PIN 21
#define IMU_SCL_PIN 22
#define MPU9250_ADDRESS 0x68

#define MIN_RPM 0.0f
#define MAX_RPM 6000.0f

#define SERIAL_BAUDRATE 115200
#define COMMAND_TIMEOUT_MS 500
#define IMU_SEND_PERIOD_MS 20

RPMESCController esc1(ESC_1_PIN, MIN_RPM, MAX_RPM);
RPMESCController esc2(ESC_2_PIN, MIN_RPM, MAX_RPM);
RPMESCController esc3(ESC_3_PIN, MIN_RPM, MAX_RPM);
RPMESCController esc4(ESC_4_PIN, MIN_RPM, MAX_RPM);

ESCManager escManager(esc1, esc2, esc3, esc4);

SerialMotorProtocol protocol(
    MIN_RPM,
    MAX_RPM,
    COMMAND_TIMEOUT_MS
);

GY91IMU imu(
    MPU9250_ADDRESS,
    IMU_SDA_PIN,
    IMU_SCL_PIN
);

bool timeoutStopApplied = false;
bool imuReady = false;

unsigned long lastIMUSendTime = 0;

void applyMotorCommand(const MotorCommandPacket& packet) {
    escManager.setRPMCommands(
        packet.rpm[0],
        packet.rpm[1],
        packet.rpm[2],
        packet.rpm[3]
    );

    escManager.update();

    timeoutStopApplied = false;
}

void handlePacket(const MotorCommandPacket& packet) {
    switch (packet.type) {
        case MotorCommandType::MOTOR:
            applyMotorCommand(packet);
            break;

        case MotorCommandType::STOP:
            escManager.stopAll();
            timeoutStopApplied = false;
            break;

        default:
            break;
    }
}

void handleCommunicationTimeout() {
    if (
        protocol.hasCommunicationTimeout() &&
        !timeoutStopApplied
    ) {
        escManager.stopAll();
        timeoutStopApplied = true;
    }
}

void sendIMUData() {
    if (!imuReady) {
        return;
    }

    unsigned long now = millis();

    if ((now - lastIMUSendTime) < IMU_SEND_PERIOD_MS) {
        return;
    }

    lastIMUSendTime = now;

    IMUData data;

    if (!imu.read(data)) {
        Serial.println("<IMU_ERROR>");
        return;
    }

    Serial.print("<IMU,");
    Serial.print(now);
    Serial.print(",");
    Serial.print(data.ax, 4);
    Serial.print(",");
    Serial.print(data.ay, 4);
    Serial.print(",");
    Serial.print(data.az, 4);
    Serial.print(",");
    Serial.print(data.gx, 4);
    Serial.print(",");
    Serial.print(data.gy, 4);
    Serial.print(",");
    Serial.print(data.gz, 4);
    Serial.println(">");
}

void setup() {
    Serial.begin(SERIAL_BAUDRATE);

    escManager.begin();
    escManager.armAll(5000);
    escManager.stopAll();

    imuReady = imu.begin();
}

void loop() {
    protocol.update(Serial);

    if (protocol.hasNewPacket()) {
        MotorCommandPacket packet = protocol.getLastPacket();
        handlePacket(packet);
    }

    handleCommunicationTimeout();

    sendIMUData();
}