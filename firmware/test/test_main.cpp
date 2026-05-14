#include <Arduino.h>

#include "RPMESCController.hpp"
#include "ESCManager.hpp"
#include "SerialMotorProtocol.hpp"

#define ESC_1_PIN 18
#define ESC_2_PIN 19
#define ESC_3_PIN 25
#define ESC_4_PIN 26

#define MIN_RPM 0.0f
#define MAX_RPM 6000.0f

#define SERIAL_BAUDRATE 115200
#define COMMAND_TIMEOUT_MS 1000

RPMESCController esc1(ESC_1_PIN, MIN_RPM, MAX_RPM);
RPMESCController esc2(ESC_2_PIN, MIN_RPM, MAX_RPM);
RPMESCController esc3(ESC_3_PIN, MIN_RPM, MAX_RPM);
RPMESCController esc4(ESC_4_PIN, MIN_RPM, MAX_RPM);

ESCManager escManager(
    esc1,
    esc2,
    esc3,
    esc4
);

SerialMotorProtocol protocol(
    MIN_RPM,
    MAX_RPM,
    COMMAND_TIMEOUT_MS
);

bool timeoutStopApplied = false;

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

void setup() {
    Serial.begin(SERIAL_BAUDRATE);

    escManager.begin();
    escManager.armAll(5000);
    escManager.stopAll();
}

void loop() {
    protocol.update(Serial);

    if (protocol.hasNewPacket()) {
        MotorCommandPacket packet = protocol.getLastPacket();
        handlePacket(packet);
    }

    handleCommunicationTimeout();
}