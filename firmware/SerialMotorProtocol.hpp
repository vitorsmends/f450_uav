#pragma once

#include <Arduino.h>

#define MOTOR_PROTOCOL_NUM_MOTORS 4
#define MOTOR_PROTOCOL_MAX_PACKET_SIZE 96

enum class MotorCommandType {
    NONE,
    MOTOR,
    STOP,
    INVALID
};

struct MotorCommandPacket {
    MotorCommandType type;
    float rpm[MOTOR_PROTOCOL_NUM_MOTORS];
};

class SerialMotorProtocol {
private:
    String buffer;
    bool receiving;
    bool newPacketAvailable;

    MotorCommandPacket lastPacket;

    unsigned long lastPacketTimestamp;

    float minRPM;
    float maxRPM;
    unsigned long timeoutMs;

    bool parseMotorPayload(
        const String& payload,
        MotorCommandPacket& packet
    ) const;

    bool parseStopPayload(
        const String& payload,
        MotorCommandPacket& packet
    ) const;

    bool decodePacket(
        const String& rawPacket,
        MotorCommandPacket& packet
    ) const;

public:
    SerialMotorProtocol(
        float rpmMin,
        float rpmMax,
        unsigned long communicationTimeoutMs = 1000
    );

    void update(Stream& stream);

    bool hasNewPacket() const;

    MotorCommandPacket getLastPacket();

    bool hasCommunicationTimeout() const;

    String buildMotorPacket(
        float rpm1,
        float rpm2,
        float rpm3,
        float rpm4
    ) const;

    String buildStopPacket() const;
};