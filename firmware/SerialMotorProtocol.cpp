#include "SerialMotorProtocol.hpp"

SerialMotorProtocol::SerialMotorProtocol(
    float rpmMin,
    float rpmMax,
    unsigned long communicationTimeoutMs
)
    : buffer(""),
      receiving(false),
      newPacketAvailable(false),
      lastPacketTimestamp(0),
      minRPM(rpmMin),
      maxRPM(rpmMax),
      timeoutMs(communicationTimeoutMs)
{
    lastPacket.type = MotorCommandType::NONE;

    for (int i = 0; i < MOTOR_PROTOCOL_NUM_MOTORS; i++) {
        lastPacket.rpm[i] = 0.0f;
    }
}

bool SerialMotorProtocol::parseMotorPayload(
    const String& payload,
    MotorCommandPacket& packet
) const {
    String fields[5];

    int startIndex = 0;

    for (int i = 0; i < 5; i++) {
        int commaIndex = payload.indexOf(',', startIndex);

        if (i < 4) {
            if (commaIndex < 0) {
                return false;
            }

            fields[i] = payload.substring(startIndex, commaIndex);
            startIndex = commaIndex + 1;
        } else {
            fields[i] = payload.substring(startIndex);

            if (fields[i].indexOf(',') >= 0) {
                return false;
            }
        }

        fields[i].trim();

        if (fields[i].length() == 0) {
            return false;
        }
    }

    if (!fields[0].equals("MOTOR")) {
        return false;
    }

    packet.type = MotorCommandType::MOTOR;

    for (int i = 0; i < MOTOR_PROTOCOL_NUM_MOTORS; i++) {
        float rpmValue = fields[i + 1].toFloat();

        if (rpmValue < minRPM || rpmValue > maxRPM) {
            return false;
        }

        packet.rpm[i] = rpmValue;
    }

    return true;
}

bool SerialMotorProtocol::parseStopPayload(
    const String& payload,
    MotorCommandPacket& packet
) const {
    String command = payload;
    command.trim();

    if (!command.equals("STOP")) {
        return false;
    }

    packet.type = MotorCommandType::STOP;

    for (int i = 0; i < MOTOR_PROTOCOL_NUM_MOTORS; i++) {
        packet.rpm[i] = 0.0f;
    }

    return true;
}

bool SerialMotorProtocol::decodePacket(
    const String& rawPacket,
    MotorCommandPacket& packet
) const {
    packet.type = MotorCommandType::INVALID;

    for (int i = 0; i < MOTOR_PROTOCOL_NUM_MOTORS; i++) {
        packet.rpm[i] = 0.0f;
    }

    if (
        !rawPacket.startsWith("<") ||
        !rawPacket.endsWith(">")
    ) {
        return false;
    }

    String payload = rawPacket.substring(1, rawPacket.length() - 1);
    payload.trim();

    if (payload.startsWith("MOTOR,")) {
        return parseMotorPayload(payload, packet);
    }

    if (payload.equals("STOP")) {
        return parseStopPayload(payload, packet);
    }

    return false;
}

void SerialMotorProtocol::update(Stream& stream) {
    while (stream.available()) {
        char c = static_cast<char>(stream.read());

        if (c == '<') {
            receiving = true;
            buffer = "<";
            continue;
        }

        if (c == '>' && receiving) {
            buffer += ">";

            MotorCommandPacket decodedPacket;
            bool valid = decodePacket(buffer, decodedPacket);

            if (valid) {
                lastPacket = decodedPacket;
                lastPacketTimestamp = millis();
                newPacketAvailable = true;
            }

            buffer = "";
            receiving = false;

            continue;
        }

        if (receiving) {
            buffer += c;

            if (buffer.length() > MOTOR_PROTOCOL_MAX_PACKET_SIZE) {
                buffer = "";
                receiving = false;
            }
        }
    }
}

bool SerialMotorProtocol::hasNewPacket() const {
    return newPacketAvailable;
}

MotorCommandPacket SerialMotorProtocol::getLastPacket() {
    newPacketAvailable = false;
    return lastPacket;
}

bool SerialMotorProtocol::hasCommunicationTimeout() const {
    if (lastPacketTimestamp == 0) {
        return false;
    }

    return (millis() - lastPacketTimestamp) > timeoutMs;
}

String SerialMotorProtocol::buildMotorPacket(
    float rpm1,
    float rpm2,
    float rpm3,
    float rpm4
) const {
    String packet = "<MOTOR,";

    packet += String(rpm1, 2);
    packet += ",";
    packet += String(rpm2, 2);
    packet += ",";
    packet += String(rpm3, 2);
    packet += ",";
    packet += String(rpm4, 2);
    packet += ">";

    return packet;
}

String SerialMotorProtocol::buildStopPacket() const {
    return "<STOP>";
}