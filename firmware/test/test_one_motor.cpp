#include <Arduino.h>

#include "RPMESCController.hpp"

/* ============================================================
 * ESC GPIO Configuration
 * ============================================================ */

#define ESC_1_PIN 18

/* ============================================================
 * RPM Limits
 * ============================================================ */

#define MIN_RPM 0.0f
#define MAX_RPM 6000.0f

/* ============================================================
 * Serial Configuration
 * ============================================================ */

#define SERIAL_BAUDRATE 115200
#define SERIAL_TIMEOUT_MS 1000

/* ============================================================
 * Start Configuration
 * ============================================================ */

#define START_KICK_RPM 1800.0f
#define START_KICK_TIME_MS 300

RPMESCController esc1(
    ESC_1_PIN,
    MIN_RPM,
    MAX_RPM
);

float currentRPM = 0.0f;

void stopMotor() {
    currentRPM = 0.0f;
    esc1.stop();

    Serial.println("Motor stopped.");
}

void applyRPM(float rpm) {
    rpm = constrain(rpm, MIN_RPM, MAX_RPM);

    if (rpm <= 0.0f) {
        stopMotor();
        return;
    }

    /*
     * If the motor is stopped, apply a short startup kick first.
     * This helps sensorless ESCs start the motor without stalling.
     */
    if (currentRPM <= 0.0f) {
        Serial.println("Applying startup kick...");

        esc1.setRPM(START_KICK_RPM);
        delay(START_KICK_TIME_MS);
    }

    currentRPM = rpm;
    esc1.setRPM(currentRPM);

    Serial.print("Applied RPM command: ");
    Serial.println(currentRPM);

    Serial.print("Throttle: ");
    Serial.print(esc1.getThrottlePercent());
    Serial.println("%");
}

void printMenu() {
    Serial.println();
    Serial.println("==================================");
    Serial.println("Single ESC Test");
    Serial.println("==================================");
    Serial.println("Send a direct RPM command:");
    Serial.println("Example: 800");
    Serial.println("Example: 1200");
    Serial.println("Example: 1500");
    Serial.println();
    Serial.println("Send STOP or s to stop motor.");
    Serial.println("Motor starts stopped.");
    Serial.println("==================================");
    Serial.println();
}

void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    Serial.setTimeout(SERIAL_TIMEOUT_MS);

    Serial.println();
    Serial.println("Initializing ESC...");

    esc1.begin();

    Serial.println("Arming ESC...");
    esc1.arm(5000);

    stopMotor();

    Serial.println("ESC armed.");
    printMenu();
}

void loop() {
    if (!Serial.available()) {
        return;
    }

    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) {
        return;
    }

    if (
        input.equalsIgnoreCase("s") ||
        input.equalsIgnoreCase("STOP")
    ) {
        stopMotor();
        return;
    }

    float rpm = input.toFloat();

    if (rpm <= 0.0f) {
        stopMotor();
        return;
    }

    applyRPM(rpm);
}