#include <Arduino.h>
#include "RPMESCController.hpp"

#define ESC_1_PIN 18

RPMESCController esc1(
    ESC_1_PIN,
    0.0f,      // Minimum RPM command
    12000.0f   // Maximum RPM command estimate
);

void setup() {
    Serial.begin(115200);

    esc1.begin();

    Serial.println("Arming ESC...");
    esc1.arm(5000);
    Serial.println("ESC armed.");
}

void loop() {
    Serial.println("Command: 2000 RPM");
    esc1.setRPM(2000.0f);
    delay(4000);

    Serial.println("Command: 4000 RPM");
    esc1.setRPM(4000.0f);
    delay(4000);

    Serial.println("Command: 6000 RPM");
    esc1.setRPM(6000.0f);
    delay(4000);

    Serial.println("Stop");
    esc1.stop();
    delay(5000);
}