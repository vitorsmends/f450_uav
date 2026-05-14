#include "ESCManager.hpp"

ESCManager::ESCManager(
    RPMESCController& esc1,
    RPMESCController& esc2,
    RPMESCController& esc3,
    RPMESCController& esc4
)
    : motor1(esc1),
      motor2(esc2),
      motor3(esc3),
      motor4(esc4),
      rpm1(0.0f),
      rpm2(0.0f),
      rpm3(0.0f),
      rpm4(0.0f)
{
}

void ESCManager::begin() {
    motor1.begin();
    motor2.begin();
    motor3.begin();
    motor4.begin();
}

void ESCManager::armAll(uint32_t armTimeMs) {
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();

    delay(armTimeMs);
}

void ESCManager::stopAll() {
    rpm1 = 0.0f;
    rpm2 = 0.0f;
    rpm3 = 0.0f;
    rpm4 = 0.0f;

    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
}

void ESCManager::setRPMCommands(float m1, float m2, float m3, float m4) {
    rpm1 = m1;
    rpm2 = m2;
    rpm3 = m3;
    rpm4 = m4;
}

void ESCManager::update() {
    motor1.setRPM(rpm1);
    motor2.setRPM(rpm2);
    motor3.setRPM(rpm3);
    motor4.setRPM(rpm4);
}