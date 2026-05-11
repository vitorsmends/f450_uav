#include "RPMESCController.hpp"

RPMESCController::RPMESCController(
    uint8_t escPin,
    float rpmMin,
    float rpmMax,
    uint16_t minUs,
    uint16_t maxUs,
    uint16_t stopUs
)
    : pin(escPin),
      minPulseUs(minUs),
      maxPulseUs(maxUs),
      stopPulseUs(stopUs),
      pwmFreq(50),
      pwmResolution(16),
      minRPM(rpmMin),
      maxRPM(rpmMax),
      currentCommandRPM(0.0f),
      currentThrottle(0.0f)
{
}

uint32_t RPMESCController::pulseToDuty(uint16_t pulseUs) const {
    const uint32_t maxDuty = (1UL << pwmResolution) - 1;
    const float periodUs = 1000000.0f / pwmFreq;

    return static_cast<uint32_t>((pulseUs / periodUs) * maxDuty);
}

uint16_t RPMESCController::throttleToPulse(float throttlePercent) const {
    throttlePercent = constrain(throttlePercent, 0.0f, 100.0f);

    return minPulseUs +
           static_cast<uint16_t>((throttlePercent / 100.0f) *
           (maxPulseUs - minPulseUs));
}

float RPMESCController::rpmToThrottle(float rpm) const {
    rpm = constrain(rpm, minRPM, maxRPM);

    if (maxRPM <= minRPM) {
        return 0.0f;
    }

    return ((rpm - minRPM) / (maxRPM - minRPM)) * 100.0f;
}

void RPMESCController::writePulse(uint16_t pulseUs) {
    pulseUs = constrain(pulseUs, minPulseUs, maxPulseUs);
    ledcWrite(pin, pulseToDuty(pulseUs));
}

void RPMESCController::begin() {
    ledcAttach(pin, pwmFreq, pwmResolution);
    stop();
}

void RPMESCController::arm(uint32_t armTimeMs) {
    stop();
    delay(armTimeMs);
}

void RPMESCController::setRPM(float rpm) {
    if (rpm <= 0.0f) {
        stop();
        return;
    }

    currentCommandRPM = constrain(rpm, minRPM, maxRPM);
    currentThrottle = rpmToThrottle(currentCommandRPM);

    const uint16_t pulseUs = throttleToPulse(currentThrottle);
    writePulse(pulseUs);
}

void RPMESCController::stop() {
    currentCommandRPM = 0.0f;
    currentThrottle = 0.0f;
    writePulse(stopPulseUs);
}

void RPMESCController::setRPMRange(float rpmMin, float rpmMax) {
    minRPM = rpmMin;
    maxRPM = rpmMax;
}

void RPMESCController::setPulseRange(
    uint16_t minUs,
    uint16_t maxUs,
    uint16_t stopUs
) {
    minPulseUs = minUs;
    maxPulseUs = maxUs;
    stopPulseUs = stopUs;
}

float RPMESCController::getCommandRPM() const {
    return currentCommandRPM;
}

float RPMESCController::getThrottlePercent() const {
    return currentThrottle;
}