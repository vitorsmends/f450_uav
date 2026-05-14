#pragma once

#include <Arduino.h>

/**
 * @class RPMESCController
 * @brief Controls a brushless ESC using PWM with an RPM-based command interface.
 *
 * @details
 * This class generates a standard ESC PWM signal using the ESP32 LEDC peripheral.
 * The public interface receives RPM commands, but the current implementation uses
 * an open-loop linear mapping from RPM to PWM pulse width.
 *
 * @warning This class does not measure real motor RPM. The commanded RPM is only
 * a reference value converted to throttle. Accurate RPM control requires feedback
 * from an encoder, Hall sensor, optical sensor, or ESC telemetry.
 */
class RPMESCController {
private:
    uint8_t pin;

    uint16_t minPulseUs;
    uint16_t maxPulseUs;
    uint16_t stopPulseUs;

    uint16_t pwmFreq;
    uint8_t pwmResolution;

    float minRPM;
    float maxRPM;

    float currentCommandRPM;
    float currentThrottle;

    /**
     * @brief Converts a PWM pulse width to an ESP32 LEDC duty cycle.
     * @param pulseUs Pulse width in microseconds.
     * @return LEDC duty cycle value.
     */
    uint32_t pulseToDuty(uint16_t pulseUs) const;

    /**
     * @brief Converts throttle percentage to PWM pulse width.
     * @param throttlePercent Throttle percentage from 0.0 to 100.0.
     * @return PWM pulse width in microseconds.
     */
    uint16_t throttleToPulse(float throttlePercent) const;

    /**
     * @brief Converts an RPM command to throttle percentage.
     * @param rpm Desired RPM command.
     * @return Throttle percentage from 0.0 to 100.0.
     */
    float rpmToThrottle(float rpm) const;

    /**
     * @brief Writes a raw PWM pulse to the ESC.
     * @param pulseUs Pulse width in microseconds.
     */
    void writePulse(uint16_t pulseUs);

public:
    /**
     * @brief Constructs an RPM-based ESC controller.
     *
     * @param escPin GPIO pin connected to the ESC signal wire.
     * @param rpmMin Minimum RPM command mapped to minimum throttle.
     * @param rpmMax Maximum RPM command mapped to maximum throttle.
     * @param minUs Minimum PWM pulse width in microseconds.
     * @param maxUs Maximum PWM pulse width in microseconds.
     * @param stopUs Stop pulse width in microseconds.
     */
    RPMESCController(
        uint8_t escPin,
        float rpmMin,
        float rpmMax,
        uint16_t minUs = 1000,
        uint16_t maxUs = 2000,
        uint16_t stopUs = 1000
    );

    /**
     * @brief Initializes the ESP32 LEDC output and sends the stop signal.
     */
    void begin();

    /**
     * @brief Arms the ESC by holding the stop signal for a defined time.
     * @param armTimeMs Arming time in milliseconds.
     */
    void arm(uint32_t armTimeMs = 5000);

    /**
     * @brief Sends an RPM command to the ESC.
     *
     * @details
     * The RPM command is converted to throttle using an open-loop linear mapping.
     * This does not guarantee that the real motor RPM will match the requested RPM.
     *
     * @param rpm Desired RPM command.
     */
    void setRPM(float rpm);

    /**
     * @brief Stops the motor by sending the stop pulse.
     */
    void stop();

    /**
     * @brief Updates the RPM mapping range.
     * @param rpmMin New minimum RPM command.
     * @param rpmMax New maximum RPM command.
     */
    void setRPMRange(float rpmMin, float rpmMax);

    /**
     * @brief Updates the ESC PWM pulse range.
     * @param minUs Minimum pulse width in microseconds.
     * @param maxUs Maximum pulse width in microseconds.
     * @param stopUs Stop pulse width in microseconds.
     */
    void setPulseRange(
        uint16_t minUs,
        uint16_t maxUs,
        uint16_t stopUs = 1000
    );

    /**
     * @brief Gets the last commanded RPM.
     * @return Last commanded RPM.
     */
    float getCommandRPM() const;

    /**
     * @brief Gets the last throttle command.
     * @return Last throttle command in percentage.
     */
    float getThrottlePercent() const;
};