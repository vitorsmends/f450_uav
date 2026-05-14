#pragma once

#include <Arduino.h>
#include "RPMESCController.hpp"

/**
 * @class ESCManager
 * @brief Manages and synchronizes multiple ESC controllers.
 *
 * @details
 * This class provides a centralized interface for controlling four independent
 * ESCs through their corresponding RPMESCController instances.
 *
 * The manager stores RPM commands internally and applies them simultaneously
 * during the update() cycle. This architecture improves synchronization between
 * motors and simplifies future integration with:
 *
 * - ROS nodes
 * - Serial communication interfaces
 * - Flight controllers
 * - Motor mixers
 * - Closed-loop control systems
 *
 * @note
 * The ESCManager does not directly generate PWM signals. PWM generation is
 * delegated to the RPMESCController instances.
 */
class ESCManager {
private:

    /**
     * @brief Reference to motor 1 ESC controller.
     */
    RPMESCController& motor1;

    /**
     * @brief Reference to motor 2 ESC controller.
     */
    RPMESCController& motor2;

    /**
     * @brief Reference to motor 3 ESC controller.
     */
    RPMESCController& motor3;

    /**
     * @brief Reference to motor 4 ESC controller.
     */
    RPMESCController& motor4;

    /**
     * @brief Stored RPM command for motor 1.
     */
    float rpm1;

    /**
     * @brief Stored RPM command for motor 2.
     */
    float rpm2;

    /**
     * @brief Stored RPM command for motor 3.
     */
    float rpm3;

    /**
     * @brief Stored RPM command for motor 4.
     */
    float rpm4;

public:

    /**
     * @brief Constructs an ESCManager instance.
     *
     * @param esc1 Reference to ESC controller for motor 1.
     * @param esc2 Reference to ESC controller for motor 2.
     * @param esc3 Reference to ESC controller for motor 3.
     * @param esc4 Reference to ESC controller for motor 4.
     */
    ESCManager(
        RPMESCController& esc1,
        RPMESCController& esc2,
        RPMESCController& esc3,
        RPMESCController& esc4
    );

    /**
     * @brief Initializes all ESC controllers.
     *
     * @details
     * Calls the begin() method for every managed RPMESCController instance.
     */
    void begin();

    /**
     * @brief Arms all ESCs simultaneously.
     *
     * @details
     * Sends the stop signal to all ESCs and waits for the specified arming time.
     *
     * @param armTimeMs Arming time in milliseconds.
     */
    void armAll(uint32_t armTimeMs = 5000);

    /**
     * @brief Stops all motors immediately.
     *
     * @details
     * Sends the stop command to all managed ESCs and resets stored RPM commands.
     */
    void stopAll();

    /**
     * @brief Stores RPM commands for all motors.
     *
     * @details
     * The commands are internally stored and later applied using update().
     *
     * @param m1 RPM command for motor 1.
     * @param m2 RPM command for motor 2.
     * @param m3 RPM command for motor 3.
     * @param m4 RPM command for motor 4.
     */
    void setRPMCommands(float m1, float m2, float m3, float m4);

    /**
     * @brief Applies the stored RPM commands to all ESCs.
     *
     * @details
     * This method sends the latest stored RPM commands to each managed
     * RPMESCController instance.
     */
    void update();
};