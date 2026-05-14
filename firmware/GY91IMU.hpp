#pragma once

#include <Arduino.h>
#include <Wire.h>

/**
 * @struct IMUData
 * @brief Stores accelerometer and gyroscope measurements from the GY-91 module.
 *
 * @details
 * Acceleration values are expressed in g.
 * Gyroscope values are expressed in degrees per second.
 */
struct IMUData {
    float ax; /**< Acceleration on X axis in g. */
    float ay; /**< Acceleration on Y axis in g. */
    float az; /**< Acceleration on Z axis in g. */

    float gx; /**< Angular velocity on X axis in degrees per second. */
    float gy; /**< Angular velocity on Y axis in degrees per second. */
    float gz; /**< Angular velocity on Z axis in degrees per second. */
};

/**
 * @class GY91IMU
 * @brief Driver wrapper for reading IMU data from a GY-91 module.
 *
 * @details
 * This class communicates with the MPU9250 IMU present in the GY-91 module
 * using the I2C bus. It initializes the sensor and reads raw accelerometer
 * and gyroscope measurements, converting them to physical units.
 *
 * The current implementation reads:
 * - Accelerometer data in g
 * - Gyroscope data in degrees per second
 *
 * Magnetometer and barometer readings are not included in this class yet.
 */
class GY91IMU {
private:
    uint8_t address; /**< I2C address of the MPU9250 device. */
    uint8_t sdaPin;  /**< ESP32 GPIO used as I2C SDA. */
    uint8_t sclPin;  /**< ESP32 GPIO used as I2C SCL. */

    /**
     * @brief Writes a single byte to an MPU9250 register.
     *
     * @param reg Register address.
     * @param value Value to write.
     * @return true if the write operation succeeds.
     * @return false if the I2C transaction fails.
     */
    bool writeRegister(uint8_t reg, uint8_t value);

    /**
     * @brief Reads multiple consecutive bytes from MPU9250 registers.
     *
     * @param reg Initial register address.
     * @param buffer Output buffer where read bytes will be stored.
     * @param length Number of bytes to read.
     * @return true if all bytes are successfully read.
     * @return false if the I2C transaction fails.
     */
    bool readRegisters(
        uint8_t reg,
        uint8_t* buffer,
        uint8_t length
    );

public:
    /**
     * @brief Constructs a GY91IMU object.
     *
     * @param i2cAddress I2C address of the MPU9250, usually 0x68 or 0x69.
     * @param sda ESP32 GPIO used for I2C SDA.
     * @param scl ESP32 GPIO used for I2C SCL.
     */
    GY91IMU(
        uint8_t i2cAddress,
        uint8_t sda,
        uint8_t scl
    );

    /**
     * @brief Initializes the I2C bus and wakes up the MPU9250.
     *
     * @details
     * This method configures the ESP32 I2C pins and sets the MPU9250 power
     * management register to exit sleep mode.
     *
     * @return true if the sensor is successfully initialized.
     * @return false if initialization fails.
     */
    bool begin();

    /**
     * @brief Reads accelerometer and gyroscope measurements.
     *
     * @param data Reference to an IMUData structure where measurements are stored.
     * @return true if the sensor data is successfully read.
     * @return false if the I2C read operation fails.
     */
    bool read(IMUData& data);
};