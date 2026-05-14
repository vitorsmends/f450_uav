#include "GY91IMU.hpp"

#define MPU9250_PWR_MGMT_1   0x6B
#define MPU9250_ACCEL_XOUT_H 0x3B

GY91IMU::GY91IMU(uint8_t i2cAddress, uint8_t sda, uint8_t scl)
    : address(i2cAddress),
      sdaPin(sda),
      sclPin(scl)
{
}

bool GY91IMU::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool GY91IMU::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length) {
    Wire.beginTransmission(address);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    uint8_t received = Wire.requestFrom(address, length);

    if (received != length) {
        return false;
    }

    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = Wire.read();
    }

    return true;
}

bool GY91IMU::begin() {
    Wire.begin(sdaPin, sclPin);
    Wire.setClock(400000);

    delay(100);

    return writeRegister(MPU9250_PWR_MGMT_1, 0x00);
}

bool GY91IMU::read(IMUData& data) {
    uint8_t buffer[14];

    if (!readRegisters(MPU9250_ACCEL_XOUT_H, buffer, 14)) {
        return false;
    }

    int16_t rawAx = (buffer[0] << 8) | buffer[1];
    int16_t rawAy = (buffer[2] << 8) | buffer[3];
    int16_t rawAz = (buffer[4] << 8) | buffer[5];

    int16_t rawGx = (buffer[8] << 8) | buffer[9];
    int16_t rawGy = (buffer[10] << 8) | buffer[11];
    int16_t rawGz = (buffer[12] << 8) | buffer[13];

    data.ax = rawAx / 16384.0f;
    data.ay = rawAy / 16384.0f;
    data.az = rawAz / 16384.0f;

    data.gx = rawGx / 131.0f;
    data.gy = rawGy / 131.0f;
    data.gz = rawGz / 131.0f;

    return true;
}