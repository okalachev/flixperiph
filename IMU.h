#pragma once

#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"

// IMU driver interface

class IMUInterface {
public:
	enum IMUType {
		UNKNOWN,
		MPU9250,
		MPU9252,
		MPU6500,
		ICM20948,
	};
	enum DlpfBandwidth : int8_t {
		DLPF_BANDWIDTH_184HZ = 0x01,
		DLPF_BANDWIDTH_92HZ = 0x02,
		DLPF_BANDWIDTH_41HZ = 0x03,
		DLPF_BANDWIDTH_20HZ = 0x04,
		DLPF_BANDWIDTH_10HZ = 0x05,
		DLPF_BANDWIDTH_5HZ = 0x06
	};
	enum AccelRange : int8_t {
		ACCEL_RANGE_2G = 0x00,
		ACCEL_RANGE_4G = 0x08,
		ACCEL_RANGE_8G = 0x10,
		ACCEL_RANGE_16G = 0x18
	};
	enum GyroRange : int8_t {
		GYRO_RANGE_250DPS = 0x00,
		GYRO_RANGE_500DPS = 0x08,
		GYRO_RANGE_1000DPS = 0x10,
		GYRO_RANGE_2000DPS = 0x18
	};
	enum Rate {
		RATE_MIN,
		RATE_50HZ_APPROX,
		RATE_1KHZ_APPROX,
		RATE_8KHZ_APPROX,
		RATE_MAX
	};

	virtual bool begin() = 0;
	virtual void reset() = 0;
	virtual uint8_t whoAmI() = 0;
	virtual bool read() = 0;
	virtual void waitForData() = 0;
	virtual void getAccel(float& x, float& y, float& z) const = 0;
	virtual void getGyro(float& x, float& y, float& z) const = 0;
	virtual void getMag(float& x, float& y, float& z) const = 0;
	virtual float getTemp() const = 0;
	virtual bool setRate(Rate rate) = 0;
	virtual const char* getModel() const = 0;
};
