#pragma once

#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"

// IMU driver interface

class IMUInterface {
public:
	enum DLPF {
		DLPF_OFF,
		DLPF_MAX,
		DLPF_100HZ_APPROX,
		DLPF_50HZ_APPROX,
		DLPF_5HZ_APPROX,
		DLPF_MIN
	};
	enum AccelRange {
		ACCEL_RANGE_MIN,
		ACCEL_RANGE_2G,
		ACCEL_RANGE_4G,
		ACCEL_RANGE_8G,
		ACCEL_RANGE_16G,
		ACCEL_RANGE_MAX
	};
	enum GyroRange {
		GYRO_RANGE_MIN,
		GYRO_RANGE_250DPS,
		GYRO_RANGE_500DPS,
		GYRO_RANGE_1000DPS,
		GYRO_RANGE_2000DPS,
		GYRO_RANGE_MAX
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
	virtual bool setRate(const Rate rate) = 0;
	virtual bool setAccelRange(const AccelRange range) = 0;
	virtual bool setGyroRange(const GyroRange range) = 0;
	virtual bool setDLPF(const DLPF dlpf) = 0;
	virtual const char* getModel() const = 0;
};
