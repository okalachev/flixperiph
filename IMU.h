#pragma once

#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "logger.h"

#ifdef ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

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
	virtual int status() const = 0; // 0 - success, otherwise error
	virtual uint8_t whoAmI() = 0;
	virtual bool read() = 0;
	virtual void waitForData() = 0;
	virtual void getAccel(float& x, float& y, float& z) const = 0;
	virtual void getGyro(float& x, float& y, float& z) const = 0;
	virtual void getMag(float& x, float& y, float& z) const = 0;
	virtual float getTemp() const = 0;
	virtual bool setRate(const Rate rate) = 0;
	virtual float getRate() = 0;
	virtual bool setAccelRange(const AccelRange range) = 0;
	virtual bool setGyroRange(const GyroRange range) = 0;
	virtual bool setDLPF(const DLPF dlpf) = 0;
	virtual const char* getModel() const = 0;
	virtual bool setupInterrupt() = 0;
};

#ifdef ESP32
static void ARDUINO_ISR_ATTR _interruptHandler(void *interruptSemaphore) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR((SemaphoreHandle_t)interruptSemaphore, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#endif

// Base for all IMU drivers
class IMUBase : public IMUInterface, public Logger {
private:
	bool usingInterrupt = false;
	SemaphoreHandle_t interruptSemaphore;
	hw_timer_t *timer = NULL;

	bool setupInterruptTimer() {
#ifdef ESP32
		interruptSemaphore = xSemaphoreCreateBinary();

		float rate = getRate();
		uint32_t frequency = 1000000;
		uint64_t alarmValue = round(1000000 / rate);

		if (timer != NULL) {
			timerEnd(timer);
		}

		timer = timerBegin(frequency);
		if (timer == NULL) {
			log("Failed to create timer");
			return false;
		}

		timerAttachInterruptArg(timer, _interruptHandler, interruptSemaphore);
		timerAlarm(timer, alarmValue, true, 0);
		usingInterrupt = true;
		return true;
#else
		return false;
#endif
	}

	bool setupInterruptPin(uint8_t pin) {
#ifdef ESP32
		interruptSemaphore = xSemaphoreCreateBinary();
		attachInterruptArg(pin, _interruptHandler, interruptSemaphore, FALLING);
		usingInterrupt = true;
		return true;
#else
		return false;
#endif
	}

protected:
	bool setupInterrupt(int pin = -1) {
		if (usingInterrupt) return true; // already set

		if (pin == -1) {
			return setupInterruptTimer();
		} else {
			return setupInterruptPin(pin);
		}
	}

public:
	void waitForData() {
		if (this->status()) return; // don't wait if there's an error

		if (usingInterrupt) {
#ifdef ESP32
			xSemaphoreTake(interruptSemaphore, portMAX_DELAY); // wait using interrupt
			this->read();
#endif
		} else {
			while (!this->read()); // wait using polling
		}
	}
};
