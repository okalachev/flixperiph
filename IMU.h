#pragma once

#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "logger.h"

#ifdef ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

// Base class for IMU drivers
class IMU : public Logger {
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

	virtual bool begin() { return false; }
	virtual bool begin(SPIClass& spi, int cs = -1, int drdy = -1) { return false; }
	virtual bool begin(TwoWire& i2c, int drdy = -1) { return false; }
	virtual void reset() {}
	virtual int status() const { return -1; } // 0 - success, otherwise error
	virtual uint8_t whoAmI() { return 0; }
	virtual bool read() { return false; }
	virtual void getAccel(float& x, float& y, float& z) const { x = y = z = NAN; }
	virtual void getGyro(float& x, float& y, float& z) const { x = y = z = NAN; }
	virtual void getMag(float& x, float& y, float& z) const { x = y = z = NAN; }
	virtual float getTemp() { return NAN; }
	virtual bool setRate(const Rate rate) { return false; }
	virtual float getRate() { return 0; }
	virtual bool setAccelRange(const AccelRange range) { return false; }
	virtual bool setGyroRange(const GyroRange range) { return false; }
	virtual bool setDLPF(const DLPF dlpf) { return false; }
	virtual char const* getModel() const { return "None"; }
	virtual bool setupInterrupt() { return false; }

	// Fabric methods
	static IMU *create(int model, SPIClass& spi, int cs = -1, int drdy = -1);
	static IMU *create(int model, TwoWire& i2c, int drdy = -1);

private:
	bool usingInterrupt = false;
	int interruptPin = -1;

#ifdef ESP32
	SemaphoreHandle_t interruptSemaphore;
	hw_timer_t *timer = NULL;

	static void ARDUINO_ISR_ATTR interruptHandler(void *interruptSemaphore) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR((SemaphoreHandle_t)interruptSemaphore, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

	bool setupInterruptTimer() {
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

		timerAttachInterruptArg(timer, interruptHandler, interruptSemaphore);
		timerAlarm(timer, alarmValue, true, 0);
		usingInterrupt = true;
		return true;
	}

	bool setupInterruptPin(uint8_t pin) {
		interruptSemaphore = xSemaphoreCreateBinary();
		pinMode(pin, INPUT_PULLUP);
		attachInterruptArg(digitalPinToInterrupt(pin), interruptHandler, interruptSemaphore, FALLING);
		usingInterrupt = true;
		interruptPin = pin;
		return true;
	}
#else
	bool setupInterruptTimer() { return false; }
	bool setupInterruptPin(uint8_t pin) { return false; }
#endif

protected:
	bool setupInterrupt(int pin) {
		if (usingInterrupt) return true; // already set

		if (pin == -1) {
			return setupInterruptTimer();
		} else {
			return setupInterruptPin(pin);
		}
	}

public:
	virtual void waitForData() {
		if (this->status() && interruptPin != -1) return; // don't hang if error and interrupt pin is used

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
