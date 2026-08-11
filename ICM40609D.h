/*
 * ICM-40609-D 6-axis IMU driver for Arduino.
 * Repository: https://github.com/okalachev/flixperiph
 */

#pragma once

#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>

#include "IMU.h"
#include "logger.h"

class ICM40609D : public IMU {
public:
	static constexpr uint8_t WHO_AM_I_VALUE = 0x3B;

	enum I2cAddr : uint8_t {
		I2C_ADDR_PRIM = 0x68,
		I2C_ADDR_SEC = 0x69
	};

	/* Constructors */
	ICM40609D(TwoWire& w, int drdy = -1, I2cAddr addr = I2C_ADDR_PRIM)
		: _wire{&w}, intPin{drdy}, i2cAddress{static_cast<uint8_t>(addr)}, useSPI{false} {}
	ICM40609D(SPIClass& s, int cs = -1, int drdy = -1)
		: _spi{&s}, csPin{cs}, intPin{drdy}, useSPI{true} {}

	/* IMU interface implementation */
	bool begin() override;
	void reset() override;
	int status() const override { return _status; }
	uint8_t whoAmI() override;
	bool read() override;
	void getAccel(float& x, float& y, float& z) const override;
	void getGyro(float& x, float& y, float& z) const override;
	void getMag(float& x, float& y, float& z) const override;
	float getTemp() override;
	bool setRate(const Rate rate) override;
	float getRate() override;
	bool setAccelRange(const AccelRange range) override;
	bool setGyroRange(const GyroRange range) override;
	bool setDLPF(const DLPF dlpf) override;
	const char* getModel() const override { return "ICM-40609-D"; }
	bool setupInterrupt() override;

private:
	/* Communication */
	TwoWire* _wire = nullptr;
	SPIClass* _spi = nullptr;
	uint8_t i2cAddress = I2C_ADDR_PRIM;
	int csPin = -1;
	int intPin = -1;
	bool useSPI = true;
	SPISettings spiSettingsCfg;
	SPISettings spiSettingsData;
	uint8_t currentBank = 0;

	/* State */
	int _status = 0;
	float accelScale_ = 0.0f;
	float gyroScale_ = 0.0f;

	/* Data buffer: TEMP(2) + ACCEL(6) + GYRO(6) = 14 bytes */
	uint8_t buffer_[14] = {};
	float accel_[3] = {};
	float gyro_[3] = {};
	float temp_ = 0.0f;

	/* Constants */
	static constexpr float G_MPS2 = 9.80665f;
	static constexpr float DEG2RAD = 3.14159265358979323846f / 180.0f;
	static constexpr float TEMP_SCALE = 132.48f;
	static constexpr float TEMP_OFFSET = 25.0f;
	static constexpr uint32_t SPI_CFG_CLOCK = 1000000;    // 1 MHz for config
	static constexpr uint32_t SPI_DATA_CLOCK = 16000000;  // 16 MHz for data read

	/* Bank 0 register addresses */
	static constexpr uint8_t REG_DEVICE_CONFIG       = 0x11;
	static constexpr uint8_t REG_DRIVE_CONFIG        = 0x13;
	static constexpr uint8_t REG_INT_CONFIG          = 0x14;
	static constexpr uint8_t REG_TEMP_DATA1          = 0x1D;
	static constexpr uint8_t REG_ACCEL_DATA_X1       = 0x1F;
	static constexpr uint8_t REG_INT_STATUS          = 0x2D;
	static constexpr uint8_t REG_SIGNAL_PATH_RESET   = 0x4B;
	static constexpr uint8_t REG_INTF_CONFIG1        = 0x4D;
	static constexpr uint8_t REG_PWR_MGMT0           = 0x4E;
	static constexpr uint8_t REG_GYRO_CONFIG0        = 0x4F;
	static constexpr uint8_t REG_ACCEL_CONFIG0       = 0x50;
	static constexpr uint8_t REG_GYRO_CONFIG1        = 0x51;
	static constexpr uint8_t REG_GYRO_ACCEL_CONFIG0  = 0x52;
	static constexpr uint8_t REG_ACCEL_CONFIG1       = 0x53;
	static constexpr uint8_t REG_INT_CONFIG0         = 0x63;
	static constexpr uint8_t REG_INT_CONFIG1         = 0x64;
	static constexpr uint8_t REG_INT_SOURCE0         = 0x65;
	static constexpr uint8_t REG_WHO_AM_I            = 0x75;
	static constexpr uint8_t REG_BANK_SEL            = 0x76;

	/* Register bit masks / values */
	static constexpr uint8_t DATA_RDY_INT            = 0x08;
	static constexpr uint8_t UI_DRDY_INT1_EN         = 0x08;

	/* ODR codes for ACCEL_CONFIG0 / GYRO_CONFIG0 [3:0] */
	static constexpr uint8_t ODR_32KHZ  = 0x01;
	static constexpr uint8_t ODR_16KHZ  = 0x02;
	static constexpr uint8_t ODR_8KHZ   = 0x03;
	static constexpr uint8_t ODR_4KHZ   = 0x04;
	static constexpr uint8_t ODR_2KHZ   = 0x05;
	static constexpr uint8_t ODR_1KHZ   = 0x06;
	static constexpr uint8_t ODR_200HZ  = 0x07;
	static constexpr uint8_t ODR_100HZ  = 0x08;
	static constexpr uint8_t ODR_50HZ   = 0x09;
	static constexpr uint8_t ODR_25HZ   = 0x0A;
	static constexpr uint8_t ODR_12_5HZ = 0x0B;
	static constexpr uint8_t ODR_500HZ  = 0x0F;

	/* Low-level register I/O */
	void switchBank(uint8_t bank);
	void writeRegister(uint8_t reg, uint8_t val);
	uint8_t readRegister(uint8_t reg);
	void readRegisters(uint8_t reg, uint8_t count, uint8_t* dest);

	/* Helpers */
	bool setODR(uint8_t odrCode);
	float odrCodeToHz(uint8_t code) const;
	bool enableDataReadyInterrupt();
};
