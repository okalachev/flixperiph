/*
 * ICM-40609-D 6-axis IMU driver for Arduino.
 * Repository: https://github.com/okalachev/flixperiph
 */

#include "ICM40609D.h"

bool ICM40609D::begin() {
	setLogName("IMU");

	if (useSPI) {
		_spi->begin();
		if (csPin == -1) {
			#ifdef ESP32
				csPin = _spi->pinSS();
			#else
				csPin = SS;
			#endif
		}
		pinMode(csPin, OUTPUT);
		digitalWrite(csPin, HIGH);
		spiSettingsCfg = SPISettings(SPI_CFG_CLOCK, MSBFIRST, SPI_MODE0);
		spiSettingsData = SPISettings(SPI_DATA_CLOCK, MSBFIRST, SPI_MODE0);
	}
	currentBank = 0;

	// Soft reset
	reset();

	// Check WHO_AM_I with retries
	int attempts = 0;
	while (true) {
		uint8_t id = whoAmI();
		if (id == WHO_AM_I_VALUE) {
			log("ICM-40609-D detected");
			break;
		}
		if (attempts++ > 10) {
			_status = 1;
			log("Wrong WHO_AM_I: 0x%02X (expected 0x%02X)", id, WHO_AM_I_VALUE);
			return false;
		}
		delay(10);
	}

	// Turn on accel and gyro in Low Noise mode
	writeRegister(REG_PWR_MGMT0, 0x0F);
	delay(1); // wait for gyro startup (>200 μs)

	// Set default accel range: 16G
	if (!setAccelRange(ACCEL_RANGE_16G)) {
		_status = 2;
		log("Failed to set accel range");
		return false;
	}

	// Set default gyro range: 2000 DPS
	if (!setGyroRange(GYRO_RANGE_2000DPS)) {
		_status = 3;
		log("Failed to set gyro range");
		return false;
	}

	// Set default ODR: 1 kHz
	if (!setODR(ODR_1KHZ)) {
		_status = 4;
		log("Failed to set ODR");
		return false;
	}

	_status = 0;
	return true;
}

void ICM40609D::reset() {
	writeRegister(REG_DEVICE_CONFIG, 0x01); // software reset
	delay(2); // wait for reset to complete (>1ms)
	currentBank = 0;
}

uint8_t ICM40609D::whoAmI() {
	return readRegister(REG_WHO_AM_I);
}

bool ICM40609D::read() {
	if (this->status()) return false; // FIXME:
	// Check data ready // FIXME:
	// uint8_t intStatus = readRegister(REG_INT_STATUS);
	// if (!(intStatus & DATA_RDY_INT)) {
	// 	return false;
	// }

	// Burst read: TEMP(2) + ACCEL(6) + GYRO(6) = 14 bytes starting from TEMP_DATA1
	readRegisters(REG_TEMP_DATA1, 14, buffer_);

	// Parse temperature
	int16_t rawTemp = (static_cast<int16_t>(buffer_[0]) << 8) | buffer_[1];
	temp_ = (static_cast<float>(rawTemp) / TEMP_SCALE) + TEMP_OFFSET;

	// Parse accelerometer (bytes 2-7)
	int16_t rawAccel[3];
	rawAccel[0] = (static_cast<int16_t>(buffer_[2]) << 8) | buffer_[3];
	rawAccel[1] = (static_cast<int16_t>(buffer_[4]) << 8) | buffer_[5];
	rawAccel[2] = (static_cast<int16_t>(buffer_[6]) << 8) | buffer_[7];

	accel_[0] = static_cast<float>(rawAccel[0]) * accelScale_ * G_MPS2;
	accel_[1] = static_cast<float>(rawAccel[1]) * accelScale_ * G_MPS2;
	accel_[2] = static_cast<float>(rawAccel[2]) * accelScale_ * G_MPS2;

	// Parse gyroscope (bytes 8-13)
	int16_t rawGyro[3];
	rawGyro[0] = (static_cast<int16_t>(buffer_[8])  << 8) | buffer_[9];
	rawGyro[1] = (static_cast<int16_t>(buffer_[10]) << 8) | buffer_[11];
	rawGyro[2] = (static_cast<int16_t>(buffer_[12]) << 8) | buffer_[13];

	gyro_[0] = static_cast<float>(rawGyro[0]) * gyroScale_ * DEG2RAD;
	gyro_[1] = static_cast<float>(rawGyro[1]) * gyroScale_ * DEG2RAD;
	gyro_[2] = static_cast<float>(rawGyro[2]) * gyroScale_ * DEG2RAD;

	return true;
}

void ICM40609D::getAccel(float& x, float& y, float& z) const {
	x = accel_[0];
	y = accel_[1];
	z = accel_[2];
}

void ICM40609D::getGyro(float& x, float& y, float& z) const {
	x = gyro_[0];
	y = gyro_[1];
	z = gyro_[2];
}

void ICM40609D::getMag(float& x, float& y, float& z) const {
	// No magnetometer on ICM-40609-D
	x = 0.0f;
	y = 0.0f;
	z = 0.0f;
}

float ICM40609D::getTemp() {
	return temp_;
}

bool ICM40609D::setAccelRange(const AccelRange range) {
	uint8_t fsSel;
	float scale;
	switch (range) {
		case ACCEL_RANGE_MIN:
			fsSel = 0x03;
			scale = 4.0f / 32768.0f;
			break;
		case ACCEL_RANGE_2G:
			// ICM-40609-D does not support 2g: ACCEL_FS_SEL=100 is reserved.
			// Use the closest supported range (4g).
			fsSel = 0x03;
			scale = 4.0f / 32768.0f;
			log("ACCEL_RANGE_2G is not supported on ICM-40609-D, using 4g");
			break;
		case ACCEL_RANGE_4G:
			fsSel = 0x03;
			scale = 4.0f / 32768.0f;
			break;
		case ACCEL_RANGE_8G:
			fsSel = 0x02;
			scale = 8.0f / 32768.0f;
			break;
		case ACCEL_RANGE_MAX:
			fsSel = 0x00;
			scale = 32.0f / 32768.0f;
			break;
		case ACCEL_RANGE_16G:
			fsSel = 0x01;
			scale = 16.0f / 32768.0f;
			break;
		default:
			log("Unsupported accel range: %d", range);
			return false;
	}
	uint8_t reg = readRegister(REG_ACCEL_CONFIG0);
	reg = (fsSel << 5) | (reg & 0x1F);
	writeRegister(REG_ACCEL_CONFIG0, reg);
	accelScale_ = scale;

	return true;
}

bool ICM40609D::setGyroRange(const GyroRange range) {
	uint8_t fsSel;
	float scale;
	switch (range) {
		case GYRO_RANGE_MIN:
		case GYRO_RANGE_250DPS:
			fsSel = 0x03;
			scale = 250.0f / 32768.0f;
			break;
		case GYRO_RANGE_500DPS:
			fsSel = 0x02;
			scale = 500.0f / 32768.0f;
			break;
		case GYRO_RANGE_1000DPS:
			fsSel = 0x01;
			scale = 1000.0f / 32768.0f;
			break;
		case GYRO_RANGE_MAX:
		case GYRO_RANGE_2000DPS:
			fsSel = 0x00;
			scale = 2000.0f / 32768.0f;
			break;
		default:
			log("Unsupported gyro range: %d", range);
			return false;
	}
	uint8_t reg = readRegister(REG_GYRO_CONFIG0);
	reg = (fsSel << 5) | (reg & 0x1F);
	writeRegister(REG_GYRO_CONFIG0, reg);
	gyroScale_ = scale;
	return true;
}

bool ICM40609D::setDLPF(const DLPF dlpf) {
	/*
	 * GYRO_ACCEL_CONFIG0 (0x52):
	 *   Bits [7:4] = ACCEL_UI_FILT_BW
	 *   Bits [3:0] = GYRO_UI_FILT_BW
	 *
	 * BW codes (at 1 kHz ODR):
	 *   0: ODR/2   (500 Hz, effectively off)
	 *   1: ODR/4   (250 Hz)
	 *   4: ODR/10  (100 Hz)
	 *   6: ODR/20  (50 Hz)
	 *   7: ODR/40  (25 Hz)
	 */
	uint8_t bwCode;
	switch (dlpf) {
		case DLPF_OFF:
		case DLPF_MAX:
			bwCode = 0x00; // ODR/2 = maximum BW
			break;
		case DLPF_100HZ_APPROX:
			bwCode = 0x04; // ODR/10
			break;
		case DLPF_50HZ_APPROX:
			bwCode = 0x06; // ODR/20
			break;
		case DLPF_MIN:
		case DLPF_5HZ_APPROX:
			bwCode = 0x07; // ODR/40 (minimum)
			break;
		default:
			log("Unsupported DLPF setting");
			return false;
	}
	uint8_t val = (bwCode << 4) | bwCode; // same BW for gyro and accel
	writeRegister(REG_GYRO_ACCEL_CONFIG0, val);
	return true;
}

bool ICM40609D::setRate(const Rate rate) {
	switch (rate) {
		case RATE_MIN:
			return setODR(ODR_12_5HZ);
		case RATE_50HZ_APPROX:
			return setODR(ODR_50HZ);
		case RATE_1KHZ_APPROX:
			return setODR(ODR_1KHZ);
		case RATE_8KHZ_APPROX:
			return setODR(ODR_8KHZ);
		case RATE_MAX:
			return setODR(ODR_32KHZ);
		default:
			log("Unsupported rate setting");
			return false;
	}
}

float ICM40609D::getRate() {
	uint8_t reg = readRegister(REG_GYRO_CONFIG0);
	uint8_t odrCode = reg & 0x0F;
	return odrCodeToHz(odrCode);
}

bool ICM40609D::setupInterrupt() {
	bool res = IMUBase::setupInterrupt(intPin);
	if (intPin != -1 && res) {
		enableDataReadyInterrupt();
	}
	return res;
}

bool ICM40609D::enableDataReadyInterrupt() {
	// INT1: push-pull, pulsed, active low (compatible with FALLING edge)
	writeRegister(REG_INT_CONFIG, 0x02); // push-pull, pulsed, active low

	// Clear bit 4 of INT_CONFIG1 for proper INT operation
	uint8_t reg = readRegister(REG_INT_CONFIG1);
	reg &= ~0x10;
	writeRegister(REG_INT_CONFIG1, reg);

	// Route data ready to INT1
	writeRegister(REG_INT_SOURCE0, UI_DRDY_INT1_EN);
	return true;
}

bool ICM40609D::setODR(uint8_t odrCode) {
	// Set accel ODR
	uint8_t accelCfg = readRegister(REG_ACCEL_CONFIG0);
	accelCfg = (accelCfg & 0xF0) | odrCode;
	writeRegister(REG_ACCEL_CONFIG0, accelCfg);

	// Set gyro ODR
	uint8_t gyroCfg = readRegister(REG_GYRO_CONFIG0);
	gyroCfg = (gyroCfg & 0xF0) | odrCode;
	writeRegister(REG_GYRO_CONFIG0, gyroCfg);
	return true;
}

float ICM40609D::odrCodeToHz(uint8_t code) const {
	switch (code) {
		case ODR_32KHZ:  return 32000.0f;
		case ODR_16KHZ:  return 16000.0f;
		case ODR_8KHZ:   return 8000.0f;
		case ODR_4KHZ:   return 4000.0f;
		case ODR_2KHZ:   return 2000.0f;
		case ODR_1KHZ:   return 1000.0f;
		case ODR_200HZ:  return 200.0f;
		case ODR_100HZ:  return 100.0f;
		case ODR_50HZ:   return 50.0f;
		case ODR_25HZ:   return 25.0f;
		case ODR_12_5HZ: return 12.5f;
		case ODR_500HZ:  return 500.0f;
		default:         return 0.0f;
	}
}

/* ---- Low-level register I/O ---- */

void ICM40609D::switchBank(uint8_t bank) {
	if (bank == currentBank) return;
	currentBank = bank;
	// REG_BANK_SEL is accessible from any bank
	if (useSPI) {
		_spi->beginTransaction(spiSettingsCfg);
		digitalWrite(csPin, LOW);
		_spi->transfer(REG_BANK_SEL);
		_spi->transfer(bank);
		digitalWrite(csPin, HIGH);
		_spi->endTransaction();
	} else {
		_wire->beginTransmission(i2cAddress);
		_wire->write(REG_BANK_SEL);
		_wire->write(bank);
		_wire->endTransmission();
	}
}

void ICM40609D::writeRegister(uint8_t reg, uint8_t val) {
	if (useSPI) {
		_spi->beginTransaction(spiSettingsCfg);
		digitalWrite(csPin, LOW);
		_spi->transfer(reg & 0x7F); // write: bit 7 = 0
		_spi->transfer(val);
		digitalWrite(csPin, HIGH);
		_spi->endTransaction();
	} else {
		_wire->beginTransmission(i2cAddress);
		_wire->write(reg);
		_wire->write(val);
		_wire->endTransmission();
	}
}

uint8_t ICM40609D::readRegister(uint8_t reg) {
	uint8_t val = 0;
	if (useSPI) {
		_spi->beginTransaction(spiSettingsCfg);
		digitalWrite(csPin, LOW);
		_spi->transfer(reg | 0x80); // read: bit 7 = 1
		val = _spi->transfer(0x00);
		digitalWrite(csPin, HIGH);
		_spi->endTransaction();
	} else {
		_wire->beginTransmission(i2cAddress);
		_wire->write(reg);
		_wire->endTransmission(false);
		_wire->requestFrom(i2cAddress, static_cast<uint8_t>(1));
		if (_wire->available()) {
			val = _wire->read();
		}
	}
	return val;
}

void ICM40609D::readRegisters(uint8_t reg, uint8_t count, uint8_t* dest) {
	if (useSPI) {
		_spi->beginTransaction(spiSettingsData);
		digitalWrite(csPin, LOW);
		_spi->transfer(reg | 0x80); // read: bit 7 = 1
		for (uint8_t i = 0; i < count; i++) {
			dest[i] = _spi->transfer(0x00);
		}
		digitalWrite(csPin, HIGH);
		_spi->endTransaction();
	} else {
		_wire->beginTransmission(i2cAddress);
		_wire->write(reg);
		_wire->endTransmission(false);
		_wire->requestFrom(i2cAddress, count);
		for (uint8_t i = 0; i < count && _wire->available(); i++) {
			dest[i] = _wire->read();
		}
	}
}
