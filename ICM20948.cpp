/*
 * Code based on: https://github.com/wollewald/ICM20948_WE.
 * Author: Wolfgang (Wolle) Ewald.
 * License: MIT (see LICENSE file).
 * Further information can be found on:
 * https://wolles-elektronikkiste.de/icm-20948-9-achsensensor-teil-i (German)
 * https://wolles-elektronikkiste.de/en/icm-20948-9-axis-sensor-part-i (English)
*/

#include "ICM20948.h"

/************ Basic Settings ************/

bool ICM20948::begin() {
	if (useSPI) {
		_spi->begin();
		if (csPin == -1) {
			// use default CS pin
			#ifdef ESP32
				csPin = _spi->pinSS();
			#else
				csPin = SS;
			#endif
		}
		pinMode(csPin, OUTPUT);
		digitalWrite(csPin, HIGH);
		mySPISettings = SPISettings(7000000, MSBFIRST, SPI_MODE0);
	}
	currentBank = 0;

	reset();
	if (whoAmI() != ICM20948_WHO_AM_I_CONTENT) {
		delay(2000);
		if (whoAmI() != ICM20948_WHO_AM_I_CONTENT){
			log("Error: incorrect WHO_AM_I value: 0x%02X", whoAmI());
			return false;
		}
	}

	accRangeFactor = 1.0;
	gyrRangeFactor = 1.0;
	fifoType = ICM20948_FIFO_ACC;

	sleep(false);
	writeRegister8(2, ICM20948_ODR_ALIGN_EN, 1); // aligns ODR

	initMagnetometer();

	return true;
}

uint8_t ICM20948::whoAmI() {
	return readRegister8(0, ICM20948_WHO_AM_I);
}

void ICM20948::enableAcc(bool enAcc) {
	regVal = readRegister8(0, ICM20948_PWR_MGMT_2);
	if (enAcc) {
		regVal &= ~ICM20948_ACC_EN;
	} else {
		regVal |= ICM20948_ACC_EN;
	}
	writeRegister8(0, ICM20948_PWR_MGMT_2, regVal);
}

bool ICM20948::setAccelRange(const AccelRange range) {
	int accRange;
	switch (range) {
		case ACCEL_RANGE_MIN:
		case ACCEL_RANGE_2G:
			accRange = ICM20948_ACC_RANGE_2G;
			break;
		case ACCEL_RANGE_4G:
			accRange = ICM20948_ACC_RANGE_4G;
			break;
		case ACCEL_RANGE_8G:
			accRange = ICM20948_ACC_RANGE_8G;
			break;
		case ACCEL_RANGE_16G:
			accRange = ICM20948_ACC_RANGE_16G;
			break;
		default:
			log("Unsupported accel range: %d", range);
			return false;
	}
	regVal = readRegister8(2, ICM20948_ACCEL_CONFIG);
	regVal &= ~(0x06);
	regVal |= (accRange<<1);
	writeRegister8(2, ICM20948_ACCEL_CONFIG, regVal);
	accRangeFactor = 1<<accRange;
	return true;
}

void ICM20948::setAccDLPF(ICM20948_dlpf dlpf) {
	regVal = readRegister8(2, ICM20948_ACCEL_CONFIG);
	if(dlpf == ICM20948_DLPF_OFF){
		regVal &= 0xFE;
		writeRegister8(2, ICM20948_ACCEL_CONFIG, regVal);
		return;
	} else {
		regVal |= 0x01;
		regVal &= 0xC7;
		regVal |= (dlpf<<3);
	}
	writeRegister8(2, ICM20948_ACCEL_CONFIG, regVal);
}

void ICM20948::setAccSampleRateDivider(uint16_t accSplRateDiv) {
	writeRegister16(2, ICM20948_ACCEL_SMPLRT_DIV_1, accSplRateDiv);
}

void ICM20948::enableGyr(bool enGyr) {
	regVal = readRegister8(0, ICM20948_PWR_MGMT_2);
	if (enGyr) {
		regVal &= ~ICM20948_GYR_EN;
	} else {
		regVal |= ICM20948_GYR_EN;
	}
	writeRegister8(0, ICM20948_PWR_MGMT_2, regVal);
}

bool ICM20948::setGyroRange(const GyroRange range) {
	int gyroRange;
	switch (range) {
		case GYRO_RANGE_MIN:
		case GYRO_RANGE_250DPS:
			gyroRange = ICM20948_GYRO_RANGE_250;
			break;
		case GYRO_RANGE_500DPS:
			gyroRange = ICM20948_GYRO_RANGE_500;
			break;
		case GYRO_RANGE_1000DPS:
			gyroRange = ICM20948_GYRO_RANGE_1000;
			break;
		case GYRO_RANGE_2000DPS:
			gyroRange = ICM20948_GYRO_RANGE_2000;
			break;
		default:
			log("Unsupported gyro range: %d", range);
			return false;
	}
	regVal = readRegister8(2, ICM20948_GYRO_CONFIG_1);
	regVal &= ~(0x06);
	regVal |= (gyroRange<<1);
	writeRegister8(2, ICM20948_GYRO_CONFIG_1, regVal);
	gyrRangeFactor = (1<<gyroRange);
	return true;
}

void ICM20948::setGyrDLPF(ICM20948_dlpf dlpf) {
	regVal = readRegister8(2, ICM20948_GYRO_CONFIG_1);
	if (dlpf == ICM20948_DLPF_OFF) {
		regVal &= 0xFE;
		writeRegister8(2, ICM20948_GYRO_CONFIG_1, regVal);
		return;
	} else {
		regVal |= 0x01;
		regVal &= 0xC7;
		regVal |= (dlpf<<3);
	}
	writeRegister8(2, ICM20948_GYRO_CONFIG_1, regVal);
}

void ICM20948::setGyrSampleRateDivider(uint8_t gyrSplRateDiv) {
	writeRegister8(2, ICM20948_GYRO_SMPLRT_DIV, gyrSplRateDiv);
}

void ICM20948::setTempDLPF(ICM20948_dlpf dlpf) {
	writeRegister8(2, ICM20948_TEMP_CONFIG, dlpf);
}

bool ICM20948::setDLPF(const DLPF dlpf) {
	/*  ICMD20948 DLPF table:
	 *
	 *  DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
     *    0              246.0               1125/(1+ASRD) (default)
     *    1              246.0               1125/(1+ASRD)
     *    2              111.4               1125/(1+ASRD)
     *    3               50.4               1125/(1+ASRD)
     *    4               23.9               1125/(1+ASRD)
     *    5               11.5               1125/(1+ASRD)
     *    6                5.7               1125/(1+ASRD)
     *    7              473.0               1125/(1+ASRD)
     *    OFF           1209.0               4500
     *
     *    ASRD = Accelerometer Sample Rate Divider (0...4095)
	 */
	ICM20948_dlpf val;
	switch (dlpf) {
		case DLPF_OFF:
			val = ICM20948_DLPF_OFF;
			break;
		case DLPF_MAX:
			val = ICM20948_DLPF_7;
			break;
		case DLPF_100HZ_APPROX:
			val = ICM20948_DLPF_2;
			break;
		case DLPF_50HZ_APPROX:
			val = ICM20948_DLPF_3;
			break;
		case DLPF_MIN:
		case DLPF_5HZ_APPROX:
			val = ICM20948_DLPF_6;
			break;
		default:
			log("Unsupported DLPF setting");
			return false;
	}
	setAccDLPF(val);
	setGyrDLPF(val);
	return true;
}

void ICM20948::setI2CMstSampleRate(uint8_t rateExp) {
	if(rateExp < 16){
		writeRegister8(3, ICM20948_I2C_MST_ODR_CFG, rateExp);
	}
}

void ICM20948::setSPIClockSpeed(unsigned long clock) {
	mySPISettings = SPISettings(clock, MSBFIRST, SPI_MODE0);
}

bool ICM20948::setRate(const Rate rate) {
	// output rate = 1125 Hz / (1 + srd)
	// TODO: Implement this function
	switch(rate) {
		case RATE_MIN: // 1.125 Hz
			setAccSampleRateDivider(0x4F);
			setGyrSampleRateDivider(0x4F);
			setI2CMstSampleRate(0x4F);
			return true;
		case RATE_50HZ_APPROX: // 37 Hz
			setAccSampleRateDivider(0x07);
			setGyrSampleRateDivider(0x07);
			setI2CMstSampleRate(0x07);
			return true;
		case RATE_MAX:
		case RATE_1KHZ_APPROX: // 1125 Hz
			setAccSampleRateDivider(0);
			setGyrSampleRateDivider(0);
			setI2CMstSampleRate(0);
			return true;
		default:
			log("Unsupported rate setting");
			return false;
	}
}

/************* Results *************/

bool ICM20948::read() {
	readAllData(buffer);
	return true;
}

void ICM20948::waitForData() {
	const static uint8_t RAW_DATA_0_RDY_INT = 0x01;
	while (true) {
		uint8_t intStatus1 = readRegister8(0, ICM20948_INT_STATUS_1);
		bool dataReady = intStatus1 & RAW_DATA_0_RDY_INT;
		if (dataReady) break;
	}
	read();
}

void ICM20948::getAccel(float& x, float& y, float& z) const {
	x = static_cast<int16_t>(((buffer[0]) << 8) | buffer[1]) * 1.0;
	y = static_cast<int16_t>(((buffer[2]) << 8) | buffer[3]) * 1.0;
	z = static_cast<int16_t>(((buffer[4]) << 8) | buffer[5]) * 1.0;
	// raw to g
	x = x * accRangeFactor / 16384.0;
	y = y * accRangeFactor / 16384.0;
	z = z * accRangeFactor / 16384.0;
	// convert to m/s^2
	static constexpr float G = 9.80665f;
	x = x * G;
	y = y * G;
	z = z * G;
}

float ICM20948::getTemp() const {
	int16_t rawTemp = static_cast<int16_t>(((buffer[12]) << 8) | buffer[13]);
	float tmp = (rawTemp*1.0 - ICM20948_ROOM_TEMP_OFFSET)/ICM20948_T_SENSITIVITY + 21.0;
	return tmp;
}

void ICM20948::getGyro(float& x, float& y, float& z) const {
	x = (int16_t)(((buffer[6]) << 8) | buffer[7]) * 1.0;
	y = (int16_t)(((buffer[8]) << 8) | buffer[9]) * 1.0;
	z = (int16_t)(((buffer[10]) << 8) | buffer[11]) * 1.0;
	// raw to dps
	x = x * gyrRangeFactor * 250.0 / 32768.0;
	y = y * gyrRangeFactor * 250.0 / 32768.0;
	z = z * gyrRangeFactor * 250.0 / 32768.0;
	// convert to rad/s
	x = x * DEG_TO_RAD;
	y = y * DEG_TO_RAD;
	z = z * DEG_TO_RAD;
}

void ICM20948::getMag(float& x, float& y, float& z) const {
	x = static_cast<int16_t>((buffer[15]) << 8) | buffer[14];
	y = static_cast<int16_t>((buffer[17]) << 8) | buffer[16];
	z = static_cast<int16_t>((buffer[19]) << 8) | buffer[18];
	// correct values
	x = x * AK09916_MAG_LSB;
	y = y * AK09916_MAG_LSB;
	z = z * AK09916_MAG_LSB;
	// orient magnetometer to match accel and gyro
	x = x;
	y = -y;
	z = -z;
}


/********* Power, Sleep, Standby *********/

void ICM20948::enableCycle(ICM20948_cycle cycle){
	regVal = readRegister8(0, ICM20948_LP_CONFIG);
	regVal &= 0x0F;
	regVal |= cycle;

	writeRegister8(0, ICM20948_LP_CONFIG, regVal);
}

void ICM20948::enableLowPower(bool enLP) { // vielleicht besser privat????
	regVal = readRegister8(0, ICM20948_PWR_MGMT_1);
	if (enLP) {
		regVal |= ICM20948_LP_EN;
	} else {
		regVal &= ~ICM20948_LP_EN;
	}
	writeRegister8(0, ICM20948_PWR_MGMT_1, regVal);
}

void ICM20948::setGyrAverageInCycleMode(ICM20948_gyroAvgLowPower avg) {
	writeRegister8(2, ICM20948_GYRO_CONFIG_2, avg);
}

void ICM20948::setAccAverageInCycleMode(ICM20948_accAvgLowPower avg) {
	writeRegister8(2, ICM20948_ACCEL_CONFIG_2, avg);
}

void ICM20948::sleep(bool sleep) {
	regVal = readRegister8(0, ICM20948_PWR_MGMT_1);
	if(sleep) {
		regVal |= ICM20948_SLEEP;
	} else {
		regVal &= ~ICM20948_SLEEP;
	}
	writeRegister8(0, ICM20948_PWR_MGMT_1, regVal);
}

/************** Interrupts ***************/

void ICM20948::setIntPinPolarity(ICM20948_intPinPol pol) {
	regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
	if (pol) {
		regVal |= ICM20948_INT1_ACTL;
	} else {
		regVal &= ~ICM20948_INT1_ACTL;
	}
	writeRegister8(0, ICM20948_INT_PIN_CFG, regVal);
}

void ICM20948::enableIntLatch(bool latch) {
	regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
	if (latch) {
		regVal |= ICM20948_INT_1_LATCH_EN;
	} else {
		regVal &= ~ICM20948_INT_1_LATCH_EN;
	}
	writeRegister8(0, ICM20948_INT_PIN_CFG, regVal);
}

void ICM20948::enableClearIntByAnyRead(bool clearByAnyRead) {
	regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
	if (clearByAnyRead){
		regVal |= ICM20948_INT_ANYRD_2CLEAR;
	} else {
		regVal &= ~ICM20948_INT_ANYRD_2CLEAR;
	}
	writeRegister8(0, ICM20948_INT_PIN_CFG, regVal);
}

void ICM20948::setFSyncIntPolarity(ICM20948_intPinPol pol) {
	regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
	if (pol) {
		regVal |= ICM20948_ACTL_FSYNC;
	} else{
		regVal &= ~ICM20948_ACTL_FSYNC;
	}
	writeRegister8(0, ICM20948_INT_PIN_CFG, regVal);
}

void ICM20948::enableInterrupt(ICM20948_intType intType) {
	switch(intType){
		case ICM20948_FSYNC_INT:
			regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
			regVal |= ICM20948_FSYNC_INT_MODE_EN;
			writeRegister8(0, ICM20948_INT_PIN_CFG, regVal); // enable FSYNC as interrupt pin
			regVal = readRegister8(0, ICM20948_INT_ENABLE);
			regVal |= 0x80;
			writeRegister8(0, ICM20948_INT_ENABLE, regVal); // enable wake on FSYNC interrupt
			break;

		case ICM20948_WOM_INT:
			regVal = readRegister8(0, ICM20948_INT_ENABLE);
			regVal |= 0x08;
			writeRegister8(0, ICM20948_INT_ENABLE, regVal);
			regVal = readRegister8(2, ICM20948_ACCEL_INTEL_CTRL);
			regVal |= 0x02;
			writeRegister8(2, ICM20948_ACCEL_INTEL_CTRL, regVal);
			break;

		case ICM20948_DMP_INT:
			regVal = readRegister8(0, ICM20948_INT_ENABLE);
			regVal |= 0x02;
			writeRegister8(0, ICM20948_INT_ENABLE, regVal);
			break;

		case ICM20948_DATA_READY_INT:
			writeRegister8(0, ICM20948_INT_ENABLE_1, 0x01);
			break;

		case ICM20948_FIFO_OVF_INT:
			writeRegister8(0, ICM20948_INT_ENABLE_2, 0x01);
			break;

		case ICM20948_FIFO_WM_INT:
			writeRegister8(0, ICM20948_INT_ENABLE_3, 0x01);
			break;
	}
}

void ICM20948::disableInterrupt(ICM20948_intType intType) {
	switch (intType) {
		case ICM20948_FSYNC_INT:
			regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
			regVal &= ~ICM20948_FSYNC_INT_MODE_EN;
			writeRegister8(0, ICM20948_INT_PIN_CFG, regVal);
			regVal = readRegister8(0, ICM20948_INT_ENABLE);
			regVal &= ~(0x80);
			writeRegister8(0, ICM20948_INT_ENABLE, regVal);
			break;

		case ICM20948_WOM_INT:
			regVal = readRegister8(0, ICM20948_INT_ENABLE);
			regVal &= ~(0x08);
			writeRegister8(0, ICM20948_INT_ENABLE, regVal);
			regVal = readRegister8(2, ICM20948_ACCEL_INTEL_CTRL);
			regVal &= ~(0x02);
			writeRegister8(2, ICM20948_ACCEL_INTEL_CTRL, regVal);
			break;

		case ICM20948_DMP_INT:
			regVal = readRegister8(0, ICM20948_INT_ENABLE);
			regVal &= ~(0x02);
			writeRegister8(0, ICM20948_INT_ENABLE, regVal);
			break;

		case ICM20948_DATA_READY_INT:
			writeRegister8(0, ICM20948_INT_ENABLE_1, 0x00);
			break;

		case ICM20948_FIFO_OVF_INT:
			writeRegister8(0, ICM20948_INT_ENABLE_2, 0x00);
			break;

		case ICM20948_FIFO_WM_INT:
			writeRegister8(0, ICM20948_INT_ENABLE_3, 0x00);
			break;
	}
}

uint8_t ICM20948::readAndClearInterrupts() {
	uint8_t intSource = 0;
	regVal = readRegister8(0, ICM20948_I2C_MST_STATUS);
	if (regVal & 0x80) {
		intSource |= 0x01;
	}
	regVal = readRegister8(0, ICM20948_INT_STATUS);
	if (regVal & 0x08) {
		intSource |= 0x02;
	}
	if (regVal & 0x02) {
		intSource |= 0x04;
	}
	regVal = readRegister8(0, ICM20948_INT_STATUS_1);
	if (regVal & 0x01) {
		intSource |= 0x08;
	}
	regVal = readRegister8(0, ICM20948_INT_STATUS_2);
	if (regVal & 0x01) {
		intSource |= 0x10;
	}
	regVal = readRegister8(0, ICM20948_INT_STATUS_3);
	if (regVal & 0x01) {
		intSource |= 0x20;
	}
	return intSource;
}

bool ICM20948::checkInterrupt(uint8_t source, ICM20948_intType type) {
	source &= type;
	return source;
}

void ICM20948::setWakeOnMotionThreshold(uint8_t womThresh, ICM20948_womCompEn womCompEn) {
	regVal = readRegister8(2, ICM20948_ACCEL_INTEL_CTRL);
	if(womCompEn) {
		regVal |= 0x01;
	} else {
		regVal &= ~(0x01);
	}
	writeRegister8(2, ICM20948_ACCEL_INTEL_CTRL, regVal);
	writeRegister8(2, ICM20948_ACCEL_WOM_THR, womThresh);
}

/***************** FIFO ******************/

void ICM20948::enableFifo(bool fifo) {
	regVal = readRegister8(0, ICM20948_USER_CTRL);
	if (fifo) {
		regVal |= ICM20948_FIFO_EN;
	} else {
		regVal &= ~ICM20948_FIFO_EN;
	}
	writeRegister8(0, ICM20948_USER_CTRL, regVal);
}

void ICM20948::setFifoMode(ICM20948_fifoMode mode) {
	if (mode) {
		regVal = 0x01;
	} else {
		regVal = 0x00;
	}
	writeRegister8(0, ICM20948_FIFO_MODE, regVal);
}

void ICM20948::startFifo(ICM20948_fifoType fifo) {
	fifoType = fifo;
	writeRegister8(0, ICM20948_FIFO_EN_2, fifoType);
}

void ICM20948::stopFifo() {
	writeRegister8(0, ICM20948_FIFO_EN_2, 0);
}

void ICM20948::resetFifo() {
	writeRegister8(0, ICM20948_FIFO_RST, 0x01);
	writeRegister8(0, ICM20948_FIFO_RST, 0x00);
}

int16_t ICM20948::getFifoCount() {
	int16_t regVal16 = static_cast<int16_t>(readRegister16(0, ICM20948_FIFO_COUNT));
	return regVal16;
}

int16_t ICM20948::getNumberOfFifoDataSets() {
	int16_t numberOfSets = getFifoCount();

	if ((fifoType == ICM20948_FIFO_ACC) || (fifoType == ICM20948_FIFO_GYR)) {
		numberOfSets /= 6;
	} else if (fifoType==ICM20948_FIFO_ACC_GYR) {
		numberOfSets /= 12;
	}

	return numberOfSets;
}

void ICM20948::findFifoBegin() {
	uint16_t count = getFifoCount();
	int16_t start = 0;

	if ((fifoType == ICM20948_FIFO_ACC) || (fifoType == ICM20948_FIFO_GYR)) {
		start = count%6;
		for(int i=0; i<start; i++){
			readRegister8(0, ICM20948_FIFO_R_W);
		}
	} else if (fifoType==ICM20948_FIFO_ACC_GYR) {
		start = count%12;
		for(int i=0; i<start; i++){
			readRegister8(0, ICM20948_FIFO_R_W);
		}
	}
}

bool ICM20948::initMagnetometer() {
	enableI2CMaster();
	resetMag();
	reset();
	sleep(false);
	writeRegister8(2, ICM20948_ODR_ALIGN_EN, 1); // aligns ODR

	bool initSuccess = false;
	uint8_t tries = 0;
	while (!initSuccess && (tries < 10)) { // max. 10 tries to init the magnetometer
		delay(10);
		enableI2CMaster();
		delay(10);

		int16_t whoAmI = whoAmIMag();
		if (!((whoAmI == AK09916_WHO_AM_I_1) || (whoAmI == AK09916_WHO_AM_I_2))) {
			initSuccess = false;
			i2cMasterReset();
			tries++;
		}
		else {
			initSuccess = true;
		}
	}
	if (initSuccess) {
		setMagOpMode(AK09916_CONT_MODE_100HZ);
	}
	return initSuccess;
}

uint16_t ICM20948::whoAmIMag() {
	return static_cast<uint16_t>(readAK09916Register16(AK09916_WIA_1));
}

void ICM20948::setMagOpMode(AK09916_opMode opMode) {
	writeAK09916Register8(AK09916_CNTL_2, opMode);
	delay(10);
	if(opMode!=AK09916_PWR_DOWN){
		enableMagDataRead(AK09916_HXL, 0x08);
	}
}

void ICM20948::resetMag() {
	writeAK09916Register8(AK09916_CNTL_3, 0x01);
	delay(100);
}

/************************************************
     Private Functions
*************************************************/

void ICM20948::setClockToAutoSelect() {
	regVal = readRegister8(0, ICM20948_PWR_MGMT_1);
	regVal |= 0x01;
	writeRegister8(0, ICM20948_PWR_MGMT_1, regVal);
	delay(10);
}

void ICM20948::switchBank(uint8_t newBank) {
	if (newBank != currentBank) {
		currentBank = newBank;
		if (!useSPI) {
			_wire->beginTransmission(i2cAddress);
			_wire->write(ICM20948_REG_BANK_SEL);
			_wire->write(currentBank<<4);
			_wire->endTransmission();
		} else {
			_spi->beginTransaction(mySPISettings);
			digitalWrite(csPin, LOW);
			_spi->transfer(ICM20948_REG_BANK_SEL);
			_spi->transfer(currentBank<<4);
			digitalWrite(csPin, HIGH);
			_spi->endTransaction();
		}
	}
}

void ICM20948::writeRegister8(uint8_t bank, uint8_t reg, uint8_t val) {
	switchBank(bank);

	if (!useSPI) {
		_wire->beginTransmission(i2cAddress);
		_wire->write(reg);
		_wire->write(val);
		_wire->endTransmission();
	} else {
		_spi->beginTransaction(mySPISettings);
		digitalWrite(csPin, LOW);
		_spi->transfer(reg);
		_spi->transfer(val);
		digitalWrite(csPin, HIGH);
		_spi->endTransaction();
	}
}

void ICM20948::writeRegister16(uint8_t bank, uint8_t reg, int16_t val) {
	switchBank(bank);
	int8_t MSByte = static_cast<int8_t>((val>>8) & 0xFF);
	uint8_t LSByte = val & 0xFF;

	if (!useSPI) {
		_wire->beginTransmission(i2cAddress);
		_wire->write(reg);
		_wire->write(MSByte);
		_wire->write(LSByte);
		_wire->endTransmission();
	} else {
		_spi->beginTransaction(mySPISettings);
		digitalWrite(csPin, LOW);
		_spi->transfer(reg);
		_spi->transfer(val);
		digitalWrite(csPin, HIGH);
		_spi->endTransaction();
	}
}

uint8_t ICM20948::readRegister8(uint8_t bank, uint8_t reg) {
	switchBank(bank);
	uint8_t regValue = 0;

	if(!useSPI){
		_wire->beginTransmission(i2cAddress);
		_wire->write(reg);
		_wire->endTransmission(false);
		_wire->requestFrom(i2cAddress, static_cast<uint8_t>(1));
		if(_wire->available()){
			regValue = _wire->read();
		}
	} else {
		reg |= 0x80;
		_spi->beginTransaction(mySPISettings);
		digitalWrite(csPin, LOW);
		_spi->transfer(reg);
		regValue = _spi->transfer(0x00);
		digitalWrite(csPin, HIGH);
		_spi->endTransaction();
	}
	return regValue;
}

int16_t ICM20948::readRegister16(uint8_t bank, uint8_t reg) {
	switchBank(bank);
	uint8_t MSByte = 0, LSByte = 0;
	int16_t reg16Val = 0;

	if (!useSPI) {
		_wire->beginTransmission(i2cAddress);
		_wire->write(reg);
		_wire->endTransmission(false);
		_wire->requestFrom(i2cAddress, static_cast<uint8_t>(2));
		if(_wire->available()){
			MSByte = _wire->read();
			LSByte = _wire->read();
		}
	} else {
		reg = reg | 0x80;
		_spi->beginTransaction(mySPISettings);
		digitalWrite(csPin, LOW);
		_spi->transfer(reg);
		MSByte = _spi->transfer(0x00);
		LSByte = _spi->transfer(0x00);
		digitalWrite(csPin, HIGH);
		_spi->endTransaction();
	}
	reg16Val = (MSByte<<8) + LSByte;
	return reg16Val;
}

void ICM20948::readAllData(uint8_t* data) {
	switchBank(0);

	if(!useSPI){
		_wire->beginTransmission(i2cAddress);
		_wire->write(ICM20948_ACCEL_OUT);
		_wire->endTransmission(false);
		_wire->requestFrom(i2cAddress, static_cast<uint8_t>(20));
		if(_wire->available()){
			for(int i=0; i<20; i++){
				data[i] = _wire->read();
			}
		}
	}
	else{
		uint8_t reg = ICM20948_ACCEL_OUT | 0x80;
		_spi->beginTransaction(mySPISettings);
		digitalWrite(csPin, LOW);
		_spi->transfer(reg);
		for(int i=0; i<20; i++){
				data[i] = _spi->transfer(0x00);
		}
		digitalWrite(csPin, HIGH);
		_spi->endTransaction();
	}
}

void ICM20948::writeAK09916Register8(uint8_t reg, uint8_t val) {
	writeRegister8(3, ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS); // write AK09916
	writeRegister8(3, ICM20948_I2C_SLV0_REG, reg); // define AK09916 register to be written to
	writeRegister8(3, ICM20948_I2C_SLV0_DO, val);
}


uint8_t ICM20948::readAK09916Register8(uint8_t reg) {
	enableMagDataRead(reg, 0x01);
	enableMagDataRead(AK09916_HXL, 0x08);
	regVal = readRegister8(0, ICM20948_EXT_SLV_SENS_DATA_00);
	return regVal;
}

int16_t ICM20948::readAK09916Register16(uint8_t reg) {
	int16_t regValue = 0;
	enableMagDataRead(reg, 0x02);
	regValue = readRegister16(0, ICM20948_EXT_SLV_SENS_DATA_00);
	enableMagDataRead(AK09916_HXL, 0x08);
	return regValue;
}

void ICM20948::reset() {
	writeRegister8(0, ICM20948_PWR_MGMT_1, ICM20948_RESET);
	delay(10);  // wait for registers to reset
}

void ICM20948::enableI2CMaster() {
	writeRegister8(0, ICM20948_USER_CTRL, ICM20948_I2C_MST_EN); //enable I2C master
	writeRegister8(3, ICM20948_I2C_MST_CTRL, 0x07); // set I2C clock to 345.60 kHz
	delay(10);
}

void ICM20948::i2cMasterReset() {
	uint8_t regVal = readRegister8(0, ICM20948_USER_CTRL);
	regVal |= ICM20948_I2C_MST_RST;
	writeRegister8(0, ICM20948_USER_CTRL, regVal);
	delay(10);
}

void ICM20948::enableMagDataRead(uint8_t reg, uint8_t bytes) {
	writeRegister8(3, ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS | AK09916_READ); // read AK09916
	writeRegister8(3, ICM20948_I2C_SLV0_REG, reg); // define AK09916 register to be read
	writeRegister8(3, ICM20948_I2C_SLV0_CTRL, 0x80 | bytes); //enable read | number of byte
	delay(10);
}
