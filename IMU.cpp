#include "FlixPeriph.h"

IMU* IMU::create(int model, SPIClass& spi, int cs, int drdy) {
	switch (model) {
		case 1: return new MPU9250(spi, cs, drdy);
		case 2: return new ICM20948(spi, cs, drdy);
		case 4: return new ICM40609D(spi, cs, drdy);
		default: return new IMU();
	}
}

IMU* IMU::create(int model, TwoWire& i2c, int drdy) {
	switch (model) {
		case 1: return new MPU9250(i2c, drdy);
		case 2: return new ICM20948(i2c, drdy);
		case 3: return new MPU6050(i2c, drdy);
		case 4: return new ICM40609D(i2c, drdy);
		default: return new IMU();
	}
}
