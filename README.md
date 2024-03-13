# Arduino drivers for the Flix drone peripherals

This library contains Arduino drivers for the peripherals used in the [Flix drone](https://github.com/okalachev/flix) project.

Based on:

* [Bolder Flight InvenSense-IMU library](https://github.com/bolderflight/invensense-imu) (version 1.0.2). Original author: Brian Taylor (brian.taylor@bolderflight.com).

## MPU9250 IMU

Example for SPI-connected IMU:

```cpp
#include <MPU9250.h>
#include <SPI.h>

MPU9250 IMU(SPI); // no need to specify CS pin, the default pin is used automatically

void setup() {
  IMU.begin();
}

void loop() {
  float gx, gy, gz, ax, ay, az, mx, my, mz;
  IMU.waitForData(); // blockingly read the data, use IMU.read() for non-blocking read
  IMU.getGyro(gx, gy, gz);
  IMU.getAccel(ax, ay, az);
  IMU.getMag(mx, my, mz);
}
```
