# Arduino drivers for the Flix drone peripherals

This library contains Arduino drivers for the peripherals used in the [Flix drone](https://github.com/okalachev/flix) project.

Based on:

* [Bolder Flight InvenSense-IMU library](https://github.com/bolderflight/invensense-imu). Original author: Brian Taylor (brian.taylor@bolderflight.com).
* [Bolder Flight SBUS library](https://github.com/bolderflight/sbus). Original author: Brian Taylor (brian.taylor@bolderflight.com).

## MPU9250/MPU6500 IMU

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
  // Process the data...
}
```

The library will detect IMU type (MPU9250 or MPU6500) automatically. You can use the following method to check the detected type:

```cpp
if (IMU.isMPU6500()) {
  // MPU6500 detected
} else {
  // MPU9250 detected
}
```

Notice, that MPU6500 does not include a magnetometer, so magnetometer data will be always zero.

You can also use `<MPU6500.h>` header and `MPU6500` class, which is an alias for `MPU9250` class.

## SBUS

Example for SBUS receiver, connected to Serial2:

```cpp
#include <SBUS.h>

SBUS RC(Serial2, true); // Using Serial2, software inversion enabled

void setup() {
  Serial.begin(115200);
  RC.begin();
}

void loop() {
  if (!RC.read()) return;
  SBUSData data = RC.data();
  for (int i = 0; i < data.NUM_CH; i++) {
    Serial.print(data.ch[i]);
    Serial.print(" ");
  }
}
```
