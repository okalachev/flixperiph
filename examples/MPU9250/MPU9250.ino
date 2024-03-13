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
