#include <FlixPeriph.h>
#include <SPI.h>

MPU9250 imu(SPI); // no need to specify CS pin, the default pin is used automatically

void setup() {
  Serial.begin(115200);
  bool success = imu.begin();
  if (!success) {
    while (1) {
      Serial.println("Failed to initialize IMU");
      delay(5000);
    }
  }
}

void loop() {
  float gx, gy, gz, ax, ay, az, mx, my, mz;
  imu.waitForData(); // blockingly read the data, use imu.read() for non-blocking read
  imu.getGyro(gx, gy, gz);
  imu.getAccel(ax, ay, az);
  imu.getMag(mx, my, mz);
  // Print data for Serial Plotter:
  Serial.print("gx:"); Serial.print(gx); Serial.print(" gy:"); Serial.print(gy); Serial.print(" gz:"); Serial.print(gz);
  Serial.print(" ax:"); Serial.print(ax); Serial.print(" ay:"); Serial.print(ay); Serial.print(" az:"); Serial.print(az);
  Serial.print(" mx:"); Serial.print(mx); Serial.print(" my:"); Serial.print(my); Serial.print(" mz:"); Serial.println(mz);
  delay(50); // slow down the output
}
