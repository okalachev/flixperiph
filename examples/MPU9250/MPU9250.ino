#include <MPU9250.h>
#include <SPI.h>

MPU9250 IMU(SPI); // no need to specify CS pin, the default pin is used automatically

void setup() {
  Serial.begin(115200);
  bool success = IMU.begin();
  if (!success) {
    Serial.println("Failed to initialize IMU");
    while (1) {}
  }
}

void loop() {
  float gx, gy, gz, ax, ay, az, mx, my, mz;
  IMU.waitForData(); // blockingly read the data, use IMU.read() for non-blocking read
  IMU.getGyro(gx, gy, gz);
  IMU.getAccel(ax, ay, az);
  IMU.getMag(mx, my, mz);
  Serial.print("gx: "); Serial.print(gx); Serial.print(" "); Serial.print("gy: "); Serial.print(gy); Serial.print(" "); Serial.print("gz: "); Serial.print(gz);
  Serial.print(" ax: "); Serial.print(ax); Serial.print(" "); Serial.print("ay: "); Serial.print(ay); Serial.print(" "); Serial.print("az: "); Serial.print(az);
  Serial.print(" mx: "); Serial.print(mx); Serial.print(" "); Serial.print("my: "); Serial.print(my); Serial.print(" "); Serial.print("mz: "); Serial.println(mz);
}
