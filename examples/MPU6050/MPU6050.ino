#include <FlixPeriph.h>

MPU6050 imu(Wire);

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
  float gx, gy, gz, ax, ay, az;
  imu.waitForData(); // blockingly read the data, use imu.read() for non-blocking read
  imu.getGyro(gx, gy, gz);
  imu.getAccel(ax, ay, az);
  // Print data for Serial Plotter:
  Serial.print("gx:"); Serial.print(gx); Serial.print(" gy:"); Serial.print(gy); Serial.print(" gz:"); Serial.print(gz);
  Serial.print(" ax:"); Serial.print(ax); Serial.print(" ay:"); Serial.print(ay); Serial.print(" az:"); Serial.print(az);
  Serial.println();
  delay(50); // slow down the output
}
