#include <MPU6050.h>

MPU6050 IMU(Wire);

void setup() {
  Serial.begin(115200);
  bool success = IMU.begin();
  if (!success) {
    while (1) {
      Serial.println("Failed to initialize IMU");
      delay(5000);
    }
  }
}

void loop() {
  float gx, gy, gz, ax, ay, az;
  IMU.waitForData(); // blockingly read the data, use IMU.read() for non-blocking read
  IMU.getGyro(gx, gy, gz);
  IMU.getAccel(ax, ay, az);
  // Print data for Serial Plotter:
  Serial.print("gx:"); Serial.print(gx); Serial.print(" gy:"); Serial.print(gy); Serial.print(" gz:"); Serial.print(gz);
  Serial.print(" ax:"); Serial.print(ax); Serial.print(" ay:"); Serial.print(ay); Serial.print(" az:"); Serial.print(az);
  delay(50); // slow down the output
}
