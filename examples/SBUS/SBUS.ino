#include <SBUS.h>

SBUS RC(Serial2); // Using signal pin 4, software inversion enabled

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
  Serial.println();
}
