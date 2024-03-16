#include <SBUS.h>

SBUS RC(Serial2, true); // Using Serial2, software inversion enabled

void setup() {
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
