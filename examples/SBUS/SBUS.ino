#include <FlixPeriph.h>

SBUS rc(Serial2); // Using signal pin 4, software inversion enabled

void setup() {
  Serial.begin(115200);
  rc.begin();
}

void loop() {
  if (!rc.read()) return;
  SBUSData data = rc.data();
  for (int i = 0; i < data.NUM_CH; i++) {
    Serial.print(data.ch[i]);
    Serial.print(" ");
  }
  Serial.println();
}
