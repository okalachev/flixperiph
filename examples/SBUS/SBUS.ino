#include <FlixPeriph.h>

SBUS rc(Serial2); // Using signal pin 4, software inversion enabled

void setup() {
  Serial.begin(115200);
  rc.begin();
}

void loop() {
  if (!rc.read()) return;
  uint16_t channels[16];
  rc.getChannels(channels);
  for (int i = 0; i < 16; i++) {
    Serial.print(channels[i]);
    Serial.print(" ");
  }
  Serial.println();
}
