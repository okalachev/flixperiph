#include <FlixPeriph.h>

CRSF rc(Serial1);

void setup() {
  Serial.begin(115200);
  rc.begin(16, -1, true, 400000); // RX=GPIO16, TX unused, inverted, 400k baud
}

void loop() {
  if (rc.read()) {
    uint16_t channels[16];
    rc.getChannels(channels);
    Serial.print("Channels: ");
    for (int i = 0; i < 16; i++) {
      Serial.print(channels[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}
