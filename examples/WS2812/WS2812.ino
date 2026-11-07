#include <FlixPeriph.h>

WS2812 led;

const RGB rainbow[] = {
  {255, 0, 0},
  {255, 127, 0},
  {255, 255, 0},
  {0, 255, 0},
  {0, 0, 255},
  {75, 0, 130},
  {148, 0, 211}
};

void setup() {
  Serial.begin(115200);
  led.begin(RGB_BUILTIN - SOC_GPIO_PIN_COUNT, 1);

  led.setBrightness(0.1);
}

void loop() {
  static size_t color = 0;
  Serial.print("Showing rainbow color: ");
  Serial.println(color);

  led.write(&rainbow[color]);
  color = (color + 1) % (sizeof(rainbow) / sizeof(rainbow[0]));
  delay(300);
}
