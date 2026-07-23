#include <HardwareSerial.h>

HardwareSerial RadarSerial(2);

#define C4001_RX_PIN 16
#define C4001_TX_PIN 17

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Test UART2 C4001");
  Serial.println("C4001 TX -> ESP32 GPIO16 RX2");
  Serial.println("C4001 RX -> ESP32 GPIO17 TX2");

  RadarSerial.begin(9600, SERIAL_8N1, C4001_RX_PIN, C4001_TX_PIN);
}

void loop() {
  while (RadarSerial.available()) {
    uint8_t b = RadarSerial.read();

    if (b < 16) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");
  }

  delay(10);
}