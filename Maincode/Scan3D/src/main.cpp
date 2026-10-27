#include <Arduino.h>

HardwareSerial TFSerial(1);

#define TF_RX 18
#define TF_TX 17

uint8_t frame[9];
uint8_t indexFrame = 0;

void setup() {
    Serial.begin(115200);
    TFSerial.begin(115200, SERIAL_8N1, TF_RX, TF_TX);

    Serial.println("TF-Luna Distance Test");
}

void loop() {
    static unsigned long lastPrint = 0;

    while (TFSerial.available()) {
        uint8_t b = TFSerial.read();

        if (indexFrame == 0 && b != 0x59) continue;

        if (indexFrame == 1 && b != 0x59) {
            indexFrame = 0;
            continue;
        }

        frame[indexFrame++] = b;

        if (indexFrame == 9) {
            indexFrame = 0;

            uint16_t checksum = 0;
            for (int i = 0; i < 8; i++) {
                checksum += frame[i];
            }

            if ((checksum & 0xFF) == frame[8]) {
                uint16_t distance =
                    frame[2] | (frame[3] << 8);

                if (millis() - lastPrint >= 1000) {
                    Serial.printf("Distance: %u cm\n", distance);
                    lastPrint = millis();
                }
            }
        }
    }
}