#include <Arduino.h>

const int PIN_TFLUNA_RX = 17;   // ESP32 RX <- chân TXD của TF-Luna
const int PIN_TFLUNA_TX = 18;   // ESP32 TX -> chân RXD của TF-Luna

HardwareSerial TFLunaSerial(1);
const uint32_t TFLUNA_BAUD = 115200;
const uint32_t TFLUNA_READ_TIMEOUT_MS = 50;

// Đọc thô, không lọc gì theo strength — chỉ báo lỗi khung/timeout.
// Trả về false nếu không nhận được khung hợp lệ trong thời gian chờ.
bool readTFLunaRaw(int& distanceCm, int& strength) {
  while (TFLunaSerial.available()) TFLunaSerial.read();   // xả khung cũ, lấy khung mới nhất

  uint8_t buf[9];
  int idx = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < TFLUNA_READ_TIMEOUT_MS) {
    if (!TFLunaSerial.available()) continue;
    uint8_t b = TFLunaSerial.read();

    if (idx < 2) {
      if (b == 0x59) buf[idx++] = b;
      else idx = 0;
    } else {
      buf[idx++] = b;
      if (idx == 9) {
        uint8_t sum = 0;
        for (int i = 0; i < 8; i++) sum += buf[i];
        if (sum != buf[8]) { idx = 0; continue; }   // sai checksum, tim khung ke tiep

        distanceCm = buf[2] | (buf[3] << 8);
        strength    = buf[4] | (buf[5] << 8);
        return true;
      }
    }
  }
  return false;
}

// Cửa sổ thống kê: in min/max strength sau mỗi 50 lần đo
const int WINDOW = 50;
int sampleCount = 0;
int strengthMin = 999999;
int strengthMax = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  TFLunaSerial.begin(TFLUNA_BAUD, SERIAL_8N1, PIN_TFLUNA_RX, PIN_TFLUNA_TX);

  Serial.println();
  Serial.println("=== Test TF-Luna qua UART ===");
  delay(100);
  int d, s;
  if (readTFLunaRaw(d, s)) {
    Serial.println("Da nhan duoc khung hop le tu TF-Luna.");
  } else {
    Serial.println("WARNING: chua nhan duoc khung hop le.");
    Serial.println("  Kiem tra: TXD/RXD co dao nguoc khong, nguon cap cho cam bien,");
    Serial.println("  toc do baud co dung 115200 (mac dinh) khong.");
  }
  Serial.println("Point sensor at object/dark surface/empty air/steep angle in turn.");
  Serial.println();
}

void loop() {
  int dist, strength;
  if (readTFLunaRaw(dist, strength)) {
    Serial.print("dist=");
    Serial.print(dist);
    Serial.print(" cm   strength=");
    Serial.println(strength);

    if (strength < strengthMin) strengthMin = strength;
    if (strength > strengthMax) strengthMax = strength;
    sampleCount++;
    if (sampleCount >= WINDOW) {
      Serial.print(">>> Last ");
      Serial.print(WINDOW);
      Serial.print(" samples: strength min=");
      Serial.print(strengthMin);
      Serial.print("  max=");
      Serial.println(strengthMax);
      Serial.println();
      sampleCount = 0;
      strengthMin = 999999;
      strengthMax = 0;
    }
  } else {
    Serial.println("Khong nhan duoc khung hop le (timeout/checksum loi).");
  }
  delay(100);
}
