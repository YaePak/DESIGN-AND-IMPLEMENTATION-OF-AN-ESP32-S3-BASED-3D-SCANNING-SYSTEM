/*
  Test TF-Luna qua UART
  ------------------------------------------------
  Kiểm tra cảm biến TF-Luna hoạt động đúng qua UART, tách riêng khỏi motor
  và các chân GPIO khác. Chỉ cần nối TF-Luna, không cần gì khác.

  Nối: TXD của TF-Luna -> GPIO17 (ESP32 RX)
       RXD của TF-Luna -> GPIO18 (ESP32 TX)

  In LIÊN TỤC, KHÔNG lọc gì cả — đây là công cụ chẩn đoán, cần thấy đúng
  số thô để tự đánh giá, khác với firmware chính vốn đã lọc theo strength.
  Dùng để: xác nhận cảm biến có phát khung dữ liệu hợp lệ không, xem
  khoảng cách đo có đúng thực tế không, và tìm ngưỡng
  TFLUNA_MIN_VALID_STRENGTH hợp lý (đưa tay/vật lại gần xa để xem độ mạnh
  tín hiệu thay đổi thế nào).

  QUY TRÌNH ĐO — làm lần lượt 4 tình huống, ghi lại khoảng strength mỗi lần:
    1. Chĩa vào vật thể ở khoảng cách quét thật (~28cm), xoay vài góc.
       -> Đây là vùng "TỐT", cần giữ lại.
    2. Chĩa vào phần TỐI MÀU NHẤT của vật.
       -> Giá trị tốt thấp nhất, ngưỡng phải nằm DƯỚI mức này.
    3. Chĩa vào khoảng không, không có vật trong 1-2m.
       -> Vùng "RÁC", cần loại bỏ.
    4. Chĩa vào vật ở góc rất nghiêng (gần song song với tia).
       -> Trường hợp khó nhất, hay gặp ở phần dốc đứng của vật thể.
  Chọn ngưỡng nằm giữa giá trị thấp nhất của tình huống 1-2 và cao nhất
  của tình huống 3.
*/

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
