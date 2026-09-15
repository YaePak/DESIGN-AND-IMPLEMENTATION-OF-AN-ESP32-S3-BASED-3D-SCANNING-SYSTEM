/*
  Test TF-Luna qua I2C
  ------------------------------------------------
  Kiểm tra cảm biến TF-Luna hoạt động đúng, tách riêng khỏi motor và các
  chân GPIO khác. Chỉ cần nối TF-Luna vào bus I2C, không cần gì khác.

  Giả định: TF-Luna đã ở chế độ I2C (chân số 5 của TF-Luna nối GND).
  Chân 2 (SDA) và chân 3 (SCL) của TF-Luna nối vào GPIO15/16 ESP32-S3.

  In LIÊN TỤC, KHÔNG lọc gì cả — đây là công cụ chẩn đoán, cần thấy đúng
  số thô để tự đánh giá, khác với firmware chính vốn đã lọc theo flux.
  Dùng để: xác nhận cảm biến có phản hồi qua I2C không, xem khoảng cách
  đo có đúng thực tế không, và tìm ngưỡng TFL_MIN_VALID_FLUX hợp lý
  (đưa tay/vật lại gần xa để xem flux thay đổi thế nào).
*/

#include <Arduino.h>
#include <Wire.h>

const int I2C_SDA = 15;
const int I2C_SCL = 16;

const uint8_t TFL_ADDR     = 0x10;   // địa chỉ I2C mặc định
const uint8_t TFL_DIST_LO  = 0x00;   // byte thấp khoảng cách (0x01 = byte cao)
const uint8_t TFL_SET_MODE = 0x23;   // 0 = đo liên tục, 1 = đo theo lệnh
const uint8_t TFL_TRIGGER  = 0x24;   // ghi 1 = đo một lần

bool tfLunaInit() {
  Wire.beginTransmission(TFL_ADDR);
  Wire.write(TFL_SET_MODE);
  Wire.write(0x01);                  // chế độ trigger
  bool ok = (Wire.endTransmission() == 0);
  delay(10);
  return ok;
}

// Đọc thô, không lọc gì. Trả về false nếu lỗi I2C (không ACK).
bool readTFLuna(int& distanceCm, int& flux) {
  Wire.beginTransmission(TFL_ADDR);
  Wire.write(TFL_TRIGGER);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(10);                         // chờ cảm biến đo xong

  Wire.beginTransmission(TFL_ADDR);
  Wire.write(TFL_DIST_LO);
  if (Wire.endTransmission(false) != 0) return false;   // repeated start; khong ACK = loi day
  Wire.requestFrom((int)TFL_ADDR, 4);
  if (Wire.available() < 4) return false;

  uint8_t distLo = Wire.read();
  uint8_t distHi = Wire.read();
  uint8_t fluxLo = Wire.read();
  uint8_t fluxHi = Wire.read();
  distanceCm = (distHi << 8) | distLo;
  flux       = (fluxHi << 8) | fluxLo;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println();
  Serial.println("=== Test TF-Luna qua I2C ===");
  if (tfLunaInit()) {
    Serial.println("Da ket noi TF-Luna, dat che do trigger thanh cong.");
  } else {
    Serial.println("WARNING: TF-Luna khong phan hoi tren I2C.");
    Serial.println("  Kiem tra: day SDA/SCL co dao nguoc khong, chan 5 co that su noi GND");
    Serial.println("  (de vao che do I2C) khong, va nguon cap cho cam bien.");
  }
  Serial.println("Dua tay/vat truoc cam bien o cac khoang cach khac nhau de quan sat.");
  Serial.println();
}

void loop() {
  int dist, flux;
  if (readTFLuna(dist, flux)) {
    Serial.print("dist=");
    Serial.print(dist);
    Serial.print(" cm   flux=");
    Serial.println(flux);
  } else {
    Serial.println("Loi doc I2C -- kiem tra lai day noi.");
  }
  delay(200);
}
