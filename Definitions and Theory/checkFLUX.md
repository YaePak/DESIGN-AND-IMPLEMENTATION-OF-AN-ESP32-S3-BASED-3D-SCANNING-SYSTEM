# TF-Luna — Đo giá trị FLUX để xác định ngưỡng TFL_MIN_VALID_FLUX
  ------------------------------------------------------------------

  Mục đích: chạy sketch này TRƯỚC, quan sát flux thật trên vật thể của
  bạn, rồi mới điền con số vào TFL_MIN_VALID_FLUX trong file scanner.

  Cách dùng — làm lần lượt 4 tình huống, ghi lại khoảng flux mỗi lần:

    1. Chĩa cảm biến vào vật thể sẽ quét, ở ĐÚNG khoảng cách quét dự
       kiến (khoảng 25-30cm). Xoay vật vài góc khác nhau.
       -> Đây là vùng flux "TỐT", cần giữ lại.

    2. Chĩa vào phần TỐI MÀU NHẤT của vật (nếu có).
       -> Đây là giá trị tốt thấp nhất, ngưỡng phải nằm DƯỚI mức này.

    3. Chĩa vào khoảng không (không có vật gì trong tầm 1-2m).
       -> Đây là vùng flux "RÁC", cần loại bỏ.

    4. Chĩa vào vật ở góc rất nghiêng (gần như song song với tia laser).
       -> Trường hợp khó nhất, hay gặp ở phần dốc đứng của vật thể.

  Chọn ngưỡng: nằm giữa giá trị thấp nhất của tình huống 1-2 và giá trị
  cao nhất của tình huống 3. Nếu hai vùng này chồng lấn nhau, chọn thiên
  về phía thấp (thà lọt vài điểm rác còn hơn thủng nhiều lỗ), rồi lọc
  tiếp bằng outlier rejection ở bước hậu xử lý.

  Sketch tự động in ra giá trị min/max của flux trong 50 lần đo gần nhất
  để bạn không phải tự nhẩm khi nhìn dòng chữ chạy.
*/

#include <Arduino.h>
#include <Wire.h>

const int I2C_SDA = 15;
const int I2C_SCL = 16;

const uint8_t TFL_ADDR     = 0x10;
const uint8_t TFL_DIST_LO  = 0x00;
const uint8_t TFL_SET_MODE = 0x23;
const uint8_t TFL_TRIGGER  = 0x24;

// Cửa sổ thống kê: in min/max sau mỗi 50 lần đo
const int WINDOW = 50;
int sampleCount = 0;
int fluxMin = 999999;
int fluxMax = 0;

void tfLunaInit() {
  Wire.beginTransmission(TFL_ADDR);
  Wire.write(TFL_SET_MODE);
  Wire.write(0x01);              // chế độ trigger
  if (Wire.endTransmission() != 0) {
    Serial.println("WARNING: TF-Luna not responding. Check wiring/address.");
  } else {
    Serial.println("TF-Luna OK, trigger mode.");
  }
  delay(10);
}

// Đọc cả khoảng cách và flux. Trả về false nếu lỗi I2C.
bool readRaw(int &distanceCm, int &flux) {
  Wire.beginTransmission(TFL_ADDR);
  Wire.write(TFL_TRIGGER);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(TFL_ADDR);
  Wire.write(TFL_DIST_LO);
  if (Wire.endTransmission(false) != 0) return false;

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
  tfLunaInit();

  Serial.println();
  Serial.println("=== TF-Luna flux measurement ===");
  Serial.println("Point the sensor at: (1) your object, (2) its darkest part,");
  Serial.println("(3) empty air, (4) a steeply angled surface.");
  Serial.println("Note the flux range for each case.");
  Serial.println();
}

void loop() {
  int dist, flux;

  if (!readRaw(dist, flux)) {
    Serial.println("I2C read error");
    delay(200);
    return;
  }

  Serial.print("dist=");
  Serial.print(dist);
  Serial.print(" cm   flux=");
  Serial.println(flux);

  // Cập nhật min/max trong cửa sổ hiện tại
  if (flux < fluxMin) fluxMin = flux;
  if (flux > fluxMax) fluxMax = flux;
  sampleCount++;

  if (sampleCount >= WINDOW) {
    Serial.print(">>> Last ");
    Serial.print(WINDOW);
    Serial.print(" samples: flux min=");
    Serial.print(fluxMin);
    Serial.print("  max=");
    Serial.println(fluxMax);
    Serial.println();
    sampleCount = 0;
    fluxMin = 999999;
    fluxMax = 0;
  }

  delay(200);   // chậm lại để đọc kịp trên Serial Monitor
}
