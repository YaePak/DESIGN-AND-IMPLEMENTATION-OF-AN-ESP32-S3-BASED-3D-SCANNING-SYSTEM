/*
  Test driver A4988 + động cơ bước (bàn xoay + trục Z)
  ------------------------------------------------------
  Kiểm tra riêng 2 driver A4988, tách khỏi TF-Luna và WiFi. Nhấn nút Start
  để bắt đầu 1 chu kỳ test, quan sát động cơ quay bằng mắt.

  Cần nối: 2 driver A4988 + 2 động cơ NEMA17, nút Start vào GPIO1.
  KHÔNG cần TF-Luna, KHÔNG cần WiFi.

  Chu kỳ test (mỗi lần nhấn nút):
    1. Bàn xoay: quay THUẬN đúng 1 vòng -> dừng 1s -> quay NGƯỢC đúng 1 vòng
    2. Trục Z  : quay THUẬN đúng 1 vòng -> dừng 1s -> quay NGƯỢC đúng 1 vòng

  CÁCH KIỂM TRA BẰNG MẮT — quan trọng nhất của bài test này:
  Trước khi nhấn nút, đánh dấu 1 điểm tham chiếu trên trục động cơ (băng
  keo nhỏ hoặc bút lông). Sau khi quay thuận rồi quay ngược đúng 1 vòng,
  dấu đó PHẢI quay về CHÍNH XÁC vị trí ban đầu. Nếu lệch đi (dù chỉ một
  chút), động cơ đang bị trượt bước — cần giảm STEP_DELAY_US (quay chậm
  lại), kiểm tra Vref trên A4988 (dòng cấp có đủ không), hoặc kiểm tra cơ
  khí có bị kẹt/ma sát không.

  Cũng nên lắng nghe: động cơ chạy êm là bình thường; tiếng rít/khục khặc
  bất thường là dấu hiệu trượt bước hoặc sai điện áp Vref.
*/

#include <Arduino.h>

const int PIN_TABLE_STEP = 5;
const int PIN_TABLE_DIR  = 6;
const int PIN_TABLE_EN   = 7;
const int PIN_Z_STEP     = 8;
const int PIN_Z_DIR      = 9;
const int PIN_Z_EN       = 14;
const int PIN_BUTTON     = 1;

// Khớp với cấu hình đang dùng trong firmware chính — đổi theo nếu khác.
const int MOTOR_STEPS_PER_REV = 200;   // NEMA17 1,8 do/buoc
const int MICROSTEPPING       = 16;    // TODO: khop voi jumper MS1/MS2/MS3 that tren A4988
const int TOTAL_STEPS_PER_REV = MOTOR_STEPS_PER_REV * MICROSTEPPING;
const int STEP_DELAY_US       = 600;   // TODO: khop voi gia tri dang dung trong firmware chinh

void stepPulse(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(5);              // A4988 can xung toi thieu ~1us
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY_US);
}

void spinMotor(const char* name, int stepPin, int dirPin, bool forward, int steps) {
  Serial.print(name);
  Serial.print(forward ? " quay THUAN, " : " quay NGUOC, ");
  Serial.print(steps);
  Serial.println(" xung...");
  digitalWrite(dirPin, forward ? HIGH : LOW);   // TODO: dao lai neu THUAN/NGUOC bi nguoc thuc te
  for (int i = 0; i < steps; i++) stepPulse(stepPin);
}

void waitForButton() {
  Serial.println();
  Serial.println("Nhan nut Start (GPIO1) de bat dau 1 chu ky test...");
  while (digitalRead(PIN_BUTTON) == HIGH) delay(50);   // HIGH = chua nhan
  delay(200);                                          // chong rung
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_TABLE_STEP, OUTPUT);
  pinMode(PIN_TABLE_DIR,  OUTPUT);
  pinMode(PIN_TABLE_EN,   OUTPUT);
  pinMode(PIN_Z_STEP,     OUTPUT);
  pinMode(PIN_Z_DIR,      OUTPUT);
  pinMode(PIN_Z_EN,       OUTPUT);
  pinMode(PIN_BUTTON,     INPUT_PULLUP);

  digitalWrite(PIN_TABLE_EN, LOW);   // EN cua A4988 tich cuc muc THAP -> LOW = bat driver
  digitalWrite(PIN_Z_EN, LOW);

  Serial.println();
  Serial.println("=== Test A4988 + dong co buoc (ban xoay + truc Z) ===");
  Serial.print("Cau hinh: ");
  Serial.print(TOTAL_STEPS_PER_REV);
  Serial.print(" xung/vong (");
  Serial.print(MOTOR_STEPS_PER_REV);
  Serial.print(" full-step x 1/");
  Serial.print(MICROSTEPPING);
  Serial.println(" microstepping)");
  Serial.println("Danh dau 1 diem tham chieu tren truc dong co truoc khi test.");
}

void loop() {
  waitForButton();

  Serial.println("--- BAN XOAY ---");
  spinMotor("TABLE", PIN_TABLE_STEP, PIN_TABLE_DIR, true, TOTAL_STEPS_PER_REV);
  delay(1000);
  spinMotor("TABLE", PIN_TABLE_STEP, PIN_TABLE_DIR, false, TOTAL_STEPS_PER_REV);
  Serial.println("TABLE xong 1 vong thuan + 1 vong nguoc -- kiem tra diem danh dau da ve dung cho chua.");

  delay(1000);

  Serial.println("--- TRUC Z ---");
  spinMotor("Z", PIN_Z_STEP, PIN_Z_DIR, true, TOTAL_STEPS_PER_REV);
  delay(1000);
  spinMotor("Z", PIN_Z_STEP, PIN_Z_DIR, false, TOTAL_STEPS_PER_REV);
  Serial.println("Z xong 1 vong thuan + 1 vong nguoc -- kiem tra diem danh dau da ve dung cho chua.");

  Serial.println("=== Chu ky test hoan tat ===");
}
