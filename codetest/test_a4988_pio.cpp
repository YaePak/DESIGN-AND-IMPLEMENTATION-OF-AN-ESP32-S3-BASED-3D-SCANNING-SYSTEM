#include <Arduino.h>

// A4988 — Bàn xoay
const int PIN_TABLE_DIR  = 4;
const int PIN_TABLE_STEP = 5;
const int PIN_TABLE_EN   = 6;
// A4988 — Trục Z
const int PIN_Z_DIR  = 10;
const int PIN_Z_STEP = 11;
const int PIN_Z_EN   = 12;
// Nút kích hoạt chu kỳ test
const int PIN_HOME_BUTTON = 7;

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
  Serial.println("Nhan nut HOME (GPIO7) de bat dau 1 chu ky test...");
  while (digitalRead(PIN_HOME_BUTTON) == HIGH) delay(50);   // HIGH = chua nhan
  delay(200);                                               // chong rung
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
  pinMode(PIN_HOME_BUTTON, INPUT_PULLUP);

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
