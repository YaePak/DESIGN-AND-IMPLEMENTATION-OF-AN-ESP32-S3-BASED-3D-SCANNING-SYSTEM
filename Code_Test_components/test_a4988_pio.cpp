#include <Arduino.h>

const int PIN_TABLE_DIR  = 4;
const int PIN_TABLE_STEP = 5;
const int PIN_TABLE_EN   = 6;

const int PIN_Z_DIR  = 10;
const int PIN_Z_STEP = 11;
const int PIN_Z_EN   = 12;

const int MOTOR_STEPS_PER_REV = 200;   // NEMA17 1,8 do/buoc
const int MICROSTEPPING       = 16;    // TODO: khop voi jumper MS1/MS2/MS3 that tren A4988
const int TOTAL_STEPS_PER_REV = MOTOR_STEPS_PER_REV * MICROSTEPPING;
const int STEP_DELAY_US       = 600;   // TODO: khop voi gia tri dang dung trong firmware chinh

const int TEST_REVS = 1;

const int PAUSE_BETWEEN_CYCLES_MS = 3000;

int cycleCount = 0;

void stepPulse(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(5);              // A4988 can xung toi thieu ~1us
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY_US);
}

void spinRevs(const char *name, int stepPin, int dirPin, bool forward, int revs) {
  Serial.print(name);
  Serial.print(forward ? " quay THUAN " : " quay NGUOC ");
  Serial.print(revs);
  Serial.print(" vong (");
  Serial.print((long)revs * TOTAL_STEPS_PER_REV);
  Serial.println(" xung)...");

  digitalWrite(dirPin, forward ? HIGH : LOW);   // TODO: dao lai neu THUAN/NGUOC bi nguoc thuc te
  long totalSteps = (long)revs * TOTAL_STEPS_PER_REV;
  for (long i = 0; i < totalSteps; i++) stepPulse(stepPin);
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

  digitalWrite(PIN_Z_EN, LOW);

  Serial.println();
  Serial.println("=== Test on dinh A4988 + dong co buoc (tu chay, khong can nut) ===");
  Serial.print("Cau hinh: ");
  Serial.print(TOTAL_STEPS_PER_REV);
  Serial.print(" xung/vong (");
  Serial.print(MOTOR_STEPS_PER_REV);
  Serial.print(" full-step x 1/");
  Serial.print(MICROSTEPPING);
  Serial.println(" microstepping)");
  Serial.print("STEP_DELAY_US = ");
  Serial.print(STEP_DELAY_US);
  Serial.print(" us   |   Moi chu ky quay ");
  Serial.print(TEST_REVS);
  Serial.println(" vong lien tiep moi chieu, lap lai lien tuc");
  Serial.println("Danh dau 1 diem tham chieu tren truc dong co TRUOC KHI cap nguon.");
  delay(2000);  
}

void loop() {
  cycleCount++;
  Serial.println();
  Serial.print("===== CHU KY #");
  Serial.print(cycleCount);
  Serial.println(" =====");

  Serial.println("--- BAN XOAY ---");
  spinRevs("TABLE", PIN_TABLE_STEP, PIN_TABLE_DIR, true, TEST_REVS);
  delay(500);
  spinRevs("TABLE", PIN_TABLE_STEP, PIN_TABLE_DIR, false, TEST_REVS);
  Serial.println("TABLE xong -- kiem tra diem danh dau da ve DUNG vi tri ban dau chua.");

  delay(1000);

  Serial.println("--- TRUC Z ---");
  spinRevs("Z", PIN_Z_STEP, PIN_Z_DIR, true, TEST_REVS);
  delay(1000);
  spinRevs("Z", PIN_Z_STEP, PIN_Z_DIR, false, TEST_REVS);
  Serial.println("Z xong -- kiem tra diem danh dau da ve DUNG vi tri ban dau chua.");

  Serial.print("Chu ky #");
  Serial.print(cycleCount);
  Serial.println(" hoan tat. Nghi truoc khi lap lai...");
  delay(PAUSE_BETWEEN_CYCLES_MS);
}
