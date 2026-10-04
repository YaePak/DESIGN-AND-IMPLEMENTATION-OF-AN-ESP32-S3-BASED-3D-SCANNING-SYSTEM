#include <Arduino.h>

const int PIN_TABLE_DIR  = 4;
const int PIN_TABLE_STEP = 5;
const int PIN_TABLE_EN   = 6;

const int PIN_Z_DIR  = 12;
const int PIN_Z_STEP = 11;
const int PIN_Z_EN   = 10;

const int MOTOR_STEPS_PER_REV = 200;   // NEMA17 1,8 do/buoc
const int MICROSTEPPING       = 16;    // TODO: khop voi jumper MS1/MS2/MS3 that tren A4988
const int TOTAL_STEPS_PER_REV = MOTOR_STEPS_PER_REV * MICROSTEPPING;
const int STEP_DELAY_US       = 600;   // TODO: khop voi gia tri dang dung trong firmware chinh
const float LEAD_SCREW_PITCH_MM = 8.0;

const int TABLE_DIR_LEVEL = HIGH;      // giong scan.cpp
const int Z_UP_LEVEL      = HIGH;      // giong scan.cpp

const int TEST_REVS               = 1;
const int PAUSE_BETWEEN_CYCLES_MS = 3000;

const float Z_TEST_MAX_MM = 150.0;

long  mmToSteps(float mm)   { return (long)(mm / LEAD_SCREW_PITCH_MM * TOTAL_STEPS_PER_REV + 0.5f); }
float stepsToMm(long steps) { return steps * LEAD_SCREW_PITCH_MM / TOTAL_STEPS_PER_REV; }

const long Z_TEST_MAX_STEPS = mmToSteps(Z_TEST_MAX_MM);

long zPosSteps  = 0;       // vi tri Z tinh bang xung, 0 = luc cap nguon
bool halted     = false;   // true = vuot gioi han mem -> dung han, can reset board
int  cycleCount = 0;

void enableDrivers() {
  digitalWrite(PIN_TABLE_EN, LOW);
  digitalWrite(PIN_Z_EN, LOW);
}

void stepPulse(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(5);              // A4988 can xung toi thieu ~1us
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY_US);
}

void halt(const char* msg) {
  halted = true;
  Serial.println();
  Serial.print("!!! LOI: ");
  Serial.print(msg);
  Serial.print("  z=");
  Serial.print(stepsToMm(zPosSteps), 2);
  Serial.println(" mm");
  Serial.println("!!! Motor DA DUNG va giu nguyen vi tri. Nhan nut RESET tren board de chay lai.");
}

void spinTable(bool forward, int revs) {
  Serial.print("TABLE");
  Serial.print(forward ? " quay THUAN " : " quay NGUOC ");
  Serial.print(revs);
  Serial.println(" vong...");

  const int backLevel = (TABLE_DIR_LEVEL == HIGH) ? LOW : HIGH;
  digitalWrite(PIN_TABLE_DIR, forward ? TABLE_DIR_LEVEL : backLevel);
  delayMicroseconds(5);              // A4988: DIR on dinh truoc canh len STEP

  long totalSteps = (long)revs * TOTAL_STEPS_PER_REV;
  for (long i = 0; i < totalSteps; i++) stepPulse(PIN_TABLE_STEP);
}

bool spinZ(bool up, int revs) {
  Serial.print("Z");
  Serial.print(up ? " LEN " : " XUONG ");
  Serial.print(revs);
  Serial.print(" vong (");
  Serial.print((long)revs * TOTAL_STEPS_PER_REV);
  Serial.println(" xung)...");

  const int downLevel = (Z_UP_LEVEL == HIGH) ? LOW : HIGH;
  digitalWrite(PIN_Z_DIR, up ? Z_UP_LEVEL : downLevel);
  delayMicroseconds(5);

  long steps = (long)revs * TOTAL_STEPS_PER_REV;
  for (long i = 0; i < steps; i++) {
    if (up && zPosSteps >= Z_TEST_MAX_STEPS) { halt("VUOT GIOI HAN TREN (Z_TEST_MAX_MM)"); return false; }
    if (!up && zPosSteps <= 0)               { halt("VUOT GIOI HAN DUOI (thap hon vi tri luc cap nguon)"); return false; }
    stepPulse(PIN_Z_STEP);
    zPosSteps += up ? 1 : -1;
  }
  return true;
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

  enableDrivers();

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
  Serial.println("Vi tri Z luc cap nguon = z=0: truc Z chi di LEN roi ve, khong xuong thap hon.");
  delay(2000);
}

void loop() {
  if (halted) {          // da vuot gioi han mem: dung yen cho reset
    delay(1000);
    return;
  }

  cycleCount++;
  Serial.println();
  Serial.print("===== CHU KY #");
  Serial.print(cycleCount);
  Serial.println(" =====");

  Serial.println("--- BAN XOAY ---");
  spinTable(true, TEST_REVS);
  delay(500);
  spinTable(false, TEST_REVS);
  Serial.println("TABLE xong -- kiem tra diem danh dau da ve DUNG vi tri ban dau chua.");

  delay(1000);

  Serial.println("--- TRUC Z ---");
  if (!spinZ(true, TEST_REVS)) return;
  delay(1000);
  if (!spinZ(false, TEST_REVS)) return;
  Serial.println("Z xong -- kiem tra diem danh dau da ve DUNG vi tri ban dau chua.");

  Serial.print("Chu ky #");
  Serial.print(cycleCount);
  Serial.println(" hoan tat. Nghi truoc khi lap lai...");
  delay(PAUSE_BETWEEN_CYCLES_MS);
}