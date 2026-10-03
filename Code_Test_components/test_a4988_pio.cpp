#include <Arduino.h>

const int PIN_TABLE_DIR  = 4;
const int PIN_TABLE_STEP = 5;
const int PIN_TABLE_EN   = 6;

const int PIN_Z_DIR  = 12;
const int PIN_Z_STEP = 11;
const int PIN_Z_EN   = 10;

const int PIN_HOME_BUTTON = 7;

const int MOTOR_STEPS_PER_REV = 200;   // NEMA17 1,8 do/buoc
const int MICROSTEPPING       = 16;    // TODO: khop voi jumper MS1/MS2/MS3 that tren A4988
const int TOTAL_STEPS_PER_REV = MOTOR_STEPS_PER_REV * MICROSTEPPING;
const int STEP_DELAY_US       = 600;   // TODO: khop voi gia tri dang dung trong firmware chinh
const float LEAD_SCREW_PITCH_MM = 8.0;

const int TABLE_DIR_LEVEL = HIGH;      
const int Z_UP_LEVEL      = HIGH;      

const int TEST_REVS = 1;

const int PAUSE_BETWEEN_CYCLES_MS = 3000;

const float    Z_TEST_MAX_MM            = 200.0;  // <= hanh trinh co khi thuc te
const bool     DISABLE_DRIVERS_ON_FAULT = false;
const bool     AUTO_START               = true;   // true: tu chay khi cap nguon nhu ban cu
const uint32_t BUTTON_DEBOUNCE_MS       = 30;
const uint32_t BUTTON_LONG_MS           = 2000;

long  mmToSteps(float mm)   { return (long)(mm / LEAD_SCREW_PITCH_MM * TOTAL_STEPS_PER_REV + 0.5f); }
float stepsToMm(long steps) { return steps * LEAD_SCREW_PITCH_MM / TOTAL_STEPS_PER_REV; }

const long Z_TEST_MAX_STEPS = mmToSteps(Z_TEST_MAX_MM);

enum TestState { ST_RUNNING, ST_PAUSED, ST_FAULT };
TestState state = AUTO_START ? ST_RUNNING : ST_PAUSED;

long zPosSteps     = 0;       // vi tri Z tinh bang xung, 0 = luc cap nguon
long zCycleBase    = 0;       // vi tri Z dau moi chu ky
bool stopRequested = false;
bool busy          = false;   // dang phat xung
bool needZero      = false;   // sau khi nha motor: lan chay sau lay vi tri moi lam z=0
char faultMsg[64]  = "";

int cycleCount = 0;

void enableDrivers() {
  digitalWrite(PIN_TABLE_EN, LOW);
  digitalWrite(PIN_Z_EN, LOW);
}

void disableDrivers() {
  digitalWrite(PIN_TABLE_EN, HIGH);
  digitalWrite(PIN_Z_EN, HIGH);
}

void raiseFault(const char* msg) {
  state = ST_FAULT;
  strncpy(faultMsg, msg, sizeof(faultMsg) - 1);
  faultMsg[sizeof(faultMsg) - 1] = 0;
  Serial.println();
  Serial.print("!!! LOI: ");
  Serial.print(msg);
  Serial.print("  z=");
  Serial.print(stepsToMm(zPosSteps), 2);
  Serial.println(" mm");
  Serial.println("!!! Motor DA DUNG, dung yen. Nhan HOME de chay tiep, giu HOME 2s de nha motor.");
  if (DISABLE_DRIVERS_ON_FAULT) disableDrivers();
}

void printStatus() {
  Serial.println("----- TRANG THAI -----");
  Serial.print("Trang thai: ");
  Serial.println(state == ST_RUNNING ? "DANG CHAY" : state == ST_PAUSED ? "TAM DUNG" : "LOI");
  if (state == ST_FAULT) { Serial.print("Loi       : "); Serial.println(faultMsg); }
  Serial.print("Vi tri Z  : "); Serial.print(stepsToMm(zPosSteps), 2);
  Serial.print(" mm (");        Serial.print(zPosSteps); Serial.println(" xung)");
  Serial.print("Gioi han  : 0 .. "); Serial.print(Z_TEST_MAX_MM, 1); Serial.println(" mm");
  Serial.print("So chu ky : "); Serial.println(cycleCount);
  Serial.println("----------------------");
}

void printHelp() {
  Serial.println("----- NUT HOME -----");
  Serial.println("  Nhan khi dang chay: DUNG ngay");
  Serial.println("  Nhan khi dang dung: chay tiep");
  Serial.println("  Giu 2 giay        : nha motor de chinh tay");
  Serial.println("----- LENH Serial Monitor (115200, Enter) -----");
  Serial.println("  stop       dung ngay");
  Serial.println("  run        chay tiep chu ky test");
  Serial.println("  clear      xoa loi -> tam dung");
  Serial.println("  status     xem trang thai");
  Serial.println("  up <mm>    nang Z (khi dang dung)");
  Serial.println("  down <mm>  ha Z   (khi dang dung)");
  Serial.println("  zero       coi vi tri hien tai la z=0");
  Serial.println("  off        nha motor (giong giu HOME 2 giay)");
  Serial.println("  help       in danh sach nay");
  Serial.println("-----------------------------------------------");
}

void requestStop(const char* who) {
  if (busy) stopRequested = true;
  if (state == ST_RUNNING) state = ST_PAUSED;
  Serial.print(">> ");
  Serial.print(who);
  Serial.println(": DUNG");
}

void resumeRun(const char* who) {
  if (state == ST_FAULT) {
    Serial.print(">> Xoa loi: ");
    Serial.println(faultMsg);
    faultMsg[0] = 0;
  }
  if (needZero) {
    zPosSteps  = 0;
    zCycleBase = 0;
    needZero   = false;
    Serial.println(">> Lay vi tri hien tai lam z=0.");
  }
  enableDrivers();
  state = ST_RUNNING;
  Serial.print(">> ");
  Serial.print(who);
  Serial.println(": CHAY TIEP");
}

void releaseMotors() {
  disableDrivers();
  needZero = true;
  if (state == ST_RUNNING) state = ST_PAUSED;
  Serial.println(">> Da NHA motor. Chinh tay xong nhan HOME de chay tiep (vi tri moi = z=0).");
}

void pollButton() {
  static int      lastRaw  = HIGH;
  static int      stable   = HIGH;
  static uint32_t tChange  = 0;
  static uint32_t tPress   = 0;
  static bool     consumed = false;

  int raw = digitalRead(PIN_HOME_BUTTON);
  uint32_t now = millis();
  if (raw != lastRaw) { lastRaw = raw; tChange = now; }

  if (raw != stable && now - tChange >= BUTTON_DEBOUNCE_MS) {
    stable = raw;
    if (stable == LOW) {                         // vua nhan xuong
      tPress   = now;
      consumed = false;
      if (state == ST_RUNNING || busy) {
        requestStop("Nut HOME");
        consumed = true;
      }
    } else if (!consumed) {                      // nha ra sau lan nhan ngan
      resumeRun("Nut HOME");
    }
  }

  if (stable == LOW && !consumed && !busy && now - tPress >= BUTTON_LONG_MS) {
    consumed = true;
    releaseMotors();
  }
}

bool moveZSteps(long steps, bool up);

static char pendingLine[40] = "";   // lenh nhan trong luc motor quay, chay sau

void handleLine(const char* line) {
  char word[16] = {0};
  float arg = 0;
  if (sscanf(line, "%15s %f", word, &arg) < 1) return;

  if (busy && strcmp(word, "stop") && strcmp(word, "s") && strcmp(word, "status")) {
    strncpy(pendingLine, line, sizeof(pendingLine) - 1);
    pendingLine[sizeof(pendingLine) - 1] = 0;
    Serial.println(">> Motor dang quay - lenh se chay sau (nhan HOME / go 'stop' de dung ngay).");
    return;
  }

  if (!strcmp(word, "stop") || !strcmp(word, "s")) {
    requestStop("Lenh stop");
  } else if (!strcmp(word, "status")) {
    printStatus();
  } else if (!strcmp(word, "help") || !strcmp(word, "?")) {
    printHelp();
  } else if (!strcmp(word, "run")) {
    resumeRun("Lenh run");
  } else if (!strcmp(word, "clear")) {
    if (state == ST_FAULT) state = ST_PAUSED;
    faultMsg[0] = 0;
    Serial.println(">> Da xoa loi. Nhan HOME hoac go 'run' de chay tiep.");
  } else if (!strcmp(word, "zero")) {
    zPosSteps  = 0;
    zCycleBase = 0;
    needZero   = false;
    Serial.println(">> Da dat z=0 tai vi tri hien tai.");
  } else if (!strcmp(word, "off")) {
    releaseMotors();
  } else if (!strcmp(word, "up") || !strcmp(word, "down")) {
    if (arg <= 0) { Serial.println(">> Can so mm > 0, vd: up 5"); return; }
    if (state == ST_RUNNING) state = ST_PAUSED;
    bool up = !strcmp(word, "up");
    if (moveZSteps(mmToSteps(arg), up)) {
      Serial.print(">> Xong. z=");
      Serial.print(stepsToMm(zPosSteps), 2);
      Serial.println(" mm");
    }
  } else {
    Serial.print(">> Lenh khong hop le: ");
    Serial.println(word);
  }
}

// Doc lenh tu Serial Monitor (goi ca trong vong lap phat xung).
void pollSerial() {
  static char line[40];
  static size_t len = 0;

  if (!busy && pendingLine[0]) {          // chay lenh da giu lai
    char cmd[40];
    strcpy(cmd, pendingLine);
    pendingLine[0] = 0;
    handleLine(cmd);
  }

  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch != '\r' && ch != '\n') {
      if (len < sizeof(line) - 1) line[len++] = (char)tolower(ch);
      continue;
    }
    if (len == 0) continue;
    line[len] = 0;
    len = 0;
    char cmd[40];
    strcpy(cmd, line);                    // ban sao: handleLine co the goi lai pollSerial
    handleLine(cmd);
  }
}

bool checkStop(long stepIndex) {
  pollButton();                                // moi xung (~0,6 ms) -> khong bo lot lan nhan
  if ((stepIndex & 0x3F) == 0) pollSerial();   // moi 64 xung doc Serial 1 lan (~40 ms)
  return stopRequested;
}

void stepPulse(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(5);              // A4988 can xung toi thieu ~1us
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY_US);
}

// Ban xoay: khong co gioi han hanh trinh, chi kiem tra lenh dung.
bool spinTable(bool forward, int revs) {
  Serial.print("TABLE");
  Serial.print(forward ? " quay THUAN " : " quay NGUOC ");
  Serial.print(revs);
  Serial.println(" vong...");

  enableDrivers();
  const int backLevel = (TABLE_DIR_LEVEL == HIGH) ? LOW : HIGH;
  digitalWrite(PIN_TABLE_DIR, forward ? TABLE_DIR_LEVEL : backLevel);
  delayMicroseconds(5);

  busy = true;
  bool ok = true;
  long totalSteps = (long)revs * TOTAL_STEPS_PER_REV;
  for (long i = 0; i < totalSteps; i++) {
    if (checkStop(i)) { ok = false; break; }
    stepPulse(PIN_TABLE_STEP);
  }
  busy = false;
  if (!ok) {
    stopRequested = false;
    Serial.println(">> Ban xoay da dung. Nhan HOME de chay tiep.");
  }
  return ok;
}

bool moveZSteps(long steps, bool up) {
  if (steps <= 0) return true;
  enableDrivers();
  const int downLevel = (Z_UP_LEVEL == HIGH) ? LOW : HIGH;
  digitalWrite(PIN_Z_DIR, up ? Z_UP_LEVEL : downLevel);
  delayMicroseconds(5);

  busy = true;
  bool ok = true;
  for (long i = 0; i < steps; i++) {
    if (checkStop(i)) {
      stopRequested = false;
      Serial.println(">> Truc Z da dung. Nhan HOME de chay tiep.");
      ok = false;
      break;
    }
    if (up && zPosSteps >= Z_TEST_MAX_STEPS) {
      raiseFault("VUOT GIOI HAN TREN (Z_TEST_MAX_MM)");
      ok = false;
      break;
    }
    if (!up && zPosSteps <= 0) {
      Serial.println("[z] Da o z=0, khong ha them.");
      break;
    }
    stepPulse(PIN_Z_STEP);
    zPosSteps += up ? 1 : -1;
  }
  busy = false;
  return ok;
}

bool spinZ(bool up, int revs) {
  Serial.print("Z");
  Serial.print(up ? " LEN " : " XUONG ");
  Serial.print(revs);
  Serial.print(" vong (");
  Serial.print((long)revs * TOTAL_STEPS_PER_REV);
  Serial.println(" xung)...");
  return moveZSteps((long)revs * TOTAL_STEPS_PER_REV, up);
}

bool waitMs(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    pollButton();
    pollSerial();
    if (state != ST_RUNNING) return false;
    delay(5);
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
  pinMode(PIN_HOME_BUTTON, INPUT_PULLUP);

  enableDrivers();

  Serial.println();
  Serial.println("=== Test on dinh A4988 + dong co buoc (co gioi han mem truc Z) ===");
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
  printHelp();
  if (!AUTO_START) Serial.println("Dang TAM DUNG - nhan HOME de bat dau.");
  delay(2000);
}

void loop() {
  pollButton();
  pollSerial();
  if (state != ST_RUNNING) {
    delay(5);
    return;
  }

  // Chu ky truoc bi dung giua chung -> dua Z ve vi tri dau chu ky.
  if (zPosSteps != zCycleBase) {
    Serial.println("Dua Z ve vi tri dau chu ky...");
    long d = zCycleBase - zPosSteps;
    if (!moveZSteps(d > 0 ? d : -d, d > 0)) return;
  }

  cycleCount++;
  Serial.println();
  Serial.print("===== CHU KY #");
  Serial.print(cycleCount);
  Serial.println(" =====");

  Serial.println("--- BAN XOAY ---");
  if (!spinTable(true, TEST_REVS))  return;
  if (!waitMs(500))                 return;
  if (!spinTable(false, TEST_REVS)) return;
  Serial.println("TABLE xong -- kiem tra diem danh dau da ve DUNG vi tri ban dau chua.");

  if (!waitMs(1000)) return;

  Serial.println("--- TRUC Z ---");
  if (!spinZ(true, TEST_REVS))  return;
  if (!waitMs(1000))            return;
  if (!spinZ(false, TEST_REVS)) return;
  Serial.println("Z xong -- kiem tra diem danh dau da ve DUNG vi tri ban dau chua.");

  Serial.print("Chu ky #");
  Serial.print(cycleCount);
  Serial.println(" hoan tat. Nghi truoc khi lap lai...");
  waitMs(PAUSE_BETWEEN_CYCLES_MS);
}
