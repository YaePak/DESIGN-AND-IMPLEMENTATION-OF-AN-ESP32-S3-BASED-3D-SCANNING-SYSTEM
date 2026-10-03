#include <Arduino.h>
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

const char* AP_SSID     = "ScannerESP32";
const char* AP_PASSWORD = "scanner123";
IPAddress serverIP(192, 168, 4, 2);
const uint16_t serverPort = 5000;
WiFiClient client;

const int PIN_TABLE_DIR  = 4;
const int PIN_TABLE_STEP = 5;
const int PIN_TABLE_EN   = 6;

const int PIN_Z_DIR  = 12;
const int PIN_Z_STEP = 11;
const int PIN_Z_EN   = 10;

const int PIN_TFLUNA_RX = 17;
const int PIN_TFLUNA_TX = 18;

const int PIN_HOME_BUTTON = 7;

HardwareSerial TFLunaSerial(1);
const uint32_t TFLUNA_BAUD = 115200;
const uint32_t TFLUNA_READ_TIMEOUT_MS = 50;

const bool AUTO_CALIBRATE_STRENGTH     = true;
const int  TFLUNA_DEFAULT_MIN_STRENGTH = 1000;
const int  TFLUNA_STRENGTH_FLOOR       = 100;
int g_minValidStrength = TFLUNA_DEFAULT_MIN_STRENGTH;

const int   MOTOR_STEPS_PER_REV = 200;
const int   MICROSTEPPING       = 16;
const int   TOTAL_STEPS_PER_REV = MOTOR_STEPS_PER_REV * MICROSTEPPING;
const float LEAD_SCREW_PITCH_MM = 8.0;

const int   STEPS_PER_REV    = 80;
const float D_ANGLE          = 2.0 * PI / STEPS_PER_REV;
const int   STEPS_PER_SAMPLE = TOTAL_STEPS_PER_REV / STEPS_PER_REV;

const float Z_LAYER_MM = 1.0;

const float Z_TRAVEL_LIMIT_MM = 200.0;

const int   Z_STEPS_PER_LAYER = (int)((Z_LAYER_MM / LEAD_SCREW_PITCH_MM) * TOTAL_STEPS_PER_REV);

const float Z_MIN_SCAN_MM        = 10.0;
const int   EMPTY_LAYERS_TO_STOP = 2;

const bool RETURN_Z_AFTER_SCAN = true;

const float DISTANCE_TO_CENTER_CM = 28.0;
const float MAX_RADIUS_MM         = 150.0;

const int STEP_DELAY_US = 600;

const int TABLE_DIR_LEVEL = HIGH;
const int Z_UP_LEVEL      = HIGH;

const int YIELD_EVERY_STEPS = 200;

const int   CAL_TABLE_STEP_DEG     = 45;
const float CAL_Z_STEP_MM          = 10.0;
const int   CAL_READS_PER_STOP     = 5;
const int   CAL_EMPTY_STOPS_TO_END = 2;
const int   CAL_MIN_OBJ_SAMPLES    = 5;
const int   CAL_MIN_BG_SAMPLES     = 3;
const int   CAL_MAX_SAMPLES        = 200;
const int   CAL_TABLE_STEPS = TOTAL_STEPS_PER_REV * CAL_TABLE_STEP_DEG / 360;
const long  CAL_Z_STEPS     = (long)((CAL_Z_STEP_MM / LEAD_SCREW_PITCH_MM) * TOTAL_STEPS_PER_REV);

const BaseType_t CORE_REALTIME = 1;
const BaseType_t CORE_COMM     = 0;

const uint32_t BUTTON_DEBOUNCE_MS = 30;
const uint32_t BUTTON_LONG_MS     = 2000;

const bool DISABLE_DRIVERS_ON_FAULT = false;

long  mmToSteps(float mm)   { return (long)(mm / LEAD_SCREW_PITCH_MM * TOTAL_STEPS_PER_REV + 0.5f); }
float stepsToMm(long steps) { return steps * LEAD_SCREW_PITCH_MM / TOTAL_STEPS_PER_REV; }

const long Z_MAX_STEPS = mmToSteps(Z_TRAVEL_LIMIT_MM);

enum FaultCode {
  FAULT_NONE = 0,
  FAULT_SOFT_MAX,      // lenh muon dua Z vuot Z_TRAVEL_LIMIT_MM
  FAULT_USER_STOP      // nhan nut HOME / go "stop" khi dang chay
};

volatile FaultCode g_fault          = FAULT_NONE;
volatile bool      g_abortRequested = false;   // core 0 bat, core 1 doc moi xung
volatile long      g_zPosSteps      = 0;       // vi tri Z hien tai (xung), 0 = day
volatile bool      g_zHomed         = false;   // true khi da co z=0
volatile bool      g_busy           = false;   // motorTask dang chay lenh

const char* faultName(FaultCode f) {
  switch (f) {
    case FAULT_NONE:      return "KHONG LOI";
    case FAULT_SOFT_MAX:  return "VUOT GIOI HAN TRUC Z (Z_TRAVEL_LIMIT_MM)";
    case FAULT_USER_STOP: return "DA DUNG (nut HOME / lenh stop)";
  }
  return "?";
}

enum CmdType { CMD_START, CMD_UP, CMD_DOWN, CMD_ZERO, CMD_CLEAR, CMD_OFF };
struct MotorCmd {
  CmdType type;
  float   arg;
};
QueueHandle_t cmdQueue = NULL;

struct RawSample {
  float radiusMm;
  float angleRad;
  float zMm;
  bool  endOfScan;
  bool  aborted;     // true = quet bi dung giua chung
};
const int QUEUE_LENGTH = 256;
QueueHandle_t rawQueue = NULL;

void connectToServer() {
  Serial.println("Waiting for PC server...");
  while (!client.connect(serverIP, serverPort)) {
    Serial.println("  connect failed, retrying in 1s...");
    delay(1000);
  }
  Serial.println("Connected to PC.");
}

void tfLunaInit() {
  TFLunaSerial.begin(TFLUNA_BAUD, SERIAL_8N1, PIN_TFLUNA_RX, PIN_TFLUNA_TX);
  delay(100);
}

bool readTFLunaRaw(int& distanceCm, int& strength) {

  while (TFLunaSerial.available()) TFLunaSerial.read();

  uint8_t buf[9];
  int idx = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < TFLUNA_READ_TIMEOUT_MS) {
    if (!TFLunaSerial.available()) {
      delay(1);
      continue;
    }
    uint8_t b = TFLunaSerial.read();

    if (idx < 2) {
      if (b == 0x59) buf[idx++] = b;
      else idx = 0;
    } else {
      buf[idx++] = b;
      if (idx == 9) {
        uint8_t sum = 0;
        for (int i = 0; i < 8; i++) sum += buf[i];
        if (sum != buf[8]) { idx = 0; continue; }

        distanceCm = buf[2] | (buf[3] << 8);
        strength   = buf[4] | (buf[5] << 8);
        return true;
      }
    }
  }
  return false;
}

float readDistanceCM() {
  int dist, strength;
  if (!readTFLunaRaw(dist, strength)) return -1.0;
  if (strength < g_minValidStrength) return -1.0;
  return (float)dist;
}

void enableDrivers() {
  digitalWrite(PIN_TABLE_EN, LOW);
  digitalWrite(PIN_Z_EN, LOW);
}

void disableDrivers() {
  digitalWrite(PIN_TABLE_EN, HIGH);
  digitalWrite(PIN_Z_EN, HIGH);
}

void raiseFault(FaultCode code, const char* detail) {
  if (g_fault == FAULT_NONE) g_fault = code;   // giu lai loi DAU TIEN
  Serial.println();
  Serial.printf("!!! %s  (%s)  z=%.2f mm\n", faultName(code), detail, stepsToMm(g_zPosSteps));
  Serial.println("!!! May dung yen tai cho. Nhan HOME de quet lai, giu HOME 2s de nha motor.");
  if (DISABLE_DRIVERS_ON_FAULT) disableDrivers();
}

void stepPulse(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY_US);
}

bool rotateTableSteps(long steps) {
  digitalWrite(PIN_TABLE_DIR, TABLE_DIR_LEVEL);
  for (long i = 0; i < steps; i++) {
    if (g_abortRequested) {
      raiseFault(FAULT_USER_STOP, "khi dang quay ban xoay");
      return false;
    }
    stepPulse(PIN_TABLE_STEP);
    if ((i + 1) % YIELD_EVERY_STEPS == 0) delay(1);
  }
  return true;
}

bool moveZSteps(long steps, bool up) {
  if (steps <= 0) return true;
  enableDrivers();
  const int downLevel = (Z_UP_LEVEL == HIGH) ? LOW : HIGH;
  digitalWrite(PIN_Z_DIR, up ? Z_UP_LEVEL : downLevel);
  delayMicroseconds(5);   // A4988: DIR on dinh truoc canh len STEP

  for (long i = 0; i < steps; i++) {
    if (g_abortRequested) {
      raiseFault(FAULT_USER_STOP, "khi dang chay truc Z");
      return false;
    }
    if (g_zHomed) {
      if (up && g_zPosSteps >= Z_MAX_STEPS) {
        raiseFault(FAULT_SOFT_MAX, "lenh muon di qua dinh truc");
        return false;
      }
      if (!up && g_zPosSteps <= 0) {
        Serial.println("[z] Da o z=0 (day truc), khong ha them.");
        return true;
      }
    }
    stepPulse(PIN_Z_STEP);
    g_zPosSteps = g_zPosSteps + (up ? 1 : -1);
    if ((i + 1) % YIELD_EVERY_STEPS == 0) delay(1);
  }
  return true;
}

bool moveZToSteps(long target) {
  long delta = target - g_zPosSteps;
  if (delta > 0) return moveZSteps(delta, true);
  if (delta < 0) return moveZSteps(-delta, false);
  return true;
}

bool stepTurntable() { return rotateTableSteps(STEPS_PER_SAMPLE); }
bool stepZAxis()     { return moveZSteps(Z_STEPS_PER_LAYER, true); }

void printStatus() {
  Serial.println("----- TRANG THAI -----");
  Serial.printf("Trang thai: %s\n", g_busy ? "DANG CHAY" : "dung yen");
  Serial.printf("Loi       : %s\n", faultName(g_fault));
  Serial.printf("Vi tri Z  : %.2f mm (%ld xung)  %s\n", stepsToMm(g_zPosSteps), (long)g_zPosSteps,
                g_zHomed ? "[da co z=0]" : "[CHUA co z=0 - nhan HOME khi Z o thap nhat]");
  Serial.printf("Gioi han  : 0 .. %.1f mm\n", Z_TRAVEL_LIMIT_MM);
  Serial.printf("Strength  : %d\n", g_minValidStrength);
  Serial.println("----------------------");
}

void printHelp() {
  Serial.println("----- NUT HOME -----");
  Serial.println("  Nhan khi dung yen : bat dau quet");
  Serial.println("  Nhan khi dang chay: DUNG ngay, dung yen tai cho");
  Serial.println("  Giu 2 giay        : nha motor de chinh tay truc Z");
  Serial.println("----- LENH Serial Monitor (115200, Enter) -----");
  Serial.println("  stop       dung ngay (giong nhan HOME khi dang chay)");
  Serial.println("  scan       bat dau quet (giong nhan HOME khi dung yen)");
  Serial.println("  status     xem trang thai");
  Serial.println("  clear      xoa thong bao loi");
  Serial.println("  up <mm>    nang Z <mm>  (van bi gioi han mem chan)");
  Serial.println("  down <mm>  ha Z <mm>");
  Serial.println("  zero       coi vi tri hien tai la z=0");
  Serial.println("  off        nha motor (giong giu HOME 2 giay)");
  Serial.println("  help       in danh sach nay");
  Serial.println("-----------------------------------------------");
}

enum CalClass { CAL_UNUSABLE, CAL_OBJECT, CAL_BACKGROUND };

CalClass classifyReading(int distCm) {
  if (distCm <= 0) return CAL_BACKGROUND;
  float rMm = (DISTANCE_TO_CENTER_CM - distCm) * 10.0f;
  if (rMm <= 0) return CAL_BACKGROUND;
  if (rMm <= MAX_RADIUS_MM) return CAL_OBJECT;
  return CAL_UNUSABLE;
}

struct CalResult {
  bool ok;
  bool separated;
  int  objLow;
  int  bgHigh;
  int  threshold;
  bool aborted;
};

static void sortInts(int* a, int n) {
  for (int i = 1; i < n; i++) {
    int v = a[i], j = i - 1;
    while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
    a[j + 1] = v;
  }
}

static int percentileInt(const int* sorted, int n, float p) {
  return sorted[(int)(p * (n - 1))];
}

CalResult computeStrengthThreshold(int* obj, int nObj, int* bg, int nBg) {
  CalResult r = { false, false, 0, -1, TFLUNA_DEFAULT_MIN_STRENGTH, false };
  if (nObj < CAL_MIN_OBJ_SAMPLES) return r;

  sortInts(obj, nObj);
  r.objLow = percentileInt(obj, nObj, 0.10f);
  if (nBg >= CAL_MIN_BG_SAMPLES) {
    sortInts(bg, nBg);
    r.bgHigh = percentileInt(bg, nBg, 0.90f);
  }

  int thr;
  if (r.bgHigh >= 0 && r.bgHigh < r.objLow) {
    thr = (r.bgHigh + r.objLow) / 2;
    r.separated = true;
  } else {

    thr = r.objLow / 2;
  }
  if (thr < TFLUNA_STRENGTH_FLOOR) thr = TFLUNA_STRENGTH_FLOOR;
  r.threshold = thr;
  r.ok = true;
  return r;
}

static int calObj[CAL_MAX_SAMPLES];
static int calBg[CAL_MAX_SAMPLES];

CalResult calibrateStrength() {
  Serial.println("[cal] Bat dau do nguong strength: quay 45 do -> nang Z 1cm, lap lai...");
  int  nObj = 0, nBg = 0;
  int  emptyStops = 0;

  for (int stopIdx = 0; ; stopIdx++) {
    float zMm = stopIdx * CAL_Z_STEP_MM;

    int hits = 0;
    for (int k = 0; k < CAL_READS_PER_STOP; k++) {
      int dist, strength;
      if (!readTFLunaRaw(dist, strength)) continue;
      CalClass c = classifyReading(dist);
      if (c == CAL_OBJECT) {
        hits++;
        if (nObj < CAL_MAX_SAMPLES) calObj[nObj++] = strength;
      } else if (c == CAL_BACKGROUND) {
        if (nBg < CAL_MAX_SAMPLES) calBg[nBg++] = strength;
      }
    }
    Serial.printf("[cal] z=%.0fmm: %d/%d mau trung vat\n", zMm, hits, CAL_READS_PER_STOP);

    if (zMm >= Z_MIN_SCAN_MM) {
      if (hits == 0) {
        if (++emptyStops >= CAL_EMPTY_STOPS_TO_END) {
          Serial.printf("[cal] Da qua dinh vat tai z=%.0fmm.\n", zMm);
          break;
        }
      } else {
        emptyStops = 0;
      }
    }

    if ((stopIdx + 1) * CAL_Z_STEP_MM > Z_TRAVEL_LIMIT_MM) {
      Serial.println("[cal] Toi gioi han hanh trinh Z, dung do.");
      break;
    }

    if (!rotateTableSteps(CAL_TABLE_STEPS) || !moveZSteps(CAL_Z_STEPS, true)) {
      Serial.println("[cal] HUY hieu chinh. May dung yen, KHONG tu ha Z.");
      CalResult bad = { false, false, 0, -1, g_minValidStrength, true };
      return bad;
    }
  }

  CalResult r = computeStrengthThreshold(calObj, nObj, calBg, nBg);

  Serial.println();
  Serial.println("========== KET QUA HIEU CHINH STRENGTH ==========");
  Serial.printf("Mau trung vat: %d   Mau nen: %d\n", nObj, nBg);
  if (!r.ok) {
    Serial.printf("KHONG DU MAU TRUNG VAT (can >= %d). Dung nguong mac dinh %d.\n",
                  CAL_MIN_OBJ_SAMPLES, TFLUNA_DEFAULT_MIN_STRENGTH);
    Serial.println("  Kiem tra: vat co o giua ban xoay khong, DISTANCE_TO_CENTER_CM co dung khong.");
  } else {
    Serial.printf("Strength trung vat (muc thap): %d\n", r.objLow);
    if (r.bgHigh >= 0) Serial.printf("Strength nen     (muc cao) : %d\n", r.bgHigh);
    else               Serial.println("Strength nen: khong du mau (vat cao hon hanh trinh do?)");
    if (r.separated) Serial.println("Vat va nen tach biet ro -> nguong = diem giua.");
    else             Serial.println("Vat va nen KHONG tach biet ro -> nguong = 1/2 muc thap cua vat.");
    Serial.printf(">>> TFLUNA_MIN_VALID_STRENGTH = %d\n", r.threshold);
  }
  Serial.println("=================================================");
  g_minValidStrength = r.threshold;

  Serial.printf("[cal] Dua truc Z ve z=0 (dang o %.1f mm)...\n", stepsToMm(g_zPosSteps));
  if (!moveZToSteps(0)) {
    r.aborted = true;
    return r;
  }
  Serial.println("[cal] Da ve z=0.");
  return r;
}

bool runScan() {
  Serial.println("[core1] Bat dau quet toa do tu z=0.");
  uint32_t startMs = millis();
  int  emptyLayers = 0;
  bool aborted     = false;

  for (int layer = 0; !aborted; layer++) {
    float z = layer * Z_LAYER_MM;

    int validThisLayer = 0;
    float angle = 0;
    for (int i = 0; i < STEPS_PER_REV; i++) {
      float d = readDistanceCM();

      if (d > 0) {
        float rMm = (DISTANCE_TO_CENTER_CM - d) * 10.0f;
        if (rMm > 0 && rMm <= MAX_RADIUS_MM) {
          validThisLayer++;
          RawSample s = { rMm, angle, z, false, false };
          xQueueSend(rawQueue, &s, pdMS_TO_TICKS(5000));
        }
      }

      angle += D_ANGLE;
      if (!stepTurntable()) { aborted = true; break; }
    }
    if (aborted) break;

    if (z >= Z_MIN_SCAN_MM) {
      if (validThisLayer == 0) {
        if (++emptyLayers >= EMPTY_LAYERS_TO_STOP) {
          Serial.printf("[core1] Top of object reached at z=%.0fmm. Stopping.\n", z);
          break;
        }
      } else {
        emptyLayers = 0;
      }
    }

    if ((layer + 1) * Z_LAYER_MM > Z_TRAVEL_LIMIT_MM) {
      Serial.printf("[core1] Reached travel limit (%.0fmm). Stopping.\n", Z_TRAVEL_LIMIT_MM);
      break;
    }

    if (!stepZAxis()) { aborted = true; break; }

    if ((layer + 1) % 10 == 0) {
      Serial.printf("[core1] layer %d done (z=%.0fmm), %lu s elapsed\n",
                    layer + 1, z, (millis() - startMs) / 1000);
    }
  }

  RawSample endMarker = { 0, 0, 0, true, aborted };
  xQueueSend(rawQueue, &endMarker, pdMS_TO_TICKS(2000));

  if (aborted) {
    Serial.println("[core1] Quet bi DUNG. May dung yen tai cho, cho lenh tiep theo.");
    return false;
  }

  Serial.println("[core1] Motion complete.");
  if (RETURN_Z_AFTER_SCAN && g_zPosSteps > 0) {
    Serial.printf("[core1] Dang ha truc Z ve z=0 (dang o %.1f mm)...\n", stepsToMm(g_zPosSteps));
    if (!moveZToSteps(0)) return false;
    Serial.println("[core1] Da ve z=0. Nhan HOME de quet lan tiep theo.");
  }
  disableDrivers();
  return true;
}

void startScan() {
  if (g_fault != FAULT_NONE) {
    Serial.printf("Xoa trang thai '%s'.\n", faultName(g_fault));
    g_fault = FAULT_NONE;
  }
  enableDrivers();

  if (!g_zHomed) {
    g_zPosSteps = 0;
    g_zHomed    = true;
    Serial.println("Lay vi tri hien tai lam z=0 (truc Z phai dang o THAP NHAT).");
  } else if (g_zPosSteps != 0) {
    Serial.printf("Dua Z ve z=0 truoc khi quet (dang o %.1f mm)...\n", stepsToMm(g_zPosSteps));
    if (!moveZToSteps(0)) return;
  }

  if (AUTO_CALIBRATE_STRENGTH) {
    CalResult c = calibrateStrength();
    if (c.aborted) return;
  } else {
    Serial.printf("[cal] Bo qua hieu chinh, dung nguong mac dinh %d.\n", g_minValidStrength);
  }
  runScan();
}

void handleCommand(const MotorCmd& c) {
  switch (c.type) {
    case CMD_START:
      startScan();
      return;

    case CMD_UP:
    case CMD_DOWN: {
      float mm = c.arg;
      if (mm <= 0) { Serial.println("Can so mm > 0, vd: up 5"); return; }
      if (mm > Z_TRAVEL_LIMIT_MM) mm = Z_TRAVEL_LIMIT_MM;
      if (!g_zHomed) Serial.println("[z] Chu y: chua co z=0 -> gioi han mem dang TAT, chi di dung so mm da go.");
      bool up = (c.type == CMD_UP);
      Serial.printf("[z] %s %.2f mm...\n", up ? "Nang" : "Ha", mm);
      if (moveZSteps(mmToSteps(mm), up))
        Serial.printf("[z] Xong. z=%.2f mm\n", stepsToMm(g_zPosSteps));
      return;
    }

    case CMD_ZERO:
      g_zPosSteps = 0;
      g_zHomed    = true;
      Serial.println("Da dat vi tri hien tai la z=0.");
      return;

    case CMD_CLEAR:
      g_fault = FAULT_NONE;
      Serial.println("Da xoa thong bao loi.");
      printStatus();
      return;

    case CMD_OFF:
      disableDrivers();
      g_zHomed = false;
      Serial.println("Da NHA motor. Chinh truc Z bang tay xuong THAP NHAT, roi nhan HOME de quet (vi tri do = z=0).");
      return;
  }
}

void motorTask(void* pv) {
  Serial.println("Dat vat len GIUA ban xoay, ha truc Z xuong THAP NHAT (ngang mat ban xoay),");
  Serial.println("roi nhan nut HOME (GPIO7). May se do nguong strength, tu ve z=0, roi moi quet.");
  printHelp();

  for (;;) {
    MotorCmd cmd;
    if (xQueueReceive(cmdQueue, &cmd, portMAX_DELAY) != pdTRUE) continue;
    g_abortRequested = false;   // lenh dung cu khong anh huong lenh moi
    g_busy = true;
    handleCommand(cmd);
    g_busy = false;
  }
}

void queueCmd(CmdType t, float arg = 0) {
  MotorCmd c;
  c.type = t;
  c.arg  = arg;
  if (g_busy) Serial.println(">> May dang chay, lenh se chay sau khi xong (nhan HOME / go 'stop' de dung ngay).");
  if (xQueueSend(cmdQueue, &c, 0) != pdTRUE) Serial.println(">> Hang doi lenh day, bo qua.");
}

void pollHomeButton() {
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
    if (stable == LOW) {                       // vua nhan xuong
      tPress   = now;
      consumed = false;
      if (g_busy) {
        g_abortRequested = true;
        consumed = true;
        Serial.println(">> Nut HOME: DUNG MAY");
      }
    } else if (!consumed) {                    // nha ra sau lan nhan ngan
      Serial.println(">> Nut HOME: BAT DAU QUET");
      queueCmd(CMD_START);
    }
  }

  if (stable == LOW && !consumed && now - tPress >= BUTTON_LONG_MS) {
    consumed = true;
    Serial.println(">> Giu nut HOME 2s: NHA MOTOR");
    queueCmd(CMD_OFF);
  }
}

void processLine(char* line) {
  for (char* p = line; *p; p++) *p = tolower(*p);
  char word[16] = {0};
  float arg = 0;
  int n = sscanf(line, "%15s %f", word, &arg);
  if (n < 1) return;

  if (!strcmp(word, "stop") || !strcmp(word, "s")) {
    g_abortRequested = true;
    Serial.println(g_busy ? ">> STOP: dang dung may..." : ">> May dang dung yen.");
    return;
  }
  if (!strcmp(word, "status")) { printStatus(); return; }
  if (!strcmp(word, "help") || !strcmp(word, "?")) { printHelp(); return; }

  if      (!strcmp(word, "scan"))  queueCmd(CMD_START);
  else if (!strcmp(word, "up"))    queueCmd(CMD_UP, arg);
  else if (!strcmp(word, "down"))  queueCmd(CMD_DOWN, arg);
  else if (!strcmp(word, "zero"))  queueCmd(CMD_ZERO);
  else if (!strcmp(word, "clear")) queueCmd(CMD_CLEAR);
  else if (!strcmp(word, "off"))   queueCmd(CMD_OFF);
  else Serial.printf("Lenh khong hop le: '%s'. Go 'help'.\n", word);
}

void cmdTask(void* pv) {
  char line[48];
  size_t len = 0;
  for (;;) {
    pollHomeButton();
    while (Serial.available()) {
      char ch = (char)Serial.read();
      if (ch == '\r' || ch == '\n') {
        if (len > 0) {
          line[len] = 0;
          processLine(line);
          len = 0;
        }
      } else if (len < sizeof(line) - 1) {
        line[len++] = ch;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static char   txBuf[1460];
static size_t txLen = 0;

void flushTxBuffer() {
  if (txLen == 0) return;
  if (!client.connected()) {
    Serial.println("[core0] Connection lost, reconnecting...");
    connectToServer();
  }
  client.write((const uint8_t*)txBuf, txLen);
  txLen = 0;
}

void commTask(void* pv) {
  RawSample s;
  for (;;) {
    if (xQueueReceive(rawQueue, &s, portMAX_DELAY) != pdTRUE) continue;

    if (s.endOfScan) {
      flushTxBuffer();
      client.stop();
      Serial.println();
      if (s.aborted) {
        Serial.println("=============== SCAN STOPPED ===============");
        Serial.println("Quet bi dung giua chung. File tren PC chi co mot phan diem.");
      } else {
        Serial.println("=============== SCAN COMPLETE ===============");
        Serial.println("File complete on PC. Run xyz_to_stl.py to build the STL.");
      }
      Serial.println("Chay lai pcrecieve.py truoc lan quet tiep theo.");
      Serial.println("=============================================");
      continue;   // khong xoa task -> quet duoc nhieu lan
    }

    float x = s.radiusMm * cos(s.angleRad);
    float y = s.radiusMm * sin(s.angleRad);

    if (sizeof(txBuf) - txLen < 40) flushTxBuffer();
    int n = snprintf(txBuf + txLen, sizeof(txBuf) - txLen, "%.2f,%.2f,%.2f\n", x, y, s.zMm);
    if (n > 0) txLen += n;
  }
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
  pinMode(PIN_HOME_BUTTON, INPUT_PULLUP);

  tfLunaInit();
  int d0, s0;
  if (readTFLunaRaw(d0, s0)) {
    Serial.printf("TF-Luna OK: dist=%d cm, strength=%d\n", d0, s0);
  } else {
    Serial.println("WARNING: chua nhan duoc khung hop le tu TF-Luna. Kiem tra day TXD/RXD, nguon cap.");
  }

  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("AP started. ESP32-S3 IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("Connect laptop to 'ScannerESP32', start pc_receiver.py.");
  delay(5000);
  connectToServer();

  rawQueue = xQueueCreate(QUEUE_LENGTH, sizeof(RawSample));
  cmdQueue = xQueueCreate(8, sizeof(MotorCmd));
  if (rawQueue == NULL || cmdQueue == NULL) {
    Serial.println("ERROR: cannot create queue.");
    while (true) delay(1000);
  }

  xTaskCreatePinnedToCore(commTask,  "comm",  8192, NULL, 1, NULL, CORE_COMM);
  xTaskCreatePinnedToCore(cmdTask,   "cmd",   4096, NULL, 2, NULL, CORE_COMM);
  xTaskCreatePinnedToCore(motorTask, "motor", 6144, NULL, 3, NULL, CORE_REALTIME);

  Serial.println("Dual-core pipeline started.");
}

void loop() {
  delay(1000);
}
