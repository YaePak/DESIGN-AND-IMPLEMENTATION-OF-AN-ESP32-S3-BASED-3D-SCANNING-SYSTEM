#include <Arduino.h>
#include <WiFi.h>
#include <esp_system.h>
#include <atomic>

const char*    AP_SSID      = "ScannerESP32";
const char*    AP_PASS      = "scanner123";
const uint16_t PC_PORT      = 5000;
const uint32_t HEARTBEAT_MS = 2000;
const uint32_t ACK_WAIT_MS  = 3000;

const int PIN_TABLE_DIR = 4, PIN_TABLE_STEP = 5, PIN_TABLE_EN = 6;
const int PIN_Z_DIR = 10, PIN_Z_STEP = 11, PIN_Z_EN = 12;
const int PIN_IR = 18, PIN_IR_UNUSED = 17;

constexpr float MOTOR_STEP_DEG = 1.8f;
constexpr int   MICROSTEP      = 1;
constexpr int   STEPS_PER_REV  = (int)(360.0f / MOTOR_STEP_DEG + 0.5f) * MICROSTEP;
constexpr float LEAD_SCREW_MM  = 8.0f;
constexpr float Z_STEPS_PER_MM = STEPS_PER_REV / LEAD_SCREW_MM;

constexpr float POINT_ANGLE_DEG   = 3.6f;
constexpr int   STEPS_PER_POINT   = (int)(POINT_ANGLE_DEG / MOTOR_STEP_DEG * MICROSTEP + 0.5f);
constexpr int   POINTS_PER_REV    = STEPS_PER_REV / STEPS_PER_POINT;
constexpr float LAYER_MM          = 1.0f;
constexpr float Z_TRAVEL_LIMIT_MM = 150.0f;
constexpr int   MAX_LAYERS        = (int)(Z_TRAVEL_LIMIT_MM / LAYER_MM) + 1;
static_assert(STEPS_PER_POINT >= 1 && STEPS_PER_REV % STEPS_PER_POINT == 0,
              "POINT_ANGLE_DEG phai la boi cua 1.8 va chia het 360");

const float DIST_TO_CENTER_MM = 90.0f;
const float MIN_RADIUS_MM     = 1.0f;

constexpr int CAL_MV[] = {1495, 1374, 1290, 1224, 1169, 1081, 945, 837,
                          803,  769,  738,  708,  681, 654, 629};
constexpr int CAL_MM[] = {30,   35,   40,   45,   50,  55,  60,  65,
                          70,   75,   80,   85,   90,  95,  100};

constexpr int CAL_POINTS = sizeof(CAL_MV) / sizeof(CAL_MV[0]);
static_assert(sizeof(CAL_MV) == sizeof(CAL_MM) && CAL_POINTS >= 2, "CAL_MV va CAL_MM phai cung so moc (>= 2)");
constexpr bool calOrdered(int i = 1) {
  return i >= CAL_POINTS || (CAL_MV[i] < CAL_MV[i - 1] && CAL_MM[i] > CAL_MM[i - 1] && calOrdered(i + 1));
}
static_assert(calOrdered(), "CAL_MV phai giam dan, CAL_MM phai tang dan (do lai moc bi trung hoac dao)");
const float MAX_RADIUS_MM = DIST_TO_CENTER_MM - CAL_MM[0];

const uint32_t SETTLE_MS        = 30;
const int      SAMPLES          = 24;
const uint32_t SAMPLE_GAP_US    = 1000;
const int      MIN_GOOD_SAMPLES = 10;
const int      BAD_READ_MV      = 100;
const int      SPREAD_MAX_MV    = 150;
const uint32_t WARMUP_MS        = 1000;

const int      EMPTY_LAYERS_TO_STOP = 1;
const float    EMPTY_RATIO          = 0.02f;
const int      NO_OBJECT_LAYERS     = 5;
const float    OUTLIER_DIST_MM      = 5.0f;
const int      OUTLIER_NEIGHBOURS   = 2;
const uint32_t TX_WAIT_MS           = 2000;

const unsigned int TABLE_HALF_US    = 1500;
const uint32_t     TABLE_STEP_GAP_US = 22000;
const unsigned int Z_UP_HALF_US     = 1000;
const unsigned int Z_RETURN_HALF_US = 800;
const unsigned int START_HALF_US    = 2500;
const long         TABLE_RAMP_STEPS = 10;
const long         Z_RAMP_STEPS     = 100;
const long         SHORT_MOVE_STEPS = 4;
const uint32_t     Z_SETTLE_MS      = 30;

const int TABLE_DIR_LEVEL = HIGH;
const int Z_UP_LEVEL      = HIGH;

constexpr long Z_LIMIT_STEPS = (long)(Z_TRAVEL_LIMIT_MM * Z_STEPS_PER_MM + 0.5f);

long     zPos = 0;
uint32_t adcReads = 0, adcBad = 0;
uint16_t g_r[MAX_LAYERS][POINTS_PER_REV];
char     g_scanId[24] = "scan";

std::atomic<int>  g_layersDone{0};
std::atomic<int>  g_sentLayers{0};
std::atomic<bool> g_scanDone{false};
std::atomic<bool> g_pcConnected{false};

float mvToMm(float mv) {
  if (mv > CAL_MV[0] || mv < CAL_MV[CAL_POINTS - 1]) return -1;
  int i = 1;
  while (mv < CAL_MV[i]) i++;
  float t = (CAL_MV[i - 1] - mv) / float(CAL_MV[i - 1] - CAL_MV[i]);
  return CAL_MM[i - 1] + t * (CAL_MM[i] - CAL_MM[i - 1]);
}

bool irMeasure(float& dist, float& mv) {
  int v[SAMPLES], good = 0;
  mv = 0;
  for (int t = 0; t < SAMPLES * 3 && good < SAMPLES; t++) {
    int x = analogReadMilliVolts(PIN_IR);
    adcReads++;
    if (x >= BAD_READ_MV) {
      int j = good++;
      while (j > 0 && v[j - 1] > x) { v[j] = v[j - 1]; j--; }
      v[j] = x;
    } else {
      adcBad++;
    }
    delayMicroseconds(SAMPLE_GAP_US);
  }
  if (good < MIN_GOOD_SAMPLES) return false;
  if (v[good - 1 - good / 8] - v[good / 8] > SPREAD_MAX_MV) return false;
  int q = good / 4;
  long sum = 0;
  for (int i = q; i < good - q; i++) sum += v[i];
  mv   = sum / float(good - 2 * q);
  dist = mvToMm(mv);
  return dist >= 0;
}

float radiusFrom(float dist) {
  float r = DIST_TO_CENTER_MM - dist;
  return (r < MIN_RADIUS_MM || r > MAX_RADIUS_MM) ? -1 : r;
}

unsigned int rampHalfUs(long i, long n, unsigned int fastHalfUs, long ramp) {
  long r    = ramp < n / 2 ? ramp : n / 2;
  long edge = i < n - 1 - i ? i : n - 1 - i;
  if (fastHalfUs >= START_HALF_US || edge >= r) return fastHalfUs;
  return START_HALF_US - (START_HALF_US - fastHalfUs) * edge / r;
}

void moveSteps(int pin, long n, unsigned int fastHalfUs, long ramp) {
  if (n <= SHORT_MOVE_STEPS) ramp = 0;
  for (long i = 0; i < n; i++) {
    unsigned int h = rampHalfUs(i, n, fastHalfUs, ramp);
    digitalWrite(pin, HIGH);
    delayMicroseconds(h);
    digitalWrite(pin, LOW);
    delayMicroseconds(h);
    if ((i & 0xFF) == 0xFF) vTaskDelay(1);
  }
}

void rotateTable(long steps) {
  digitalWrite(PIN_TABLE_DIR, TABLE_DIR_LEVEL);
  delayMicroseconds(5);
  if (MICROSTEP > 1 || steps > SHORT_MOVE_STEPS) {
    moveSteps(PIN_TABLE_STEP, steps, TABLE_HALF_US, TABLE_RAMP_STEPS);
    return;
  }
  for (long i = 0; i < steps; i++) {
    if (i) delayMicroseconds(TABLE_STEP_GAP_US);
    digitalWrite(PIN_TABLE_STEP, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TABLE_STEP, LOW);
  }
}

void moveZ(long steps, unsigned int fastHalfUs) {
  long target = constrain(zPos + steps, 0L, Z_LIMIT_STEPS);
  long n = labs(target - zPos);
  if (n == 0) return;
  int dir = target > zPos ? 1 : -1;
  digitalWrite(PIN_Z_DIR, dir > 0 ? Z_UP_LEVEL : (Z_UP_LEVEL == HIGH ? LOW : HIGH));
  delay(10);
  moveSteps(PIN_Z_STEP, n, fastHalfUs, Z_RAMP_STEPS);
  zPos = target;
}

float medianSmall(float* v, int n) {
  for (int i = 1; i < n; i++) {
    float x = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
  return (n % 2) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

void rejectOutliers(float* r) {
  static bool reject[POINTS_PER_REV];
  for (int i = 0; i < POINTS_PER_REV; i++) {
    reject[i] = false;
    if (r[i] < 0) continue;
    float nb[2 * OUTLIER_NEIGHBOURS];
    int n = 0;
    for (int k = -OUTLIER_NEIGHBOURS; k <= OUTLIER_NEIGHBOURS; k++) {
      int j = (i + k + POINTS_PER_REV) % POINTS_PER_REV;
      if (k != 0 && r[j] >= 0) nb[n++] = r[j];
    }
    reject[i] = (n == 0) || (n >= 2 && fabsf(r[i] - medianSmall(nb, n)) > OUTLIER_DIST_MM);
  }
  for (int i = 0; i < POINTS_PER_REV; i++) if (reject[i]) r[i] = -1;
}

void waitTxIdle() {
  uint32_t t0 = millis();
  while (g_pcConnected && g_sentLayers.load() < g_layersDone.load() && millis() - t0 < TX_WAIT_MS)
    vTaskDelay(pdMS_TO_TICKS(5));
}

int runScan(long& total) {
  long layerSteps = (long)(LAYER_MM * Z_STEPS_PER_MM + 0.5f);
  int  emptyMin   = max(1, (int)(EMPTY_RATIO * POINTS_PER_REV + 0.5f));
  int  empty      = 0;
  bool seen       = false;
  static float r[POINTS_PER_REV], vo[POINTS_PER_REV];
  total = 0;

  for (int layer = 0; layer < MAX_LAYERS; layer++) {
    uint32_t t0 = millis(), reads0 = adcReads, bad0 = adcBad;
    for (int p = 0; p < POINTS_PER_REV; p++) {
      vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));
      float d, mv;
      r[p] = irMeasure(d, mv) ? radiusFrom(d) : -1;
      vo[p] = mv;
      rotateTable(STEPS_PER_POINT);
    }
    rejectOutliers(r);
    int valid = 0;
    static float rv[POINTS_PER_REV], vv[POINTS_PER_REV];
    for (int p = 0; p < POINTS_PER_REV; p++) {
      g_r[layer][p] = r[p] > 0 ? (uint16_t)lroundf(r[p] * 100) : 0;
      if (r[p] > 0) { rv[valid] = r[p]; vv[valid] = vo[p]; valid++; }
    }
    total += valid;
    g_layersDone.store(layer + 1);
    uint32_t reads = adcReads - reads0;
    Serial.printf("# z=%5.1f mm | %3d/%d diem | Vo tv %4.0f mV | r tv %5.1f mm | ADC loi %4.1f%% | %.1f s\n",
                  layer * LAYER_MM, valid, POINTS_PER_REV, valid ? medianSmall(vv, valid) : 0.0f,
                  valid ? medianSmall(rv, valid) : 0.0f, reads ? 100.0f * (adcBad - bad0) / reads : 0.0f,
                  (millis() - t0) / 1000.0f);

    if (valid >= emptyMin) { seen = true; empty = 0; }
    else if (seen)         empty++;
    if (seen && empty >= EMPTY_LAYERS_TO_STOP) return layer + 1;
    if (!seen && layer + 1 >= NO_OBJECT_LAYERS) return 0;
    if (zPos + layerSteps > Z_LIMIT_STEPS) return layer + 1;
    moveZ(layerSteps, Z_UP_HALF_US);
    waitTxIdle();
    vTaskDelay(pdMS_TO_TICKS(Z_SETTLE_MS));
  }
  return MAX_LAYERS;
}

void motionTask(void*) {
  float d = -1, mv = 0;
  bool ok = irMeasure(d, mv);
  Serial.printf("# Tu kiem tra goc 0: Vo=%.0f mV -> d=%.1f mm -> r=%.1f mm%s\n", mv, ok ? d : -1.0f,
                ok ? DIST_TO_CENTER_MM - d : -1.0f, ok ? "" : " (ngoai bang hieu chuan / khong co vat)");
  uint32_t t0 = millis();
  long total = 0;
  int layers = runScan(total);
  moveZ(-zPos, Z_RETURN_HALF_US);
  digitalWrite(PIN_TABLE_EN, HIGH);
  if (layers == 0)
    Serial.printf("# KHONG THAY VAT trong %d lop dau (r = %.0f..%.0f mm). Z da ve goc.\n",
                  NO_OBJECT_LAYERS, MIN_RADIUS_MM, MAX_RADIUS_MM);
  else
    Serial.printf("# XONG: %d lop, %ld diem, %.0f s (%.1f phut). Z da ve goc.\n",
                  layers, total, (millis() - t0) / 1000.0f, (millis() - t0) / 60000.0f);
  g_scanDone = true;
  vTaskDelete(nullptr);
}

WiFiServer server(PC_PORT);
WiFiClient client;
static char txBuf[1400];
static size_t txLen = 0;
int      sentLayers = 0;
long     sentPoints = 0;
bool     started = false, waitingAck = false, finished = false;
uint32_t lastSend = 0, ackDeadline = 0;
char     ackLine[16];
int      ackLen = 0;

void resetConn() {
  started = waitingAck = false;
  sentLayers = 0;
  sentPoints = 0;
  txLen = 0;
  ackLen = 0;
  g_sentLayers = 0;
}

void dropPc(const char* why) {
  if (g_pcConnected) Serial.printf("# PC ngat ket noi (%s)\n", why);
  client.stop();
  g_pcConnected = false;
  resetConn();
}

bool flushTx() {
  if (txLen == 0) return true;
  bool ok = client.write((const uint8_t*)txBuf, txLen) == txLen;
  txLen = 0;
  lastSend = millis();
  return ok;
}

bool putLine(const char* s, int n) {
  if (txLen + n > sizeof(txBuf) && !flushTx()) return false;
  memcpy(txBuf + txLen, s, n);
  txLen += n;
  return true;
}

bool sendLayer(int layer) {
  char line[48];
  for (int p = 0; p < POINTS_PER_REV; p++) {
    if (!g_r[layer][p]) continue;
    float r = g_r[layer][p] * 0.01f;
    float a = p * (360.0f / POINTS_PER_REV) * (float)DEG_TO_RAD;
    int n = snprintf(line, sizeof(line), "%.2f,%.2f,%.2f\n", r * cosf(a), r * sinf(a), layer * LAYER_MM);
    if (!putLine(line, n)) return false;
    sentPoints++;
  }
  return flushTx();
}

void commSetup() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  server.begin();
  server.setNoDelay(true);
  Serial.printf("# WiFi %s | PC chay pcrecieve.py ket noi toi %s:%u\n",
                AP_SSID, WiFi.softAPIP().toString().c_str(), PC_PORT);
}

void commStep() {
  if (finished) return;
  if (server.hasClient()) {
    WiFiClient incoming = server.accept();
    if (g_pcConnected) dropPc("co ket noi moi");
    client = incoming;
    client.setNoDelay(true);
    resetConn();
    lastSend = millis();
    g_pcConnected = true;
    Serial.printf("# PC ket noi tu %s\n", client.remoteIP().toString().c_str());
  }
  if (!client.connected() || WiFi.softAPgetStationNum() == 0) {
    if (g_pcConnected) dropPc("mat ket noi");
    return;
  }
  if (waitingAck) {
    while (client.available()) {
      int c = client.read();
      if (c != '\n') { if (ackLen < (int)sizeof(ackLine) - 1) ackLine[ackLen++] = (char)c; continue; }
      ackLine[ackLen] = 0;
      if (strncmp(ackLine, "ACK", 3) == 0) {
        Serial.printf("# PC da nhan du %ld diem (ACK). Tat/bat nguon de quet lan moi.\n", sentPoints);
        finished = true;
        client.stop();
        server.end();
        g_pcConnected = false;
      } else {
        dropPc("PC bao thieu diem");
      }
      return;
    }
    if ((int32_t)(millis() - ackDeadline) > 0) dropPc("khong nhan duoc ACK");
    return;
  }
  if (!started) {
    char line[64];
    int n = snprintf(line, sizeof(line), "START,%s,%d,%g\n", g_scanId, POINTS_PER_REV, LAYER_MM);
    if (!putLine(line, n) || !flushTx()) { dropPc("loi gui"); return; }
    started = true;
  }
  if (sentLayers < g_layersDone.load()) {
    if (!sendLayer(sentLayers)) { dropPc("loi gui"); return; }
    g_sentLayers = ++sentLayers;
    return;
  }
  if (g_scanDone) {
    char line[64];
    int n = snprintf(line, sizeof(line), "END,%s,%ld\n", g_scanId, sentPoints);
    if (!putLine(line, n) || !flushTx()) { dropPc("loi gui"); return; }
    waitingAck  = true;
    ackDeadline = millis() + ACK_WAIT_MS;
    return;
  }
  if (millis() - lastSend > HEARTBEAT_MS && (!putLine("\n", 1) || !flushTx())) dropPc("loi gui");
}

void commTask(void*) {
  commSetup();
  for (;;) {
    commStep();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);
  const int outputs[] = {PIN_TABLE_DIR, PIN_TABLE_STEP, PIN_TABLE_EN, PIN_Z_DIR, PIN_Z_STEP, PIN_Z_EN};
  for (int pin : outputs) pinMode(pin, OUTPUT);
  digitalWrite(PIN_TABLE_STEP, LOW);
  digitalWrite(PIN_Z_STEP, LOW);
  digitalWrite(PIN_TABLE_EN, LOW);
  digitalWrite(PIN_Z_EN, LOW);
  pinMode(PIN_IR_UNUSED, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_IR, ADC_11db);
  snprintf(g_scanId, sizeof(g_scanId), "scan%lu", (unsigned long)micros());
  delay(WARMUP_MS);
  esp_reset_reason_t why = esp_reset_reason();
  Serial.printf("# Khoi dong (ly do reset = %d%s). Driver da giu motor %lu ms.\n", (int)why,
                why == ESP_RST_BROWNOUT ? ": SUT AP - kiem tra nguon" : "", (unsigned long)WARMUP_MS);
  Serial.printf("# GP2Y0A51SK0F GPIO%d | D=%.0f mm | r=%.0f..%.0f mm | %d diem/vong | lop %.1f mm\n",
                PIN_IR, DIST_TO_CENTER_MM, MIN_RADIUS_MM, MAX_RADIUS_MM, POINTS_PER_REV, LAYER_MM);
  xTaskCreatePinnedToCore(commTask,   "comm",   6144, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(motionTask, "motion", 8192, nullptr, 2, nullptr, 1);
}

void loop() {
  vTaskDelete(nullptr);
}