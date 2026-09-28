#include <Arduino.h>
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ================= WiFi Access Point =================
const char* AP_SSID     = "ScannerESP32";
const char* AP_PASSWORD = "scanner123";
IPAddress serverIP(192, 168, 4, 2);
const uint16_t serverPort = 5000;
WiFiClient client;

// ================= Bảng chân GPIO (theo sơ đồ mới) =================
// A4988 — Bàn xoay
const int PIN_TABLE_DIR  = 4;
const int PIN_TABLE_STEP = 5;
const int PIN_TABLE_EN   = 6;
// A4988 — Trục Z
const int PIN_Z_DIR  = 10;
const int PIN_Z_STEP = 11;
const int PIN_Z_EN   = 12;
// TF-Luna UART
const int PIN_TFLUNA_RX = 17;   
const int PIN_TFLUNA_TX = 18; 

const int PIN_HOME_BUTTON = 7;  // nối xuống GND khi nhấn (INPUT_PULLUP)

// ================= TF-Luna qua UART =================
HardwareSerial TFLunaSerial(1);           // dùng UART1 của ESP32-S3
const uint32_t TFLUNA_BAUD = 115200;      // tốc độ mặc định của TF-Luna
const uint32_t TFLUNA_READ_TIMEOUT_MS = 50;  // chờ tối đa 1 khung hợp lệ

// Ngưỡng strength giờ được TỰ HIỆU CHỈNH ở pha dò trước khi quét (xem hàm
// calibrateStrength). Giá trị mặc định chỉ dùng khi tắt tự hiệu chỉnh, hoặc khi
// hiệu chỉnh thất bại (không thu được đủ mẫu trúng vật).
const bool AUTO_CALIBRATE_STRENGTH     = true;
const int  TFLUNA_DEFAULT_MIN_STRENGTH = 1000;
const int  TFLUNA_STRENGTH_FLOOR       = 100;   // datasheet: dưới 100 là không tin cậy
int g_minValidStrength = TFLUNA_DEFAULT_MIN_STRENGTH;   // ngưỡng đang dùng khi quét

// ================= Động cơ & hình học quét =================
const int   MOTOR_STEPS_PER_REV = 200;   // NEMA17 1,8°/bước
const int   MICROSTEPPING       = 16;    // MS1/MS2/MS3 đều HIGH (1,2,4,8,16)
const int   TOTAL_STEPS_PER_REV = MOTOR_STEPS_PER_REV * MICROSTEPPING;
const float LEAD_SCREW_PITCH_MM = 8.0;

const int   STEPS_PER_REV    = 80;                                      // mẫu đo mỗi vòng (360/80=4,5°)
const float D_ANGLE          = 2.0 * PI / STEPS_PER_REV;
const int   STEPS_PER_SAMPLE = TOTAL_STEPS_PER_REV / STEPS_PER_REV;

const float Z_LAYER_MM = 1.0;
const float Z_TRAVEL_LIMIT_MM = 150.0;   // chặn an toàn cứng, KHÔNG phải chiều cao vật.
                                         // Đo hành trình thật của trục Z sau khi lắp khung, trừ vài mm biên.
const int   Z_STEPS_PER_LAYER = (int)((Z_LAYER_MM / LEAD_SCREW_PITCH_MM) * TOTAL_STEPS_PER_REV);

// Tự dừng khi quét qua đỉnh vật: lớp không thu được điểm nào = tia đã bắn qua
// đầu vật. Nhờ vậy vật cao bao nhiêu cũng quét được, không phí thời gian quét
// không khí phía trên.
const float Z_MIN_SCAN_MM        = 10.0; // quét ít nhất 10mm rồi mới cho phép tự dừng
const int   EMPTY_LAYERS_TO_STOP = 2;    // cần 2 lớp trống liên tiếp, tránh dừng nhầm ở khe hẹp

// Sau khi quét xong, tự hạ trục Z về z=0 để lần quét sau khỏi phải hạ bằng tay.
const bool RETURN_Z_AFTER_SCAN = true;

const float DISTANCE_TO_CENTER_CM = 28.0;
const float MAX_RADIUS_MM         = 150.0;   // xa hơn = tia bắn trượt ra ngoài vật

// ================= Núm chỉnh tốc độ quét =================
const int STEP_DELAY_US = 600;   // nhỏ -> nhanh hơn, motor có thể bị rung nếu quá nhỏ

// Chiều quay — đổi ở ĐÂY nếu động cơ chạy ngược. Trục Z giờ đi cả LÊN lẫn XUỐNG
// (pha hiệu chỉnh phải đưa Z về z=0), nên chiều chỉ được khai báo 1 chỗ này.
const int TABLE_DIR_LEVEL = HIGH;
const int Z_UP_LEVEL      = HIGH;   // mức DIR làm trục Z đi LÊN; đi xuống = mức ngược lại

// Nhường CPU sau mỗi ngần này xung. Pha hiệu chỉnh có lúc chạy trục Z liên tục
// hàng chục giây — không nhường sẽ bỏ đói task idle, watchdog có thể reset chip.
const int YIELD_EVERY_STEPS = 200;

// ================= Hiệu chỉnh tự động ngưỡng strength =================
// Pha dò chạy TRƯỚC khi quét: mỗi lần bàn xoay quay 45 độ thì trục Z nâng 1cm
// (đi xoắn ốc từ z=0), đọc vài mẫu tại mỗi điểm dừng. Khi đã qua đỉnh vật, tính
// ngưỡng từ 2 nhóm mẫu (trúng vật / trượt ra nền), rồi đưa Z về đúng z=0.
const int   CAL_TABLE_STEP_DEG     = 45;
const float CAL_Z_STEP_MM          = 10.0;
const int   CAL_READS_PER_STOP     = 5;    // đọc nhiều lần mỗi điểm dừng cho đủ mẫu
const int   CAL_EMPTY_STOPS_TO_END = 2;    // 2 điểm dừng liên tiếp không trúng vật = đã qua đỉnh
const int   CAL_MIN_OBJ_SAMPLES    = 5;    // ít hơn = không đủ tin để tính ngưỡng
const int   CAL_MIN_BG_SAMPLES     = 3;
const int   CAL_MAX_SAMPLES        = 200;
const int   CAL_TABLE_STEPS = TOTAL_STEPS_PER_REV * CAL_TABLE_STEP_DEG / 360;                     // 400 xung = 45 độ
const long  CAL_Z_STEPS     = (long)((CAL_Z_STEP_MM / LEAD_SCREW_PITCH_MM) * TOTAL_STEPS_PER_REV);   // 4000 xung = 1cm

// ================= Phân bổ nhân =================
const BaseType_t CORE_REALTIME = 1;   // APP_CPU — motor + cảm biến, không có WiFi
const BaseType_t CORE_COMM     = 0;   // PRO_CPU — xử lý + gửi, chung nhân với WiFi stack

// ================= Queue nối 2 nhân =================
struct RawSample {
  float radiusMm;      // bán kính từ tâm bàn xoay (đã đổi sang mm ở nhân 1)
  float angleRad;
  float zMm;
  bool  endOfScan;     // true = hết dữ liệu, nhân 0 dọn dẹp rồi thoát
};
const int QUEUE_LENGTH = 256;   // ≈ 11 giây dữ liệu — đủ che một cú giật WiFi dài
QueueHandle_t rawQueue = NULL;

// ---------------- WiFi / TCP — chỉ nhân 0 được gọi ----------------
void connectToServer() {
  Serial.println("Waiting for PC server...");
  while (!client.connect(serverIP, serverPort)) {
    Serial.println("  connect failed, retrying in 1s...");
    delay(1000);
  }
  Serial.println("Connected to PC.");
}

// ---------------- TF-Luna qua UART — chỉ nhân 1 được gọi ----------------
void tfLunaInit() {
  TFLunaSerial.begin(TFLUNA_BAUD, SERIAL_8N1, PIN_TFLUNA_RX, PIN_TFLUNA_TX);
  delay(100);   // cho cảm biến kịp phát vài khung đầu tiên
}

// Đọc 1 khung THÔ: khoảng cách + strength, KHÔNG lọc gì. Trả về false nếu
// timeout hoặc không có khung nào đúng checksum kịp lúc.
bool readTFLunaRaw(int& distanceCm, int& strength) {
  // Xả sạch byte cũ còn tồn trong buffer để đảm bảo lấy đúng khung MỚI —
  // cảm biến phát liên tục ~100Hz, không phải kiểu "kích 1 lần" như I2C cũ.
  while (TFLunaSerial.available()) TFLunaSerial.read();

  uint8_t buf[9];
  int idx = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < TFLUNA_READ_TIMEOUT_MS) {
    if (!TFLunaSerial.available()) {
      delay(1);   // nhường CPU cho task idle (nuôi watchdog), gần như không ảnh hưởng độ trễ
      continue;
    }
    uint8_t b = TFLunaSerial.read();

    if (idx < 2) {
      if (b == 0x59) buf[idx++] = b;   // tìm 2 byte header 0x59 0x59
      else idx = 0;                     // chưa đúng, reset tìm lại từ đầu
    } else {
      buf[idx++] = b;
      if (idx == 9) {
        uint8_t sum = 0;
        for (int i = 0; i < 8; i++) sum += buf[i];
        if (sum != buf[8]) { idx = 0; continue; }   // sai checksum, bỏ khung, tìm khung kế tiếp

        distanceCm = buf[2] | (buf[3] << 8);
        strength   = buf[4] | (buf[5] << 8);
        return true;
      }
    }
  }
  return false;   // timeout — không nhận được khung hợp lệ nào kịp lúc
}

// Dùng khi QUÉT: trả về khoảng cách (cm), hoặc -1 nếu lỗi / tín hiệu dưới ngưỡng.
float readDistanceCM() {
  int dist, strength;
  if (!readTFLunaRaw(dist, strength)) return -1.0;
  if (strength < g_minValidStrength) return -1.0;   // tín hiệu yếu
  return (float)dist;
}

// ---------------- Motor — chỉ nhân 1 được gọi ----------------
void stepPulse(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(5);        // A4988 cần xung ≥ 1µs
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY_US);
}

// Mọi chuyển động đều đi qua 2 hàm này — chiều quay chỉ khai báo 1 chỗ ở trên.
void rotateTableSteps(long steps) {
  digitalWrite(PIN_TABLE_DIR, TABLE_DIR_LEVEL);
  for (long i = 0; i < steps; i++) {
    stepPulse(PIN_TABLE_STEP);
    if ((i + 1) % YIELD_EVERY_STEPS == 0) delay(1);
  }
}

void moveZSteps(long steps, bool up) {
  const int downLevel = (Z_UP_LEVEL == HIGH) ? LOW : HIGH;
  digitalWrite(PIN_Z_DIR, up ? Z_UP_LEVEL : downLevel);
  for (long i = 0; i < steps; i++) {
    stepPulse(PIN_Z_STEP);
    if ((i + 1) % YIELD_EVERY_STEPS == 0) delay(1);
  }
}

void stepTurntable() { rotateTableSteps(STEPS_PER_SAMPLE); }
void stepZAxis()     { moveZSteps(Z_STEPS_PER_LAYER, true); }

void disableDrivers() {
  digitalWrite(PIN_TABLE_EN, HIGH);    // EN của A4988 tích cực mức THẤP
  digitalWrite(PIN_Z_EN, HIGH);
}

// Không có chuyển động cơ khí nào ở bước "home": vị trí Z lúc nhấn nút = z=0.
void waitForHomeButton() {
  Serial.println("Dat vat len GIUA ban xoay, ha truc Z xuong THAP NHAT (ngang mat ban xoay),");
  Serial.println("roi nhan nut HOME (GPIO7). May se do nguong strength, tu ve z=0, roi moi quet.");
  while (digitalRead(PIN_HOME_BUTTON) == HIGH) delay(50);   // HIGH = chưa nhấn
  delay(200);                                               // chống rung
  Serial.println("Da nhan HOME.");
}

// =====================================================================
// HIỆU CHỈNH TỰ ĐỘNG NGƯỠNG STRENGTH
// =====================================================================
// Vấn đề gốc: để biết "lớp này không quét được gì" cần một tiêu chí phân biệt
// trúng vật / trượt ra ngoài — nhưng tiêu chí đó chính là ngưỡng strength đang
// cần tìm. Vì vậy pha dò phân loại bằng HÌNH HỌC (khoảng cách), không dùng
// strength: trúng vật thì khoảng cách < tâm bàn xoay (bán kính > 0); trượt qua
// đỉnh vật thì tia bay qua trục quay, đập vào nền phía sau (bán kính <= 0),
// hoặc không có phản xạ nào (TF-Luna trả khoảng cách 0).
enum CalClass { CAL_UNUSABLE, CAL_OBJECT, CAL_BACKGROUND };

CalClass classifyReading(int distCm) {
  if (distCm <= 0) return CAL_BACKGROUND;              // không có phản xạ
  float rMm = (DISTANCE_TO_CENTER_CM - distCm) * 10.0f;
  if (rMm <= 0) return CAL_BACKGROUND;                 // tia đi qua trục quay, không chạm vật
  if (rMm <= MAX_RADIUS_MM) return CAL_OBJECT;         // trúng vật trên bàn xoay
  return CAL_UNUSABLE;                                  // quá gần cảm biến (vùng mù) — bỏ
}

struct CalResult {
  bool ok;          // đủ mẫu trúng vật để tính
  bool separated;   // vật và nền tách bạch rõ theo strength
  int  objLow;      // strength trúng vật, mức thấp (bách phân vị 10)
  int  bgHigh;      // strength nền, mức cao (bách phân vị 90); -1 nếu không đủ mẫu nền
  int  threshold;
};

static void sortInts(int* a, int n) {
  for (int i = 1; i < n; i++) {        // insertion sort — mảng nhỏ (<= 200)
    int v = a[i], j = i - 1;
    while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
    a[j + 1] = v;
  }
}

static int percentileInt(const int* sorted, int n, float p) {
  return sorted[(int)(p * (n - 1))];
}

// Hàm thuần, không đụng phần cứng. Dùng bách phân vị thay vì min/max để vài mẫu
// bất thường (mép vật, nhiễu) không kéo lệch ngưỡng.
CalResult computeStrengthThreshold(int* obj, int nObj, int* bg, int nBg) {
  CalResult r = { false, false, 0, -1, TFLUNA_DEFAULT_MIN_STRENGTH };
  if (nObj < CAL_MIN_OBJ_SAMPLES) return r;          // không đủ mẫu -> giữ mặc định

  sortInts(obj, nObj);
  r.objLow = percentileInt(obj, nObj, 0.10f);
  if (nBg >= CAL_MIN_BG_SAMPLES) {
    sortInts(bg, nBg);
    r.bgHigh = percentileInt(bg, nBg, 0.90f);
  }

  int thr;
  if (r.bgHigh >= 0 && r.bgHigh < r.objLow) {
    thr = (r.bgHigh + r.objLow) / 2;                  // tách bạch rõ -> lấy điểm giữa
    r.separated = true;
  } else {
    // Nền phía sau phản xạ mạnh ngang vật (hoặc không có mẫu nền): strength không
    // phân biệt được. Chỉ cắt đuôi yếu nhất của mẫu vật — việc loại nền đã có bộ
    // lọc hình học (bán kính > 0) trong vòng quét lo.
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
  long zStepsUp = 0;                   // đếm chính xác số xung đã đi lên để về đúng z=0

  for (int stopIdx = 0; ; stopIdx++) {
    float zMm = stopIdx * CAL_Z_STEP_MM;

    int hits = 0;
    for (int k = 0; k < CAL_READS_PER_STOP; k++) {
      int dist, strength;
      if (!readTFLunaRaw(dist, strength)) continue;   // lỗi khung/timeout: bỏ mẫu này
      CalClass c = classifyReading(dist);
      if (c == CAL_OBJECT) {
        hits++;
        if (nObj < CAL_MAX_SAMPLES) calObj[nObj++] = strength;
      } else if (c == CAL_BACKGROUND) {
        if (nBg < CAL_MAX_SAMPLES) calBg[nBg++] = strength;
      }
    }
    Serial.printf("[cal] z=%.0fmm: %d/%d mau trung vat\n", zMm, hits, CAL_READS_PER_STOP);

    // Đã qua đỉnh vật? (chỉ xét sau Z_MIN_SCAN_MM, giống vòng quét)
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

    // Kiểm tra vị trí KẾ TIẾP trước khi đi, để không bao giờ vượt giới hạn hành trình
    if ((stopIdx + 1) * CAL_Z_STEP_MM > Z_TRAVEL_LIMIT_MM) {
      Serial.println("[cal] Toi gioi han hanh trinh Z, dung do.");
      break;
    }

    rotateTableSteps(CAL_TABLE_STEPS);
    moveZSteps(CAL_Z_STEPS, true);
    zStepsUp += CAL_Z_STEPS;
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
  g_minValidStrength = r.threshold;    // khi !ok, r.threshold = giá trị mặc định

  // Đưa trục Z về đúng z=0: đi xuống ĐÚNG số xung đã đi lên.
  Serial.printf("[cal] Dua truc Z ve z=0 (%ld xung)...\n", zStepsUp);
  moveZSteps(zStepsUp, false);
  Serial.println("[cal] Da ve z=0.");
  return r;
}

// ---------------- TASK NHÂN 1 — THỜI GIAN THỰC ----------------
void motorTask(void* pv) {
  waitForHomeButton();

  if (AUTO_CALIBRATE_STRENGTH) {
    calibrateStrength();
  } else {
    Serial.printf("[cal] Bo qua hieu chinh, dung nguong mac dinh %d.\n", g_minValidStrength);
  }

  Serial.println("[core1] Bat dau quet toa do tu z=0.");
  uint32_t startMs = millis();
  int  emptyLayers  = 0;               // số lớp liên tiếp không thu được điểm nào
  long scanZStepsUp = 0;               // đếm chính xác số xung Z đã đi lên khi quét, để về đúng z=0

  for (int layer = 0; ; layer++) {
    float z = layer * Z_LAYER_MM;      // nhân thay vì cộng dồn → không trôi số thực

    int validThisLayer = 0;
    float angle = 0;
    for (int i = 0; i < STEPS_PER_REV; i++) {
      float d = readDistanceCM();

      if (d > 0) {
        float rMm = (DISTANCE_TO_CENTER_CM - d) * 10.0f;   // khoảng cách → bán kính, cm → mm
        if (rMm > 0 && rMm <= MAX_RADIUS_MM) {
          validThisLayer++;
          RawSample s = { rMm, angle, z, false };
          xQueueSend(rawQueue, &s, pdMS_TO_TICKS(5000));
        }
      }

      angle += D_ANGLE;
      stepTurntable();
    }

    // Tự dừng khi đã quét qua đỉnh vật (chỉ xét sau Z_MIN_SCAN_MM, tránh dừng
    // ngay lớp đầu nếu cảm biến chưa nhìn thấy gì do lắp đặt sai)
    if (z >= Z_MIN_SCAN_MM) {
      if (validThisLayer == 0) {
        if (++emptyLayers >= EMPTY_LAYERS_TO_STOP) {
          Serial.printf("[core1] Top of object reached at z=%.0fmm. Stopping.\n", z);
          break;
        }
      } else {
        emptyLayers = 0;               // có điểm trở lại → đặt lại bộ đếm
      }
    }

    // Kiểm tra lớp KẾ TIẾP trước khi nâng Z, để không vượt giới hạn hành trình
    if ((layer + 1) * Z_LAYER_MM > Z_TRAVEL_LIMIT_MM) {
      Serial.printf("[core1] Reached travel limit (%.0fmm). Stopping.\n", Z_TRAVEL_LIMIT_MM);
      break;
    }

    stepZAxis();
    scanZStepsUp += Z_STEPS_PER_LAYER;
    if ((layer + 1) % 10 == 0) {       // báo tiến độ thưa thớt, không làm rối nhịp
      Serial.printf("[core1] layer %d done (z=%.0fmm), %lu s elapsed\n",
                    layer + 1, z, (millis() - startMs) / 1000);
    }
  }

  // Gửi tín hiệu kết thúc TRƯỚC khi hạ Z: nhân 0 lập tức gửi nốt dữ liệu và đóng
  // kết nối, file trên PC hoàn chỉnh ngay — trong lúc nhân 1 hạ trục Z song song.
  RawSample endMarker = { 0, 0, 0, true };
  xQueueSend(rawQueue, &endMarker, portMAX_DELAY);
  Serial.println("[core1] Motion complete.");

  // Hạ Z về z=0 khi driver VẪN ĐANG BẬT. Nếu tắt driver trước, xung vẫn phát
  // nhưng động cơ không quay — trục Z nằm yên trên cao dù code tưởng đã về 0.
  if (RETURN_Z_AFTER_SCAN && scanZStepsUp > 0) {
    Serial.printf("[core1] Dang ha truc Z ve z=0 (%ld xung)... cho truc Z dung han.\n", scanZStepsUp);
    moveZSteps(scanZStepsUp, false);
    Serial.println("[core1] Da ve z=0. San sang cho lan quet tiep theo.");
  }

  disableDrivers();                    // tắt SAU CÙNG, không để motor giữ điện nóng
  vTaskDelete(NULL);
}

// ---------------- TASK NHÂN 0 — TRUYỀN THÔNG ----------------
static char   txBuf[1460];             // ≈ 1 gói TCP
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
    if (xQueueReceive(rawQueue, &s, portMAX_DELAY) != pdTRUE) continue;   // ngủ tới khi có dữ liệu

    if (s.endOfScan) {
      flushTxBuffer();
      client.stop();                   // PC thấy kết nối đóng = biết đã xong
      Serial.println();
      Serial.println("=============== SCAN COMPLETE ===============");
      Serial.println("File complete on PC. Run xyz_to_stl.py to build the STL.");
      Serial.println("=============================================");
      vTaskDelete(NULL);
    }

    float x = s.radiusMm * cos(s.angleRad);
    float y = s.radiusMm * sin(s.angleRad);

    if (sizeof(txBuf) - txLen < 40) flushTxBuffer(); // chừa chỗ cho 1 dòng
    int n = snprintf(txBuf + txLen, sizeof(txBuf) - txLen, "%.2f,%.2f,%.2f\n", x, y, s.zMm);
    if (n > 0) txLen += n;
  }
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_TABLE_STEP, OUTPUT);
  pinMode(PIN_TABLE_DIR,  OUTPUT);
  pinMode(PIN_TABLE_EN,   OUTPUT);
  pinMode(PIN_Z_STEP,     OUTPUT);
  pinMode(PIN_Z_DIR,      OUTPUT);
  pinMode(PIN_Z_EN,       OUTPUT);
  digitalWrite(PIN_TABLE_EN, LOW);     // bật driver (EN tích cực mức thấp)
  digitalWrite(PIN_Z_EN, LOW);
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
  if (rawQueue == NULL) {
    Serial.println("ERROR: cannot create queue.");
    while (true) delay(1000);
  }

  // Task motor ưu tiên cao hơn để không bị chen trong lúc lấy mẫu
  xTaskCreatePinnedToCore(commTask,  "comm",  8192, NULL, 1, NULL, CORE_COMM);
  xTaskCreatePinnedToCore(motorTask, "motor", 4096, NULL, 3, NULL, CORE_REALTIME);

  Serial.println("Dual-core pipeline started.");
}

void loop() {
  delay(1000);   // mọi việc đã giao cho 2 task
}
