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
const int PIN_TFLUNA_RX = 17;   // ESP32 RX <- chân TXD của TF-Luna
const int PIN_TFLUNA_TX = 18;   // ESP32 TX -> chân RXD của TF-Luna
// Nút HOME duy nhất (không còn công tắc hành trình riêng)
const int PIN_HOME_BUTTON = 7;  // nối xuống GND khi nhấn (INPUT_PULLUP)

// ================= TF-Luna qua UART =================
HardwareSerial TFLunaSerial(1);           // dùng UART1 của ESP32-S3
const uint32_t TFLUNA_BAUD = 115200;      // tốc độ mặc định của TF-Luna
const uint32_t TFLUNA_READ_TIMEOUT_MS = 50;  // chờ tối đa 1 khung hợp lệ
const int TFLUNA_MIN_VALID_STRENGTH = 1000;  // TODO: hiệu chỉnh lại bằng sketch test_tfluna

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

const float DISTANCE_TO_CENTER_CM = 28.0;
const float MAX_RADIUS_MM         = 150.0;   // xa hơn = tia bắn trượt ra ngoài vật

// ================= Núm chỉnh tốc độ quét =================
const int STEP_DELAY_US = 600;   // nhỏ -> nhanh hơn, motor có thể bị rung nếu quá nhỏ

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

// Trả về khoảng cách (cm), hoặc -1 nếu timeout / sai checksum / tín hiệu yếu.
float readDistanceCM() {
  // Xả sạch byte cũ còn tồn trong buffer để đảm bảo lấy đúng khung MỚI —
  // cảm biến phát liên tục ~100Hz, không phải kiểu "kích 1 lần" như I2C cũ.
  while (TFLunaSerial.available()) TFLunaSerial.read();

  uint8_t buf[9];
  int idx = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < TFLUNA_READ_TIMEOUT_MS) {
    if (!TFLunaSerial.available()) continue;
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

        int distanceCm = buf[2] | (buf[3] << 8);
        int strength   = buf[4] | (buf[5] << 8);
        if (strength < TFLUNA_MIN_VALID_STRENGTH) return -1.0;   // tín hiệu yếu
        return (float)distanceCm;
      }
    }
  }
  return -1.0;   // timeout — không nhận được khung hợp lệ nào kịp lúc
}

// ---------------- Motor — chỉ nhân 1 được gọi ----------------
void stepPulse(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(5);        // A4988 cần xung ≥ 1µs
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY_US);
}

void stepTurntable() {
  digitalWrite(PIN_TABLE_DIR, HIGH);   // đảo thành LOW nếu quay sai chiều
  for (int i = 0; i < STEPS_PER_SAMPLE; i++) stepPulse(PIN_TABLE_STEP);
}

void stepZAxis() {
  digitalWrite(PIN_Z_DIR, HIGH);       // đảo thành LOW nếu đi sai chiều
  for (int i = 0; i < Z_STEPS_PER_LAYER; i++) stepPulse(PIN_Z_STEP);
}

void disableDrivers() {
  digitalWrite(PIN_TABLE_EN, HIGH);    // EN của A4988 tích cực mức THẤP
  digitalWrite(PIN_Z_EN, HIGH);
}

// Không còn chuyển động cơ khí nào ở bước "home" — xem giải thích ở đầu file.
// Chỉ chờ đúng 1 lần nhấn, dùng làm cả mốc Z=0 lẫn lệnh bắt đầu quét.
void waitForHomeButton() {
  Serial.println("Dat vat len ban xoay, dua truc Z ve vi tri muon lam goc,");
  Serial.println("roi nhan nut HOME (GPIO7) de bat dau quet ngay (Z=0 tai day).");
  while (digitalRead(PIN_HOME_BUTTON) == HIGH) delay(50);   // HIGH = chưa nhấn
  delay(200);                                               // chống rung
  Serial.println("Da nhan HOME. Bat dau quet.");
}

// ---------------- TASK NHÂN 1 — THỜI GIAN THỰC ----------------
void motorTask(void* pv) {
  waitForHomeButton();
  uint32_t startMs = millis();
  int emptyLayers = 0;                 // số lớp liên tiếp không thu được điểm nào

  for (int layer = 0; ; layer++) {
    float z = layer * Z_LAYER_MM;      // nhân thay vì cộng dồn → không trôi số thực

    if (z > Z_TRAVEL_LIMIT_MM) {       // chặn an toàn cơ khí
      Serial.printf("[core1] Reached travel limit (%.0fmm). Stopping.\n", Z_TRAVEL_LIMIT_MM);
      break;
    }

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

    stepZAxis();
    if ((layer + 1) % 10 == 0) {       // báo tiến độ thưa thớt, không làm rối nhịp
      Serial.printf("[core1] layer %d done (z=%.0fmm), %lu s elapsed\n",
                    layer + 1, z, (millis() - startMs) / 1000);
    }
  }

  disableDrivers();                    // không để motor giữ điện nóng sau khi xong

  RawSample endMarker = { 0, 0, 0, true };
  xQueueSend(rawQueue, &endMarker, portMAX_DELAY);
  Serial.println("[core1] Motion complete.");
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
  float d0 = readDistanceCM();
  if (d0 > 0) {
    Serial.print("TF-Luna OK, khoang cach hien tai: ");
    Serial.print(d0);
    Serial.println(" cm");
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
