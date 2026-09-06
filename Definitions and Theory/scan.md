#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// xài esp32 phát wifi r kn trực tiếp với mtinh r truyền dữ liệu theo thgian thực qua wifi
// WiFi Access Point
const char* AP_SSID     = "ScannerESP32";
const char* AP_PASSWORD = "scanner123";
IPAddress serverIP(192, 168, 4, 2);     
const uint16_t serverPort = 5000;
WiFiClient client;

// Bảng chân GPIO
const int I2C_SDA = 15;
const int I2C_SCL = 16;
const int PIN_TABLE_STEP = 5;
const int PIN_TABLE_DIR  = 6;
const int PIN_TABLE_EN   = 7;
const int PIN_Z_STEP     = 8;
const int PIN_Z_DIR      = 9;
const int PIN_Z_EN       = 14;
const int PIN_BUTTON   = 1;   // nút Start, nối xuống GND khi nhấn (INPUT_PULLUP)
const int PIN_LIMIT_SW = 2;   // công tắc hành trình Z, nối xuống GND khi chạm (INPUT_PULLUP)

// TF-Luna I2C
const uint8_t TFL_ADDR     = 0x10;
const uint8_t TFL_DIST_LO  = 0x00;
const uint8_t TFL_SET_MODE = 0x23;    // 0 = đo liên tục, 1 = đo theo lệnh
const uint8_t TFL_TRIGGER  = 0x24;    // 1 = đo một lần
const int TFL_MIN_VALID_FLUX = 100;  // để tạm, chỉnh sau = file checkFLUX

// Động cơ & hình học quét
const int   MOTOR_STEPS_PER_REV = 200;   // NEMA17 1,8°/bước, datasheet
const int   MICROSTEPPING       = 16;    // MS1/MS2/MS3 đều HIGH (1,2,4,8,16)
const int   TOTAL_STEPS_PER_REV = MOTOR_STEPS_PER_REV * MICROSTEPPING;   
const float LEAD_SCREW_PITCH_MM = 8.0;   

const int   STEPS_PER_REV    = 80;       // đo mỗi vòng (360/80=4,5°)
const float D_ANGLE          = 2.0 * PI / STEPS_PER_REV;
const int   STEPS_PER_SAMPLE = TOTAL_STEPS_PER_REV / STEPS_PER_REV;     

const float Z_LAYER_MM = 1.0;
const float Z_TRAVEL_LIMIT_MM = 150.0;   // đoạn lên thật của scan, đo lại r sửa lại chi tiết

const int   Z_STEPS_PER_LAYER = (int)((Z_LAYER_MM / LEAD_SCREW_PITCH_MM) * TOTAL_STEPS_PER_REV);   

const float Z_MIN_SCAN_MM        = 5.0;  // quét ít nhất 5mm rồi mới cho phép tự dừng
const int   EMPTY_LAYERS_TO_STOP = 2;    // cần 2 lớp trống liên tiếp, tránh dừng nhầm ở khe hẹp

const float DISTANCE_TO_CENTER_CM = 28.0;
const float MAX_RADIUS_MM         = 150.0;   // xa hơn = tia bắn trượt ra ngoài vật

// Hai núm chỉnh tốc độ quét
const int STEP_DELAY_US = 600; // nhỏ -> nhanh hơn, motor có thể bị rung và ngc lại
const int SENSOR_SETTLE_MS = 8;
// xài dual core
// Phân bổ nhân
const BaseType_t CORE_REALTIME = 1;   // APP_CPU — motor + cảm biến, không có WiFi
const BaseType_t CORE_COMM     = 0;   // PRO_CPU — xử lý + gửi, chung nhân với WiFi stack

// Queue nối 2 nhân
struct RawSample {
  float radiusMm;      // bán kính từ tâm bàn xoay (đã đổi sang mm ở nhân 1)
  float angleRad;
  float zMm;
  bool  endOfScan;     // true = hết dữ liệu, nhân 0 dọn dẹp rồi thoát
};
const int QUEUE_LENGTH = 256;   // ≈ 11 giây dữ liệu — đủ che một cú giật WiFi dài
QueueHandle_t rawQueue = NULL;

// xài dual core
// WiFi / TCP — chỉ nhân 0 được gọi 
void connectToServer() {
  Serial.println("Waiting for PC server...");
  while (!client.connect(serverIP, serverPort)) {
    Serial.println("  connect failed, retrying in 1s...");
    delay(1000);
  }
  Serial.println("Connected to PC.");
}

// TF-Luna qua I2C — chỉ nhân 1 được gọi 
void tfLunaInit() {
  Wire.beginTransmission(TFL_ADDR);
  Wire.write(TFL_SET_MODE);
  Wire.write(0x01);                                  // chế độ trigger
  if (Wire.endTransmission() != 0) {
    Serial.println("WARNING: TF-Luna not responding on I2C.");
  } else {
    Serial.println("TF-Luna set to trigger mode.");
  }
  delay(10);
}

// Trả về khoảng cách (cm), hoặc -1 nếu lỗi I2C / tín hiệu quá yếu.
float readDistanceCM() {
  Wire.beginTransmission(TFL_ADDR);                  // 1) kích một lần đo
  Wire.write(TFL_TRIGGER);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(SENSOR_SETTLE_MS);                           // delay() = vTaskDelay, nhả CPU trong lúc chờ

  Wire.beginTransmission(TFL_ADDR);                  // 2) đọc 4 byte từ 0x00: dist(2) + flux(2)
  Wire.write(TFL_DIST_LO);
  if (Wire.endTransmission(false) != 0) return -1.0;   // repeated start; không ACK = lỗi dây
  Wire.requestFrom((int)TFL_ADDR, 4);
  if (Wire.available() < 4) return -1.0;

  uint8_t distLo = Wire.read();
  uint8_t distHi = Wire.read();
  uint8_t fluxLo = Wire.read();
  uint8_t fluxHi = Wire.read();
  int distanceCm = (distHi << 8) | distLo;
  int flux       = (fluxHi << 8) | fluxLo;
  if (flux < TFL_MIN_VALID_FLUX) return -1.0;        // tín hiệu yếu
  return (float)distanceCm;
}

// Motor — chỉ nhân 1 được gọi
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

void homeZAxis() {
  Serial.println("Homing Z axis...");
  digitalWrite(PIN_Z_DIR, LOW);        // đặt đúng chiều VỀ PHÍA công tắc
  const int maxHomingSteps = 20000;    // giới hạn an toàn: ~50mm với cấu hình hiện tại
  int steps = 0;
  while (digitalRead(PIN_LIMIT_SW) == HIGH) {        // HIGH = chưa chạm
    stepPulse(PIN_Z_STEP);
    if (++steps > maxHomingSteps) {
      Serial.println("ERROR: homing timed out. Check limit switch wiring/direction.");
      disableDrivers();
      while (true) delay(1000);        // không được quét khi chưa biết vị trí Z
    }
  }
  Serial.println("Z axis homed.");
}

void waitForStartButton() {
  Serial.println("Place the object on the turntable, then press the start button.");
  while (digitalRead(PIN_BUTTON) == HIGH) delay(50); // HIGH = chưa nhấn
  delay(200);                                        // chống rung
  Serial.println("Button pressed. Starting scan.");
}

// TASK NHÂN 1 — THỜI GIAN THỰC
void motorTask(void* pv) {
  homeZAxis();
  waitForStartButton();
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
    if ((layer + 1) % 10 == 0) {      
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

// TASK NHÂN 0 — TRUYỀN THÔNG
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
 
//====================================================
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
  pinMode(PIN_BUTTON,   INPUT_PULLUP);
  pinMode(PIN_LIMIT_SW, INPUT_PULLUP);
 
  Wire.begin(I2C_SDA, I2C_SCL);
  tfLunaInit();
 
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
  delay(1000);   
}