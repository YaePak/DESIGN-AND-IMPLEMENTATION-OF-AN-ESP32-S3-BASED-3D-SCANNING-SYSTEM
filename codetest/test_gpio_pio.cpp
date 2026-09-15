/*
  Test GPIO — Nút Start & Công tắc hành trình
  ------------------------------------------------
  Chỉ kiểm tra 2 chân digital I/O đơn giản nhất của hệ thống, tách riêng
  khỏi I2C (TF-Luna) và driver động cơ (A4988) để dễ khoanh vùng khi có lỗi.

  Cần nối: nút Start vào GPIO1 + GND, công tắc hành trình vào GPIO2 + GND.
  KHÔNG cần nối gì khác — không cần cảm biến, không cần động cơ.

  Cách test: mở Serial Monitor (115200 baud), nhấn/thả nút và công tắc,
  quan sát trạng thái đổi ngay theo thời gian thực. Chỉ in khi trạng thái
  thay đổi, không in liên tục, để dễ theo dõi từng lần nhấn.
*/

#include <Arduino.h>

const int PIN_BUTTON   = 1;   // nối xuống GND khi nhấn (INPUT_PULLUP)
const int PIN_LIMIT_SW = 2;   // nối xuống GND khi chạm (INPUT_PULLUP)

int lastButtonState = -1;     // -1 = chưa đọc lần nào, ép in ngay lần đầu
int lastLimitState  = -1;

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LIMIT_SW, INPUT_PULLUP);

  Serial.println();
  Serial.println("=== Test GPIO: nut Start (GPIO1) & cong tac hanh trinh (GPIO2) ===");
  Serial.println("Nhan/tha nut va cong tac de xem trang thai doi theo thoi gian thuc.");
  Serial.println();
}

void loop() {
  int buttonState = digitalRead(PIN_BUTTON);
  int limitState  = digitalRead(PIN_LIMIT_SW);

  if (buttonState != lastButtonState) {
    Serial.print("[GPIO1] BUTTON   -> ");
    Serial.println(buttonState == LOW ? "PRESSED" : "released");
    lastButtonState = buttonState;
  }
  if (limitState != lastLimitState) {
    Serial.print("[GPIO2] LIMIT_SW -> ");
    Serial.println(limitState == LOW ? "TRIGGERED" : "released");
    lastLimitState = limitState;
  }

  delay(20);   // giam nhe de bot doc trung do rung co khi khi tha nut/cong tac
}
