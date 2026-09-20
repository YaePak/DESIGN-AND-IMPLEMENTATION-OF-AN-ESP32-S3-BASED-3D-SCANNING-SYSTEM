#include <Arduino.h>

const int PIN_HOME_BUTTON = 7;   // nối xuống GND khi nhấn (INPUT_PULLUP)

int lastState = -1;   // -1 = chưa đọc lần nào, ép in ngay lần đầu

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(PIN_HOME_BUTTON, INPUT_PULLUP);

  Serial.println();
  Serial.println("=== Test GPIO: nut HOME (GPIO7) ===");
  Serial.println("Nhan/tha nut de xem trang thai doi theo thoi gian thuc.");
  Serial.println();
}

void loop() {
  int state = digitalRead(PIN_HOME_BUTTON);
  if (state != lastState) {
    Serial.print("[GPIO7] HOME -> ");
    Serial.println(state == LOW ? "PRESSED" : "released");
    lastState = state;
  }
  delay(20);   // giam nhe de bot doc trung do rung co khi khi tha nut
}
