#include <Arduino.h>

// ---- Chân kết nối ----
const int tep1 = 5;   // Table (motor 1)
const int dir1 = 4;
const int ena1 = 6;

const int tep2 = 11;  // Trục Z (motor 2)
const int dir2 = 10;
const int ena2 = 12;

// ---- Thông số ----
const int  STEPS_PER_REV      = 200;      // 200 xung = 360 độ (1.8 độ/step)
const unsigned long TABLE_STEP_MS = 50;   // Mỗi step của table = 50ms
const int  Z_STEPS_PER_TABLE_REV = 2;     // Table xoay 1 vòng -> Z xoay 2 step (3.6 độ)
const int  Z_TOTAL_REVS       = 15;       // Z xoay đủ 15 vòng thì hạ về
const long Z_TOTAL_STEPS      = (long)Z_TOTAL_REVS * STEPS_PER_REV; // 3000 step

const unsigned int Z_STEP_US   = 1000;    // Tốc độ step của Z khi nâng (us)
const unsigned int Z_RETURN_US = 500;     // Tốc độ step của Z khi hạ về (us), có thể chỉnh

const int Z_UP   = HIGH;  // Chiều nâng (đổi thành LOW nếu bị ngược)
const int Z_DOWN = LOW;   // Chiều hạ

long zSteps = 0;          // Số step Z đã đi lên
bool finished = false;

// Phát 1 xung step, chu kỳ = 2 * halfPeriodUs
void stepPulseUs(int pin, unsigned int halfPeriodUs)
{
  digitalWrite(pin, HIGH);
  delayMicroseconds(halfPeriodUs);
  digitalWrite(pin, LOW);
  delayMicroseconds(halfPeriodUs);
}

// Table: 1 step trong 50ms (25ms HIGH + 25ms LOW)
void tableStep()
{
  digitalWrite(tep1, HIGH);
  delay(TABLE_STEP_MS / 2);
  digitalWrite(tep1, LOW);
  delay(TABLE_STEP_MS / 2);
}

void setup()
{
  Serial.begin(115200);

  pinMode(ena1, OUTPUT);
  pinMode(tep1, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(ena2, OUTPUT);
  pinMode(tep2, OUTPUT);
  pinMode(dir2, OUTPUT);

  digitalWrite(ena1, LOW);   // Bật driver motor 1
  digitalWrite(ena2, LOW);   // Bật driver motor 2
  digitalWrite(dir1, HIGH);  // Chiều quay table
  digitalWrite(dir2, Z_UP);  // Z đi lên
}

void loop()
{
  if (finished) return;      // Chạy 1 chu trình rồi dừng

  // 1) Table xoay 1 vòng (200 step x 50ms = 10s)
  digitalWrite(dir1, HIGH);
  for (int i = 0; i < STEPS_PER_REV; i++)
  {
    tableStep();
  }

  // 2) Sau mỗi vòng table, Z xoay 2 step (3.6 độ)
  digitalWrite(dir2, Z_UP);
  for (int i = 0; i < Z_STEPS_PER_TABLE_REV; i++)
  {
    stepPulseUs(tep2, Z_STEP_US);
    zSteps++;
  }

  Serial.print("Z steps: ");
  Serial.println(zSteps);

  // 3) Z đủ 15 vòng (3000 step) -> hạ về vị trí ban đầu
  if (zSteps >= Z_TOTAL_STEPS)
  {
    Serial.println("Z ha ve vi tri ban dau...");
    digitalWrite(dir2, Z_DOWN);
    delay(10);               // Chờ đổi chiều ổn định
    for (long i = 0; i < zSteps; i++)
    {
      stepPulseUs(tep2, Z_RETURN_US);
    }
    zSteps = 0;
    finished = true;         // Muốn chạy lặp lại thì bỏ dòng này
    Serial.println("Hoan thanh.");
  }
}