#define SENSOR_PIN 17
#define NUM_SAMPLES 20

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // ADC: 0-4095

  // ADC range for ESP32-S3
  analogSetPinAttenuation(SENSOR_PIN, ADC_11db);

  delay(1000);
  Serial.println("GP2Y0A51SK0F Sensor Test");
  Serial.println("ADC\tVoltage (V)");
}

void loop() {
  uint32_t adcSum = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    adcSum += analogRead(SENSOR_PIN);
    delay(2);
  }

  float adcAverage = adcSum / (float)NUM_SAMPLES;
  float voltage = adcAverage * 3.3 / 4095.0;

  Serial.print(adcAverage, 1);
  Serial.print('\t');
  Serial.println(voltage, 3);

  delay(500);
}