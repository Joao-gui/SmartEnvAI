#include <Arduino.h>

#define SENSOR_PIN 34
const float ADC_REF = 5.0;           // tensão de referência do ADC do ESP32
const int ADC_MAX = 4095;            // resolução 12 bits
// Sensibilidade (mV/A) - ajuste de acordo com sua versão do ACS712:
const float SENSITIVITY_mV_per_A = 185.0; // exemplo ACS712-5A

// Se você alimenta o ACS712 com 5.0V:
const float ACS_VCC = 5.0;
const float ACS_ZERO_VOLTAGE = ACS_VCC / 2.0; // tensão de saída com corrente zero

// Para calibração:
float zero_offset = 0.0; // em volts (medir com circuito desconectado ou sem corrente)

void setup() {
  Serial.begin(115200);
  pinMode(SENSOR_PIN, INPUT);
  delay(500);
  // Calibração simples: média de N leituras com corrente zero
  const int N = 100;
  long sum = 0;
  for (int i = 0; i < N; ++i) {
    sum += analogRead(SENSOR_PIN);
    delay(5);
  }
  float avgRaw = sum / (float)N;
  zero_offset = (avgRaw * (ADC_REF / ADC_MAX)); // volts
  Serial.print("Zero offset (V): ");
  Serial.println(zero_offset, 4);
}

float readCurrentAverages(int samples = 30) {
  long s = 0;
  for (int i = 0; i < samples; ++i) {
    s += analogRead(SENSOR_PIN);
    delay(2);
  }
  float rawAvg = s / (float)samples;
  float v = rawAvg * (ADC_REF / ADC_MAX); // tensão medida
  // corrente:
  // primeiro converte (v - zero_offset) em mV e divide pela sensibilidade (mV/A)
  float currentA = ((v - zero_offset) * 1000.0) / SENSITIVITY_mV_per_A;
  return currentA;
}

void loop() {
  float I = readCurrentAverages(50);
  Serial.print("Corrente (A): ");
  Serial.println(I, 3);
  delay(500);
}
