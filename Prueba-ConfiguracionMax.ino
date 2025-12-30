#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 sensor;

// Promedio
const byte RATE_SIZE = 8;
float rates[RATE_SIZE];
byte rateSpot = 0;

float bpm = 0;
float bpmAvg = 0;
unsigned long lastBeat = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);

  Serial.println("Inicializando MAX30102...");

  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("Sensor no encontrado");
    while (1);
  }

  // ===== CONFIGURACIÓN CORRECTA =====
  sensor.setup(
    40,     // brillo LED moderado
    4,      // promedio
    2,      // IR only
    100,    // sample rate
    411,    // pulse width
    8192    // ADC range (CLAVE)
  );

  sensor.setPulseAmplitudeIR(0x10);   // 🔑 NO saturar
  sensor.setPulseAmplitudeRed(0x00);
  sensor.setPulseAmplitudeGreen(0);

  Serial.println("Coloca el dedo sin apretar");
}

void loop() {
  long irValue = sensor.getIR();

  // Detección de dedo
  if (irValue < 10000) {
    Serial.println("No hay dedo");
    delay(300);
    return;
  }

  // Detección de latido (librería probada)
  if (checkForBeat(irValue)) {
    unsigned long now = millis();
    unsigned long delta = now - lastBeat;
    lastBeat = now;

    if (delta > 300 && delta < 1500) {
      bpm = 60000.0 / delta;

      rates[rateSpot++] = bpm;
      rateSpot %= RATE_SIZE;

      bpmAvg = 0;
      for (byte i = 0; i < RATE_SIZE; i++)
        bpmAvg += rates[i];
      bpmAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(" | BPM=");
  Serial.print(bpm, 1);
  Serial.print(" | Avg=");
  Serial.println(bpmAvg, 1);

  delay(10);
}