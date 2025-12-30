#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 sensor;

// ===== CORRECCIÓN: Usar FLOAT para rates =====
const byte RATE_SIZE = 10;
float rates[RATE_SIZE];  // CAMBIO CLAVE: float en lugar de int
byte rateSpot = 0;
byte rateCount = 0;

long lastBeat = 0;
float bpm = 0;
float bpmAvg = 0;  // También float

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);

  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("Sensor no encontrado");
    while (1);
  }

  sensor.setup(40, 4, 2, 100, 411, 8192);
  sensor.setPulseAmplitudeIR(0x10);
  sensor.setPulseAmplitudeRed(0x00);

  // Inicializar con floats
  for (byte i = 0; i < RATE_SIZE; i++) {
    rates[i] = 0.0;  // 0.0 en lugar de 0
  }

  Serial.println("✅ AVG calculado CORRECTAMENTE (float)");
}

void loop() {
  long irValue = sensor.getIR();
  
  if (irValue < 10000) {
    Serial.println("No hay dedo");
    delay(300);
    return;
  }

  if (checkForBeat(irValue)) {
    long now = millis();
    long delta = now - lastBeat;
    lastBeat = now;

    bpm = 60000.0 / delta;  // bpm es float

    if (bpm >= 45 && bpm <= 110) {
      // GUARDAR CORRECTAMENTE (float)
      rates[rateSpot] = bpm;  // Guarda 74.3 como 74.3 (no 74)
      rateSpot = (rateSpot + 1) % RATE_SIZE;

      if (rateCount < RATE_SIZE)
        rateCount++;

      // CALCULAR AVG CORRECTO con floats
      float suma = 0.0;  // float, no int
      for (byte i = 0; i < rateCount; i++) {
        suma += rates[i];  // Suma floats
      }

      bpmAvg = suma / rateCount;  // División float
      
      // Mostrar con más precisión
      Serial.print("❤️ BPM: ");
      Serial.print(bpm, 1);  // 1 decimal
      Serial.print(" | AVG: ");
      Serial.println(bpmAvg, 1);  // 1 decimal
    }
  }

  // Mostrar estado cada 3 segundos (no cada latido)
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay >= 3000) {
    lastDisplay = millis();
    
    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(" | BPM=");
    Serial.print(bpm, 1);
    Serial.print(" | AVG=");
    
    if (rateCount >= 3) {
      Serial.println(bpmAvg, 1);
    } else {
      Serial.println("---");
    }
  }

  delay(10);
}