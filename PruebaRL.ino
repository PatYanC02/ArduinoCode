#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#define XIAO_SDA 6
#define XIAO_SCL 7
#define BUFFER_SIZE 100

#define LED_RED 20
#define LED_GREEN 21
#define LED_BLUE 22

uint32_t irBuffer[BUFFER_SIZE]; 
uint32_t redBuffer[BUFFER_SIZE];

int32_t spo2;
int8_t validSPO2;
int32_t pulseRate;
int8_t validPulseRate;

// ===== VARIABLES OPTIMIZADAS =====
float finalLPM = 0;
float finalSpO2 = 0;
bool calibrado = false;
float factorCorreccion = 0.52;  // Ajustado: 100→52 (0.52) está bien
float ultimaLPMCruda = 0;

void apagarLEDS() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  apagarLEDS();

  Serial.println(F("\n OXÍMETRO - MODO PRUEBAS (ESP32-C3)"));
  Serial.println(F("=====================================\n"));

  // Arranque visual corto
  digitalWrite(LED_GREEN, LOW); delay(150); digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW);  delay(150); digitalWrite(LED_BLUE, HIGH);

  // I2C
  Wire.begin(XIAO_SDA, XIAO_SCL);
  Wire.setClock(400000); // Fast mode OK

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F(" Sensor no detectado"));
    while (1) {
      digitalWrite(LED_RED, LOW); delay(250);
      digitalWrite(LED_RED, HIGH); delay(250);
    }
  }

  Serial.println(F("Sensor OK"));

  // ===== CONFIGURACIÓN LIGERA =====
  byte ledBrightness = 40;     // Menos corriente
  byte sampleAverage = 2;      // Menos carga interna
  byte ledMode = 2;            // RED + IR (SpO2)
  int  sampleRate = 50;        // 50 Hz es suficiente
  int  pulseWidth = 215;       // Menor resolución, menos carga
  int  adcRange = 8192;        // 13-bit efectivo

  particleSensor.setup(
    ledBrightness,
    sampleAverage,
    ledMode,
    sampleRate,
    pulseWidth,
    adcRange
  );

  // Amplitud LED moderada
  particleSensor.setPulseAmplitudeRed(0x18); // ~9%
  particleSensor.setPulseAmplitudeIR(0x18);

  Serial.println(F("\n Configuración ligera aplicada"));
  Serial.println(F("👆 Coloca tu dedo (calibración corta)\n"));

  // ===== CALIBRACIÓN MUY CORTA =====
  unsigned long inicio = millis();
  while (millis() - inicio < 2000) {   // 2 segundos
    uint32_t ir = particleSensor.getIR();

    static unsigned long ultimoReporte = 0;
    if (millis() - ultimoReporte > 1000) {
      ultimoReporte = millis();
      Serial.print(F("IR: "));
      Serial.println(ir);
    }

    digitalWrite(LED_BLUE, LOW);
    delay(80);
    digitalWrite(LED_BLUE, HIGH);
    delay(80);
  }

  calibrado = true;

  Serial.println(F("\n LISTO (modo pruebas)"));

  digitalWrite(LED_GREEN, LOW);
  delay(600);
  digitalWrite(LED_GREEN, HIGH);
}
void loop() {
  if (!calibrado) return;

  static unsigned long lastProcess = 0;
  static bool esperandoDedo = true;

  uint32_t irValue = particleSensor.getIR();

  // ===== DETECCIÓN DE DEDO =====
  if (irValue < 3000) {
    if (!esperandoDedo) {
      esperandoDedo = true;
      finalLPM = 0;
      finalSpO2 = 0;
      apagarLEDS();
      Serial.println(F(" Esperando dedo..."));
    }

    // Parpadeo azul lento (no bloqueante)
    static bool ledState = false;
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 1500) {
      lastBlink = millis();
      ledState = !ledState;
      digitalWrite(LED_BLUE, ledState ? LOW : HIGH);
    }

    delay(20);   // Cede CPU (RTOS friendly)
    return;
  }

  // Si el dedo acaba de colocarse
  if (esperandoDedo) {
    esperandoDedo = false;
    apagarLEDS();
    Serial.println(F(" Dedo detectado"));
  }

  // ===== PROCESAR SOLO CADA ~1.2s =====
  if (millis() - lastProcess < 1200) {
    delay(5);
    return;
  }
  lastProcess = millis();

  // ===== CAPTURA DE MUESTRAS =====
  for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
    unsigned long t0 = millis();
    while (!particleSensor.available()) {
      particleSensor.check();
      delay(1);  // 🔴 CRÍTICO para ESP32
      if (millis() - t0 > 20) break; // evita bloqueo eterno
    }

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }

  // ===== PROCESAMIENTO MAXIM =====
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer,
    BUFFER_SIZE,
    redBuffer,
    &spo2,
    &validSPO2,
    &pulseRate,
    &validPulseRate
  );

  if (!(validSPO2 && validPulseRate)) {
    static uint8_t errores = 0;
    if (++errores % 10 == 0) Serial.print(F("."));
    return;
  }

  // ===== POST-PROCESADO LIGERO =====
  float rawLPM  = pulseRate * factorCorreccion;
  float rawSpO2 = spo2 + 4.0;

  // Límites
  rawLPM  = constrain(rawLPM, 45, 180);
  rawSpO2 = constrain(rawSpO2, 95, 100);

  ultimaLPMCruda = rawLPM;

  // EMA (mantener)
  if (finalLPM == 0) {
    finalLPM  = rawLPM;
    finalSpO2 = rawSpO2;
  } else {
    finalLPM  = rawLPM  * 0.15 + finalLPM  * 0.85;
    finalSpO2 = rawSpO2 * 0.10 + finalSpO2 * 0.90;
  }

  // ===== ESTADO =====
  bool enEjercicio = finalLPM > 100;

  // ===== SALIDA SERIAL (LIGERA) =====
  Serial.print(F(" "));
  Serial.print((int)finalLPM);
  Serial.print(F(" LPM | "));
  Serial.print((int)finalSpO2);
  Serial.print(F("%"));

  if (enEjercicio) Serial.print(F(" RUN"));
  else Serial.print(F(" REST"));

  if (irValue > 15000) Serial.print(F(" OK"));

  Serial.println();

  // ===== LEDs =====
  apagarLEDS();
  if (enEjercicio) {
    digitalWrite(LED_RED, LOW);
  } else if (finalLPM < 60) {
    digitalWrite(LED_BLUE, LOW);
  } else {
    digitalWrite(LED_GREEN, LOW);
  }
}
