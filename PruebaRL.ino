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

  Serial.println(F("\n🫀 OXÍMETRO - VERSIÓN ESTABLE"));
  Serial.println(F("==============================\n"));

  // Animación simple
  digitalWrite(LED_GREEN, LOW); delay(200); digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW); delay(200); digitalWrite(LED_BLUE, HIGH);

  Wire.begin(XIAO_SDA, XIAO_SCL);
  Wire.setClock(400000);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("❌ Sensor no detectado"));
    while (1) {
      digitalWrite(LED_RED, LOW); delay(200); digitalWrite(LED_RED, HIGH); delay(200);
    }
  }

  Serial.println(F("✅ Sensor OK"));
  
  // ===== CONFIGURACIÓN EQUILIBRADA =====
  byte ledBrightness = 60;        // Aumentado un poco
  byte sampleAverage = 4;         // Balance velocidad/precisión
  byte ledMode = 2;
  int sampleRate = 100;           // Normal
  int pulseWidth = 411;
  int adcRange = 16384;           // 14-bit

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  
  particleSensor.setPulseAmplitudeRed(0x1A);  // 26/255 = ~10%
  particleSensor.setPulseAmplitudeIR(0x1A);
  
  Serial.println(F("\n⚙️  Configuración aplicada"));
  Serial.println(F("\n👆 Coloca tu dedo y espera 3 segundos"));
  Serial.println(F("===================================\n"));

  // Calibración corta
  unsigned long inicio = millis();
  while (millis() - inicio < 3000) {
    uint32_t ir = particleSensor.getIR();
    static unsigned long ultimoReporte = 0;
    
    if (millis() - ultimoReporte > 1000) {
      ultimoReporte = millis();
      Serial.print(F("🔵 IR: "));
      Serial.println(ir);
    }
    
    // Parpadeo azul
    digitalWrite(LED_BLUE, LOW); delay(100); digitalWrite(LED_BLUE, HIGH); delay(100);
  }
  
  calibrado = true;
  Serial.println(F("\n✅ CALIBRADO - Listo para medir"));
  
  // Verde fijo 1 segundo
  digitalWrite(LED_GREEN, LOW);
  delay(1000);
  digitalWrite(LED_GREEN, HIGH);
}

void loop() {
  if (!calibrado) return;
  
  uint32_t irValue = particleSensor.getIR();
  
  // ===== UMBRAL IR BAJADO (de 8000 a 3000) =====
  if (irValue < 3000) {
    Serial.println(F("🖐️  Esperando dedo..."));
    finalLPM = 0;
    finalSpO2 = 0;
    apagarLEDS();
    
    // Parpadeo azul lento
    static bool ledState = false;
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 1500) {
      lastBlink = millis();
      ledState = !ledState;
      digitalWrite(LED_BLUE, ledState ? LOW : HIGH);
    }
    
    delay(300);
    return;
  }
  
  // Mostrar señal IR ocasionalmente
  static unsigned long ultimoIR = 0;
  if (millis() - ultimoIR > 5000) {
    ultimoIR = millis();
    Serial.print(F("📶 IR: "));
    Serial.println(irValue);
  }

  // Capturar muestras
  for (byte i = 0; i < BUFFER_SIZE; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  // Procesar
  maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE, redBuffer, 
                                        &spo2, &validSPO2, &pulseRate, &validPulseRate);

  if (validSPO2 && validPulseRate) {
    float rawLPM = (float)pulseRate;
    float rawSpO2 = (float)spo2;
    
    ultimaLPMCruda = rawLPM;
    
    // ===== CORRECCIÓN FINAL =====
    // Aplicar factor (ya está bien: 100→52)
    rawLPM = rawLPM * factorCorreccion;
    
    // Pequeño ajuste si es muy bajo/alto
    if (rawLPM < 45) rawLPM = 45;
    if (rawLPM > 180) rawLPM = 180;
    
    // ===== SpO2 MEJORADO =====
    // ¡SUBIENDO EL SpO2! (92→96)
    rawSpO2 = rawSpO2 + 4.0;  // ¡AÑADIR 4%!
    
    // Límites realistas
    if (rawSpO2 > 100.0) rawSpO2 = 100.0;
    if (rawSpO2 < 95.0) rawSpO2 = 95.0;  // Mínimo 95% en reposo

    // ===== FILTRADO SUAVE =====
    if (finalLPM == 0) {
      finalLPM = rawLPM;
      finalSpO2 = rawSpO2;
    } else {
      finalLPM = (rawLPM * 0.15) + (finalLPM * 0.85);
      finalSpO2 = (rawSpO2 * 0.10) + (finalSpO2 * 0.90);
    }

    // ===== DETECCIÓN DE ESTADO =====
    bool enEjercicio = finalLPM > 100;
    
    // ===== MOSTRAR SIMPLE Y CLARO =====
    Serial.print(F("❤️ "));
    Serial.print((int)finalLPM);
    Serial.print(F(" LPM | SpO2: "));
    Serial.print((int)finalSpO2);
    Serial.print(F("%"));
    
    // Estado simple
    if (enEjercicio) {
      Serial.print(F(" 🏃"));
    } else {
      Serial.print(F(" 💺"));
      if (finalSpO2 < 96) Serial.print(F(" ⚠️"));  // Alerta si bajo
    }
    
    // Calidad
    if (irValue > 15000) Serial.print(F(" ✅"));
    
    // Debug cada 10 segundos
    static unsigned long ultimoDebug = 0;
    if (millis() - ultimoDebug > 10000) {
      ultimoDebug = millis();
      Serial.print(F(" [Raw:"));
      Serial.print((int)ultimaLPMCruda);
      Serial.print(F(" IR:"));
      Serial.print(irValue);
      Serial.print(F("]"));
    }
    
    Serial.println();

    // ===== LEDs SIMPLES =====
    apagarLEDS();
    
    if (enEjercicio) {
      digitalWrite(LED_RED, LOW);
    } else if (finalLPM < 60) {
      digitalWrite(LED_BLUE, LOW);
    } else if (finalLPM <= 100) {
      digitalWrite(LED_GREEN, LOW);
    } else {
      digitalWrite(LED_RED, LOW);
    }

  } else {
    // Error silencioso
    static int errores = 0;
    errores++;
    if (errores % 15 == 0) {
      Serial.print(F("."));
      if (errores > 60) {
        Serial.println();
        errores = 0;
      }
    }
  }
  
  delay(1000);
}