#include <Arduino.h>
#include <WiFi.h>
#include "time.h"
#include "DHT.h"
#include "HX711.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h> // Librería LCD I2C

// ---------------- WiFi + NTP ----------------
const char* ssid     = "Edison L";
const char* password = "12345678";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -18000;
const int   daylightOffset_sec = 0;

// ---------------- Pines ----------------
#define DHTPIN 4
#define DHTTYPE DHT22
#define DT  18
#define SCK 19
#define ONE_WIRE_BUS 21
#define RELAY_PIN    5
#define SERVO_PIN    14
#define LED_PIN      2            // LED que indica que hay peso en la balanza
#define LED_RELAY_ON  25         // LED verde -> relay encendido
#define LED_RELAY_OFF 26         // LED rojo   -> relay apagado
#define BTN_SYSTEM 32   // Botón para sistema general (toggle + long-press cycle power mode)
#define BTN_BALANZA 33  // Botón para balanza (toggle)

// ---------------- Objetos sensores ----------------
DHT dht(DHTPIN, DHTTYPE);
HX711 balanza;
float factor_calibracion = -7050.0; // ajustar según la celda y montaje

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// ---------------- LCD I2C ----------------
// Nota: si tu dirección I2C no es 0x27 prueba 0x3F
LiquidCrystal_I2C lcd(0x27, 16, 2);
// Usaremos I2C con pines personalizados para evitar conflicto con ONE_WIRE_BUS (21)
// SDA = 23, SCL = 22
const int I2C_SDA_PIN = 23;
const int I2C_SCL_PIN = 22;

// ---------------- Variables globales ----------------
float sumTemp = 0, sumHum = 0;
int countSamples = 0;
float temperatureC = 0.0;
SemaphoreHandle_t mutex;

// Estado botones
volatile bool systemActive = false;
volatile bool balanzaActive = false;

// Servo
const int PWM_FREQ = 50;
const int PWM_RESOLUTION = 16;
const int PWM_CHANNEL = 0;
const float MIN_PULSE_US = 500.0;
const float MAX_PULSE_US = 2500.0;
const int OSCILLATION_ANGLE = 15;
const int STEP_DELAY_MS = 20;
const int STEP_ANGLE = 1;

volatile int currentAngle = 90;
volatile bool movingUp = true;

// ---------------- Power modes ----------------
enum PowerMode { NORMAL = 0, LOW_POWER = 1, SLEEP = 2 };
volatile PowerMode powerMode = NORMAL;

TaskHandle_t handleButtons = NULL;
TaskHandle_t handleReadDHT = NULL;
TaskHandle_t handleAvgPrint = NULL;
TaskHandle_t handleLeerBalanza = NULL;
TaskHandle_t handleReadDS18B20 = NULL;
TaskHandle_t handleRelayControl = NULL;
TaskHandle_t handleServoOsc = NULL;
TaskHandle_t handleSerialSend = NULL;

// ---------------- Communication/frame ----------------
uint32_t frameSeq = 0;
const int MAX_SEND_RETRIES = 3;
const int ACK_TIMEOUT_MS = 800; // esperar ACK por Serial

// ---------------- Utilidades ----------------
void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  char buf[64];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo); // timestamp ISO-like
  Serial.print(buf);
}

String getTimestampString() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char buf[64];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &timeinfo);
    return String(buf);
  } else {
    // si falla, usar millis
    char fallback[32];
    snprintf(fallback, sizeof(fallback), "uptime_ms:%lu", millis());
    return String(fallback);
  }
}

// checksum simple XOR de bytes
uint8_t computeChecksum(const uint8_t *data, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 0; i < len; ++i) cs ^= data[i];
  return cs;
}

// Arma una trama CSV-like:
// <SEQ>|<TIMESTAMP>|T_DHT:<val>|H_DHT:<val>|T_DS:<val>|PESO:<val>|CHK:<hex>\n
String buildFrame(float t_dht, float h_dht, float t_ds, float peso) {
  frameSeq++;
  String body = String(frameSeq) + "|" + getTimestampString();
  body += "|T_DHT:" + String(t_dht, 2);
  body += "|H_DHT:" + String(h_dht, 2);
  body += "|T_DS:" + String(t_ds, 2);
  body += "|PESO:" + String(peso, 2);

  // calcular checksum sobre body bytes
  const char *cstr = body.c_str();
  uint8_t cs = computeChecksum((const uint8_t*)cstr, strlen(cstr));
  char chkbuf[8];
  snprintf(chkbuf, sizeof(chkbuf), "%02X", cs);
  String frame = body + "|CHK:" + String(chkbuf) + "\n";
  return frame;
}

// Verifica checksum de una trama recibida (retorna true si OK)
bool validateFrame(const String &frame) {
  // buscar "|CHK:XX"
  int idx = frame.indexOf("|CHK:");
  if (idx < 0) return false;
  String body = frame.substring(0, idx);
  String chkStr = frame.substring(idx + 5); // desde XX hasta \n
  chkStr.trim();
  if (chkStr.length() < 2) return false;
  uint8_t provided = (uint8_t) strtoul(chkStr.c_str(), NULL, 16);
  uint8_t cs = computeChecksum((const uint8_t*)body.c_str(), body.length());
  return (cs == provided);
}

// Envía por Serial y espera ACK: si no, reintenta hasta MAX_SEND_RETRIES.
// ACK esperado de formato: "ACK:<SEQ>\n"
bool sendFrameWithAck(const String &frame, uint32_t seq) {
  for (int attempt = 1; attempt <= MAX_SEND_RETRIES; ++attempt) {
    Serial.print(frame); // enviar
    unsigned long start = millis();
    String resp = "";
    while (millis() - start < ACK_TIMEOUT_MS) {
      while (Serial.available()) {
        char c = (char)Serial.read();
        resp += c;
        if (c == '\n') break;
      }
      if (resp.length()) {
        resp.trim();
        // validar ACK
        String want = "ACK:" + String(seq);
        if (resp == want) {
          Serial.printf("[COMMS] ACK recibido seq=%lu (attempt %d)\n", seq, attempt);
          return true;
        } else {
          // si llegó otra cosa intentamos validar si es una respuesta de error
          Serial.printf("[COMMS] Resp inesperada: '%s'\n", resp.c_str());
          break;
        }
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    Serial.printf("[COMMS] No ACK seq=%lu, reintento %d/%d\n", seq, attempt, MAX_SEND_RETRIES);
    // pequeña espera antes de reintentar
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  Serial.printf("[COMMS] Falló envío seq=%lu después de %d intentos\n", seq, MAX_SEND_RETRIES);
  return false;
}

// ---------------- Funciones auxiliares de hardware ----------------
float angleToPulseUs(int angle) {
  float pulse = MIN_PULSE_US + ((float)angle / 180.0f) * (MAX_PULSE_US - MIN_PULSE_US);
  return pulse;
}

void setServoAngle(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  float pulseUs = angleToPulseUs(angle);
  const float periodUs = 1000000.0f / PWM_FREQ;
  uint32_t maxDuty = (1UL << PWM_RESOLUTION) - 1;
  uint32_t duty = (uint32_t)((pulseUs / periodUs) * (float)maxDuty);
  ledcWrite(PWM_CHANNEL, duty);
}

// ---------------- Botones: polling con detección de short/long press ----------------
unsigned long lastDebounce[40] = {0};
unsigned long btnDownTimestamp[40] = {0};
bool btnStateLast[40] = {false};

void processButtonEvent(int pin, bool pressed, bool longPress) {
  // pressed = true on falling edge (activo-bajo) event
  if (pin == BTN_SYSTEM) {
    if (longPress) {
      // ciclo modos: NORMAL -> LOW_POWER -> SLEEP -> NORMAL
      if (powerMode == NORMAL) powerMode = LOW_POWER;
      else if (powerMode == LOW_POWER) powerMode = SLEEP;
      else powerMode = NORMAL;

      // aplicar efectos del modo
      if (powerMode == LOW_POWER) {
        // pausar servo para ahorrar energía
        if (handleServoOsc) vTaskSuspend(handleServoOsc);
        Serial.println("[PWR] MODO LOW_POWER");
        lcd.noBacklight();
      } else if (powerMode == SLEEP) {
        // pausar servo y reducir muestreo (suspender tareas no esenciales)
        if (handleServoOsc) vTaskSuspend(handleServoOsc);
        Serial.println("[PWR] MODO SLEEP");
        lcd.noBacklight();
      } else {
        // NORMAL
        if (handleServoOsc) vTaskResume(handleServoOsc);
        Serial.println("[PWR] MODO NORMAL");
        lcd.backlight();
      }
    } else {
      // short press: toggle systemActive
      systemActive = !systemActive;
      Serial.printf("[BTN] SystemActive = %s\n", systemActive ? "ON":"OFF");
      if (!systemActive) {
        lcd.clear();
      } else {
        // si activamos, refrescar pantalla inmediatamente
        lcd.backlight();
      }
    }
  } else if (pin == BTN_BALANZA) {
    if (!longPress) {
      balanzaActive = !balanzaActive;
      Serial.printf("[BTN] BalanzaActive = %s\n", balanzaActive ? "ON":"OFF");
    } else {
      // long press on balanza: tare
      if (balanzaActive) {
        Serial.println("[BTN] Tare balanza por pulsacion larga");
        balanza.tare();
      }
    }
  }
}

void TaskButtons(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    // leer ambos botones (activo-bajo)
    int pins[] = { BTN_SYSTEM, BTN_BALANZA };
    for (int i = 0; i < 2; ++i) {
      int pin = pins[i];
      bool pressed = (digitalRead(pin) == LOW);
      unsigned long now = millis();
      if (pressed && !btnStateLast[pin]) {
        // transición HIGH->LOW, start timer
        btnDownTimestamp[pin] = now;
        btnStateLast[pin] = true;
      } else if (!pressed && btnStateLast[pin]) {
        // transición LOW->HIGH, release -> determine short/long
        unsigned long held = now - btnDownTimestamp[pin];
        bool longPress = (held >= 1500); // 1.5s -> long press
        processButtonEvent(pin, true, longPress);
        btnStateLast[pin] = false;
      }
      // else: steady state
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ---------------- Tareas sensores y control (respetando modos de potencia) ----------------
void TaskReadDHT(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    if (systemActive) {
      // ajustar periodo según modo de potencia
      int periodo = (powerMode == NORMAL) ? 2000 : (powerMode == LOW_POWER) ? 5000 : 30000;
      float h = dht.readHumidity();
      float t = dht.readTemperature();
      if (!isnan(h) && !isnan(t)) {
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
          sumTemp += t;
          sumHum  += h;
          countSamples++;
          xSemaphoreGive(mutex);
        }
        // Actualizar LCD con valores del DHT
        // Formato: línea1: "DHT T: XX.XC"
        //          línea2: "H: XX.X%   DS:YY.YC"
        lcd.setCursor(0, 0);
        lcd.printf("DHT T:%5.1fC H:%4.1f", t, h); // si tu librería no soporta printf, usa concatenación
        // Nota: LiquidCrystal_I2C con printf puede no existir en todas las implementaciones.
        // Para mayor compatibilidad:
        // char buf1[17]; snprintf(buf1,sizeof(buf1),"T:%5.1fC H:%4.1f",t,h); lcd.setCursor(0,0); lcd.print(buf1);
        // linea 2 con temp DS (global temperatureC)
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
          float ds = temperatureC;
          xSemaphoreGive(mutex);
          char buf2[17];
          snprintf(buf2, sizeof(buf2), "DS:%5.1fC           ", ds);
          lcd.setCursor(0, 1);
          lcd.print(buf2);
        }
      } else {
        Serial.println("[DHT] Lectura NAN");
      }
      vTaskDelay(pdMS_TO_TICKS(periodo));
    } else {
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
}

void TaskAvgPrint(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    // si en modo SLEEP, extendemos el tiempo entre envíos/impr
    int periodo = (powerMode == NORMAL) ? 15000 : (powerMode == LOW_POWER) ? 30000 : 60000;
    vTaskDelay(pdMS_TO_TICKS(periodo));
    if (systemActive) {
      float avgT = 0, avgH = 0;
      if (xSemaphoreTake(mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        if (countSamples > 0) {
          avgT = sumTemp / countSamples;
          avgH = sumHum  / countSamples;
          sumTemp = 0; sumHum = 0; countSamples = 0;
        }
        xSemaphoreGive(mutex);
      }
      Serial.println("----- Promedio -----");
      Serial.printf("Temp DHT22: %.2f °C\n", avgT);
      Serial.printf("Hum DHT22: %.2f %%\n", avgH);
      printLocalTime();
      Serial.println("--------------------");

      // construir trama y pasar a envíos (la tarea de envíos la procesa)
      // guardamos en variables temporales para que la tarea de envío las recoja
      // aquí directamente construimos y enviamos (puede bloquear un poco por ACK)
      String frame = buildFrame(avgT, avgH, temperatureC, 0.0);
      // extraer seq from frame for ACK check: it's the initial number up to '|'
      uint32_t seq = frame.substring(0, frame.indexOf('|')).toInt();
      bool ok = sendFrameWithAck(frame, seq);
      if (!ok) {
        Serial.println("[COMMS] Error al enviar frame (fallaron reintentos)");
      }
    }
  }
}

void TaskLeerBalanza(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    int periodo = (powerMode == NORMAL) ? 500 : (powerMode == LOW_POWER) ? 1000 : 5000;
    if (balanzaActive) {
      if (balanza.is_ready()) {
        float peso = balanza.get_units(20); 
        if (abs(peso) < 10) peso = 0; // umbral de ruido
        Serial.printf("[Balanza] Peso: %.2f (unidades de scale)\n", peso);
        digitalWrite(LED_PIN, peso > 0 ? HIGH : LOW);
        // enviar un mini-frame sobre peso inmediato (no bloqueante de ACK aquí)
        String body = String(frameSeq+1) + "|" + getTimestampString();
        body += "|PESO:" + String(peso,2);
        uint8_t cs = computeChecksum((const uint8_t*)body.c_str(), body.length());
        char chkbuf[8]; snprintf(chkbuf, sizeof(chkbuf), "%02X", cs);
        String frame = body + "|CHK:" + String(chkbuf) + "\n";
        Serial.print(frame); // fire-and-forget para peso (no ACK)
      } else {
        Serial.println("[Balanza] No lista");
      }
    } else {
      digitalWrite(LED_PIN, LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(periodo));
  }
}

void TaskReadDS18B20(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    int periodo = (powerMode == NORMAL) ? 2000 : (powerMode == LOW_POWER) ? 5000 : 30000;
    if (systemActive) {
      sensors.requestTemperatures();
      float t = sensors.getTempCByIndex(0);
      if (t != DEVICE_DISCONNECTED_C) {
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
          temperatureC = t;
          xSemaphoreGive(mutex);
        }
        Serial.printf("[DS18B20] Temp: %.2f °C\n", t);
      } else {
        Serial.println("[DS18B20] Sensor desconectado");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(periodo));
  }
}

void TaskRelayControl(void *pvParameters) {
  (void) pvParameters;
  digitalWrite(RELAY_PIN, LOW);
  bool releEncendido = false;
  digitalWrite(LED_RELAY_ON, LOW);
  digitalWrite(LED_RELAY_OFF, HIGH);

  for (;;) {
    if (systemActive) {
      float tempLocal = 0.0;
      if (xSemaphoreTake(mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        tempLocal = temperatureC;
        xSemaphoreGive(mutex);
      }
      // hysteresis: encender a >=40, apagar a <=35
      if (tempLocal >= 40.0 && !releEncendido) {
        digitalWrite(RELAY_PIN, HIGH);
        releEncendido = true;
        digitalWrite(LED_RELAY_ON, HIGH);
        digitalWrite(LED_RELAY_OFF, LOW);
        Serial.println(">>> Ventilador ENCENDIDO");
      } else if (tempLocal <= 35.0 && releEncendido) {
        digitalWrite(RELAY_PIN, LOW);
        releEncendido = false;
        digitalWrite(LED_RELAY_ON, LOW);
        digitalWrite(LED_RELAY_OFF, HIGH);
        Serial.println(">>> Ventilador APAGADO");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void TaskServoOscillation(void *pvParameters) {
  (void) pvParameters;
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(SERVO_PIN, PWM_CHANNEL);
  currentAngle = 90;
  setServoAngle(currentAngle);

  for (;;) {
    // si estamos en modos de baja potencia y la tarea está suspendida, no llegamos aquí
    if (systemActive && powerMode == NORMAL) {
      if (movingUp) {
        currentAngle += STEP_ANGLE;
        if (currentAngle >= 90 + OSCILLATION_ANGLE) {
          currentAngle = 90 + OSCILLATION_ANGLE;
          movingUp = false;
        }
      } else {
        currentAngle -= STEP_ANGLE;
        if (currentAngle <= 90 - OSCILLATION_ANGLE) {
          currentAngle = 90 - OSCILLATION_ANGLE;
          movingUp = true;
        }
      }
      setServoAngle(currentAngle);
      vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
    } else {
      vTaskDelay(pdMS_TO_TICKS(200)); // idle wait
    }
  }
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Serial.println("Sistema iniciado");

  // Pines básicos
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(LED_RELAY_ON, OUTPUT);
  pinMode(LED_RELAY_OFF, OUTPUT);
  digitalWrite(LED_RELAY_ON, LOW);
  digitalWrite(LED_RELAY_OFF, HIGH);

  pinMode(BTN_SYSTEM, INPUT_PULLUP);
  pinMode(BTN_BALANZA, INPUT_PULLUP);

  // Inicializar I2C con pines personalizados (para no usar el 21 que usa DS18B20)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  // Inicializar LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Sistema iniciado");

  // Conectar WiFi brevemente para obtener hora NTP
  WiFi.begin(ssid, password);
  Serial.print("Conectando WiFi");
  unsigned long startConnect = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startConnect) < 10000) {
    delay(250);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  } else {
    Serial.println("\nNo se pudo conectar a WiFi (timeout).");
  }

  // Sensores / Actuadores
  dht.begin();
  balanza.begin(DT, SCK);
  balanza.set_scale(factor_calibracion);
  balanza.tare();
  sensors.begin();
  sensors.setResolution(12);

  mutex = xSemaphoreCreateMutex();

  // Crear tareas y guardar handles
  xTaskCreate(TaskButtons, "Buttons", 4096, NULL, 3, &handleButtons);
  xTaskCreate(TaskReadDHT, "ReadDHT", 4096, NULL, 1, &handleReadDHT);
  xTaskCreate(TaskAvgPrint, "AvgPrint", 4096, NULL, 1, &handleAvgPrint);
  xTaskCreate(TaskLeerBalanza, "LeerBalanza", 4096, NULL, 1, &handleLeerBalanza);
  xTaskCreate(TaskReadDS18B20, "ReadDS18B20", 4096, NULL, 1, &handleReadDS18B20);
  xTaskCreate(TaskRelayControl, "RelayCtrl", 2048, NULL, 1, &handleRelayControl);
  xTaskCreate(TaskServoOscillation, "ServoOsc", 4096, NULL, 1, &handleServoOsc);
  // No hay tarea separada para SerialSend; AvgPrint hace el envío con ACK.

  // mostrar modo inicial
  Serial.println("[PWR] MODO NORMAL");
  lcd.setCursor(0,1);
  lcd.print("MODO: NORMAL     ");
}

void loop() {
  // FreeRTOS se encarga de todo
}
