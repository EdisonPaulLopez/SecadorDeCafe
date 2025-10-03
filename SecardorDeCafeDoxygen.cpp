/**
 * @file main.cpp
 * @brief Sistema IoT con sensores (DHT22, DS18B20, HX711), LCD I2C, control de servo y relé.
 * @details Este programa corre sobre un ESP32 utilizando FreeRTOS.
 *  - Obtiene lecturas de temperatura y humedad (DHT22).
 *  - Lee temperatura del DS18B20.
 *  - Lee peso desde celda de carga con HX711.
 *  - Controla un relé con histéresis según temperatura.
 *  - Oscila un servo entre ángulos.
 *  - Muestra datos en un LCD I2C.
 *  - Envía tramas con checksum por Serial y espera ACK.
 *  - Incluye modos de ahorro de energía (NORMAL, LOW_POWER, SLEEP).
 *
 * @author Edison Lopezs
 * @date 2025
 */

#include <Arduino.h>
#include <WiFi.h>
#include "time.h"
#include "DHT.h"
#include "HX711.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ---------------- WiFi + NTP ----------------
/// SSID de la red WiFi
const char* ssid     = "Edison L";
/// Contraseña de la red WiFi
const char* password = "12345678";

/// Servidor NTP
const char* ntpServer = "pool.ntp.org";
/// Offset de zona horaria en segundos
const long  gmtOffset_sec = -18000;
/// Offset de horario de verano en segundos
const int   daylightOffset_sec = 0;

// ---------------- Pines ----------------
#define DHTPIN 4      ///< Pin del sensor DHT22
#define DHTTYPE DHT22 ///< Tipo de sensor DHT
#define DT  18        ///< Pin DT para HX711
#define SCK 19        ///< Pin SCK para HX711
#define ONE_WIRE_BUS 21 ///< Pin para bus OneWire DS18B20
#define RELAY_PIN    5 ///< Pin de salida para relé
#define SERVO_PIN    14 ///< Pin PWM para servo
#define LED_PIN      2 ///< LED que indica peso en balanza
#define LED_RELAY_ON  25 ///< LED verde (relay encendido)
#define LED_RELAY_OFF 26 ///< LED rojo (relay apagado)
#define BTN_SYSTEM 32   ///< Botón para activar sistema (toggle + long press para modos)
#define BTN_BALANZA 33  ///< Botón para activar balanza (toggle + long press para tare)

// ---------------- Objetos sensores ----------------
DHT dht(DHTPIN, DHTTYPE); ///< Objeto sensor DHT22
HX711 balanza; ///< Objeto balanza HX711
float factor_calibracion = -7050.0; ///< Factor de calibración de la balanza

OneWire oneWire(ONE_WIRE_BUS); ///< Objeto OneWire
DallasTemperature sensors(&oneWire); ///< Objeto DS18B20

// ---------------- LCD I2C ----------------
/// Objeto LCD I2C (dirección 0x27, 16x2)
LiquidCrystal_I2C lcd(0x27, 16, 2);
/// Pines personalizados I2C (SDA=23, SCL=22)
const int I2C_SDA_PIN = 23;
const int I2C_SCL_PIN = 22;

// ---------------- Variables globales ----------------
float sumTemp = 0, sumHum = 0; ///< Acumuladores para promedios
int countSamples = 0; ///< Contador de muestras
float temperatureC = 0.0; ///< Temperatura del DS18B20
SemaphoreHandle_t mutex; ///< Mutex para variables compartidas

/// Estado del sistema general
volatile bool systemActive = false;
/// Estado de la balanza
volatile bool balanzaActive = false;

// Servo
const int PWM_FREQ = 50; ///< Frecuencia PWM para servo
const int PWM_RESOLUTION = 16; ///< Resolución PWM
const int PWM_CHANNEL = 0; ///< Canal PWM
const float MIN_PULSE_US = 500.0; ///< Pulso mínimo en us
const float MAX_PULSE_US = 2500.0; ///< Pulso máximo en us
const int OSCILLATION_ANGLE = 15; ///< Ángulo de oscilación
const int STEP_DELAY_MS = 20; ///< Retardo entre pasos en ms
const int STEP_ANGLE = 1; ///< Incremento de ángulo

volatile int currentAngle = 90; ///< Ángulo actual del servo
volatile bool movingUp = true; ///< Dirección de movimiento del servo

// ---------------- Power modes ----------------
/**
 * @enum PowerMode
 * @brief Modos de energía del sistema.
 */
enum PowerMode { NORMAL = 0, LOW_POWER = 1, SLEEP = 2 };
volatile PowerMode powerMode = NORMAL; ///< Modo actual de energía

// Handles de tareas
TaskHandle_t handleButtons = NULL;
TaskHandle_t handleReadDHT = NULL;
TaskHandle_t handleAvgPrint = NULL;
TaskHandle_t handleLeerBalanza = NULL;
TaskHandle_t handleReadDS18B20 = NULL;
TaskHandle_t handleRelayControl = NULL;
TaskHandle_t handleServoOsc = NULL;
TaskHandle_t handleSerialSend = NULL;

// ---------------- Comunicación ----------------
uint32_t frameSeq = 0; ///< Secuencia de tramas
const int MAX_SEND_RETRIES = 3; ///< Máx. reintentos de envío
const int ACK_TIMEOUT_MS = 800; ///< Timeout para ACK

/**
 * @brief Imprime la hora local obtenida por NTP.
 */
void printLocalTime();

/**
 * @brief Genera un string timestamp en formato ISO.
 * @return Cadena con timestamp.
 */
String getTimestampString();

/**
 * @brief Calcula el checksum XOR de un buffer.
 * @param data Puntero a datos.
 * @param len Longitud de datos.
 * @return Checksum XOR.
 */
uint8_t computeChecksum(const uint8_t *data, size_t len);

/**
 * @brief Construye una trama de datos CSV-like con checksum.
 * @param t_dht Temperatura DHT22.
 * @param h_dht Humedad DHT22.
 * @param t_ds Temperatura DS18B20.
 * @param peso Peso en balanza.
 * @return Trama construida.
 */
String buildFrame(float t_dht, float h_dht, float t_ds, float peso);

/**
 * @brief Valida el checksum de una trama recibida.
 * @param frame Trama recibida.
 * @return true si es válida, false en caso contrario.
 */
bool validateFrame(const String &frame);

/**
 * @brief Envía trama por Serial y espera ACK.
 * @param frame Trama a enviar.
 * @param seq Número de secuencia.
 * @return true si se recibe ACK válido, false en caso contrario.
 */
bool sendFrameWithAck(const String &frame, uint32_t seq);

/**
 * @brief Convierte ángulo a ancho de pulso en us.
 * @param angle Ángulo del servo (0-180).
 * @return Ancho de pulso en us.
 */
float angleToPulseUs(int angle);

/**
 * @brief Configura el ángulo del servo.
 * @param angle Ángulo deseado.
 */
void setServoAngle(int angle);

/**
 * @brief Procesa eventos de botones (short y long press).
 * @param pin Pin del botón.
 * @param pressed True si fue presionado.
 * @param longPress True si fue pulsación larga.
 */
void processButtonEvent(int pin, bool pressed, bool longPress);

// -------- Tareas (FreeRTOS) --------
/**
 * @brief Tarea de gestión de botones.
 */
void TaskButtons(void *pvParameters);

/**
 * @brief Tarea de lectura de DHT22 y actualización de LCD.
 */
void TaskReadDHT(void *pvParameters);

/**
 * @brief Tarea de cálculo de promedios y envío de datos.
 */
void TaskAvgPrint(void *pvParameters);

/**
 * @brief Tarea de lectura de balanza HX711.
 */
void TaskLeerBalanza(void *pvParameters);

/**
 * @brief Tarea de lectura de DS18B20.
 */
void TaskReadDS18B20(void *pvParameters);

/**
 * @brief Tarea de control del relé con histéresis.
 */
void TaskRelayControl(void *pvParameters);

/**
 * @brief Tarea de oscilación del servo.
 */
void TaskServoOscillation(void *pvParameters);

/**
 * @brief Configuración inicial del sistema.
 */
void setup();

/**
 * @brief Bucle principal (no se usa, FreeRTOS maneja tareas).
 */
void loop();

