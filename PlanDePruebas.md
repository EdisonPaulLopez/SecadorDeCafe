1

Lectura de sensor DHT22 y promedio correcto

Acción: Dejar corriendo el ESP32 en condiciones normales, sin pulsar botones.

Esperado:

Cada ~15s se debe imprimir en Serial el bloque de promedio de temperatura y humedad.

En LCD debe aparecer la línea superior con temperatura y humedad del DHT, y la inferior con la lectura del DS18B20.

Formato serial incluye timestamp ISO y valores.

2

Lectura de sensor DS18B20

Acción: Observar valores en Serial y LCD. Simular cambio de temperatura acercando calor/frío al sensor.

Esperado:

En Serial aparecen logs [DS18B20] Temp: XX.XX °C.

En LCD (segunda línea) debe actualizarse DS:XX.XC.

Si sensor desconectado → en Serial aparece [DS18B20] Sensor desconectado.

3

Lectura de balanza HX711

Acción: Activar balanza con botón BTN_BALANZA. Colocar peso (ej. objetos conocidos de 100g, 200g, etc.).

Esperado:

En Serial aparece [Balanza] Peso: XX.XX.

LED_PIN se enciende mientras haya peso > 0.

Cada lectura genera mini-tramas con checksum en Serial.

4

Botón Sistema (BTN_SYSTEM) - Short press

Acción: Pulsar brevemente BTN_SYSTEM.

Esperado:

Cambia systemActive ON/OFF.

En ON → LCD se activa con backlight y valores de sensores.

En OFF → LCD se limpia y se apaga backlight.

En Serial aparece [BTN] SystemActive = ON/OFF.

5

Botón Sistema (BTN_SYSTEM) - Long press (ciclo modos de energía)

Acción: Mantener pulsado BTN_SYSTEM > 1.5s varias veces seguidas.

Esperado:

Ciclo entre NORMAL → LOW_POWER → SLEEP → NORMAL.

Logs Serial: [PWR] MODO LOW_POWER / SLEEP / NORMAL.

En LOW_POWER y SLEEP el backlight se apaga.

En NORMAL vuelve a encenderse y se reanuda el servo.

6

Botón Balanza (BTN_BALANZA) - Short/Long press

Acción:

Pulsar corto → alterna ON/OFF de la balanza.

Pulsar largo (>1.5s) mientras activa → ejecuta tare().

Esperado:

Logs Serial: [BTN] BalanzaActive = ON/OFF.

Si long press → [BTN] Tare balanza por pulsacion larga.

Peso vuelve a 0 tras tara.

7

Control del relay y LEDs de estado

Acción: Simular variación de temperatura con calor/frío al DS18B20.

Esperado:

Si temperatura ≥ 40 °C → Relay ON, LED verde ON, LED rojo OFF.

Si temperatura ≤ 35 °C → Relay OFF, LED verde OFF, LED rojo ON.

Logs Serial: >>> Ventilador ENCENDIDO / APAGADO.

8

Servo oscilante

Acción: Observar movimiento del servo conectado al pin 14.

Esperado:

En modo NORMAL y sistema activo → servo oscila ±15° alrededor de 90°.

En LOW_POWER o SLEEP → servo se detiene (tarea suspendida).

9

Comunicación y tramas con checksum

Acción:

Observar en Serial las tramas generadas cada 15s (buildFrame).

Enviar un ACK válido desde PC: ACK:<SEQ> por puerto serie.

Esperado:

Con ACK → log [COMMS] ACK recibido seq=....

Sin ACK → hasta 3 intentos, luego [COMMS] Falló envío seq=....

Si checksum incorrecto → frame inválido al validar.

10

Sincronización de RTC con WiFi/NTP

Acción: Encender ESP32 con WiFi disponible.

Esperado:

En setup aparece: WiFi conectado.

Luego printLocalTime() muestra fecha/hora NTP correcta.

WiFi se desconecta y apaga tras sincronización.

Posteriores timestamps usan RTC local.