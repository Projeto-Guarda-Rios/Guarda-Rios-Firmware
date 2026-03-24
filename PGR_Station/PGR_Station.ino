/*
 * Guardarios Station
 *
 * Reads turbidity and temperature data from STM32L053R8 via RS-485
 * and sends it to the Guarda-Rios API over WiFi.
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>

/* WiFi credentials */
const char *WIFI_SSID     = "";
const char *WIFI_PASSWORD = "";

/* API config */
const char *API_URL = "https://data.guarda-rios.pt/api/ingest";
const char *API_KEY = "0c28df1e8e725575a13b74db8989cb01374342979ff712e3aaf561e6bc2fd0d6";

/* RS-485 pins */
#define RS485_RX     4
#define RS485_TX     5
#define RS485_DE     22
#define RS485_BAUD   9600

HardwareSerial RS485Serial(1);

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== Guardarios Station ===");

  /* RS-485 setup */
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW); /* Receive mode */
  RS485Serial.begin(RS485_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  RS485Serial.setTimeout(100);

  /* WiFi setup */
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConnected — IP: %s\n", WiFi.localIP().toString().c_str());
}

void sendToAPI(float temperature, float turbidity) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, skipping send");
    return;
  }

  HTTPClient http;
  http.begin(API_URL);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-API-Key", API_KEY);

  char json[128];
  snprintf(json, sizeof(json),
           "{\"temperature\":%.2f,\"turbidity\":%.2f}",
           temperature, turbidity);

  int code = http.POST((uint8_t *)json, strlen(json));

  if (code == 201) {
    Serial.printf("API OK: temp=%.2f°C turb=%.2f NTU\n", temperature, turbidity);
  } else if (code > 0) {
    Serial.printf("API error %d: %s\n", code, http.getString().c_str());
  } else {
    Serial.printf("HTTP request failed: %s\n", http.errorToString(code).c_str());
  }

  http.end();
}

void loop() {
  if (RS485Serial.available() > 0) {
    uint8_t b = RS485Serial.read();

    if (b != 0xAA) return;

    uint32_t t0 = millis();
    while (RS485Serial.available() < 1 && millis() - t0 < 50);

    if (RS485Serial.available() == 0) return;
    if (RS485Serial.read() != 0x55) return;

    uint8_t d[5];
    if (RS485Serial.readBytes(d, 5) != 5) return;

    uint8_t chk = d[0] ^ d[1] ^ d[2] ^ d[3];
    if (chk != d[4]) return;

    uint16_t turb_raw = ((uint16_t)d[0] << 8) | d[1];
    int16_t  temp_raw = (int16_t)(((uint16_t)d[2] << 8) | d[3]);

    float temperature = temp_raw / 100.0f;
    float turbidity   = (float)turb_raw;

    Serial.printf("Turbidity: %u NTU | Temperature: %.2f C\n",
                  turb_raw, temperature);

    sendToAPI(temperature, turbidity);
  }
}