/*
 * Guarda-Rios Station WiFi UDP
 *
 * Reads turbidity and temperature data from STM32L053R8 via RS-485.
 * Samples every second and sends one authenticated binary UDP packet per
 * sample over the ESP32 onboard WiFi.
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <mbedtls/md.h>
#include <string.h>
#include <time.h>

#include "station_secrets.h"

/* RS-485 pins */
static const int RS485_RX = 4;
static const int RS485_TX = 5;
static const int RS485_DE = 22;
static const unsigned long RS485_BAUD = 9600;

/* UDP endpoint */
static const char *UDP_SERVER_IP = "172.233.121.41";
static const uint16_t UDP_SERVER_PORT = 40416;
static const uint16_t UDP_LOCAL_PORT = 5000;

/* Binary ingest protocol */
static const uint8_t PROTOCOL_VERSION = 1;
static const uint16_t STATION_ID = 1;
static const uint16_t SAMPLE_INTERVAL_S = 1;
static const uint8_t SAMPLE_COUNT = 1;
static const size_t AUTH_TAG_SIZE = 16;
static const char *NTP_SERVER = "pool.ntp.org";

static const int16_t MIN_TURBIDITY_CENTI_NTU = 0;
static const int16_t MAX_TURBIDITY_CENTI_NTU = 32767;
static const int16_t MIN_TEMPERATURE_CENTI_C = -500;
static const int16_t MAX_TEMPERATURE_CENTI_C = 5000;

static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 30000;
static const uint32_t WIFI_RETRY_INTERVAL_MS = 5000;
static const uint32_t UTC_SYNC_TIMEOUT_MS = 30000;
static const uint32_t SENSOR_FRAME_TIMEOUT_MS = 800;
static const uint32_t SAMPLE_INTERVAL_MS = (uint32_t)SAMPLE_INTERVAL_S * 1000UL;

HardwareSerial RS485Serial(1);
WiFiUDP Udp;

struct SensorSample {
  int16_t turbidityCentiNtu;
  int16_t temperatureCentiC;
};

static uint32_t gPacketCounter = 1;
static uint32_t gNextSampleAt = 0;
static uint32_t gLastWifiAttemptAt = 0;
static bool gUdpStarted = false;

static int16_t clampToInt16(int32_t value, int16_t minimum, int16_t maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return (int16_t)value;
}

static void flushRs485Input(void) {
  while (RS485Serial.available() > 0) {
    RS485Serial.read();
  }
}

static void appendU8(uint8_t *buffer, size_t bufferSize, size_t &offset, uint8_t value) {
  if (offset < bufferSize) {
    buffer[offset++] = value;
  }
}

static void appendU16BE(uint8_t *buffer, size_t bufferSize, size_t &offset, uint16_t value) {
  appendU8(buffer, bufferSize, offset, (uint8_t)((value >> 8) & 0xFF));
  appendU8(buffer, bufferSize, offset, (uint8_t)(value & 0xFF));
}

static void appendU32BE(uint8_t *buffer, size_t bufferSize, size_t &offset, uint32_t value) {
  appendU8(buffer, bufferSize, offset, (uint8_t)((value >> 24) & 0xFF));
  appendU8(buffer, bufferSize, offset, (uint8_t)((value >> 16) & 0xFF));
  appendU8(buffer, bufferSize, offset, (uint8_t)((value >> 8) & 0xFF));
  appendU8(buffer, bufferSize, offset, (uint8_t)(value & 0xFF));
}

static void appendI16BE(uint8_t *buffer, size_t bufferSize, size_t &offset, int16_t value) {
  appendU16BE(buffer, bufferSize, offset, (uint16_t)value);
}

static bool appendAuthTag(uint8_t *buffer, size_t bufferSize, size_t &offset) {
  if (offset + AUTH_TAG_SIZE > bufferSize) {
    return false;
  }

  const mbedtls_md_info_t *mdInfo = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (mdInfo == nullptr) {
    return false;
  }

  uint8_t fullDigest[32];
  const int result = mbedtls_md_hmac(mdInfo,
                                     (const unsigned char *)STATION_TOKEN,
                                     strlen(STATION_TOKEN),
                                     (const unsigned char *)buffer,
                                     offset,
                                     fullDigest);
  if (result != 0) {
    return false;
  }

  memcpy(buffer + offset, fullDigest, AUTH_TAG_SIZE);
  offset += AUTH_TAG_SIZE;
  return true;
}

static size_t buildPacket(uint8_t *buffer,
                          size_t bufferSize,
                          uint32_t startTimestampUtc,
                          const SensorSample *samples,
                          uint8_t sampleCount) {
  const size_t expectedSize =
      1 + 2 + 4 + 4 + 2 + 1 + ((size_t)sampleCount * sizeof(SensorSample)) + AUTH_TAG_SIZE;

  if (bufferSize < expectedSize) {
    return 0;
  }

  size_t offset = 0;

  appendU8(buffer, bufferSize, offset, PROTOCOL_VERSION);
  appendU16BE(buffer, bufferSize, offset, STATION_ID);
  appendU32BE(buffer, bufferSize, offset, gPacketCounter);
  appendU32BE(buffer, bufferSize, offset, startTimestampUtc);
  appendU16BE(buffer, bufferSize, offset, SAMPLE_INTERVAL_S);
  appendU8(buffer, bufferSize, offset, sampleCount);

  for (uint8_t index = 0; index < sampleCount; ++index) {
    appendI16BE(buffer, bufferSize, offset, samples[index].temperatureCentiC);
    appendI16BE(buffer, bufferSize, offset, samples[index].turbidityCentiNtu);
  }

  if (!appendAuthTag(buffer, bufferSize, offset)) {
    return 0;
  }

  return offset;
}

static void printHexDump(const uint8_t *buffer, size_t length) {
  Serial.println("Packet hex:");

  for (size_t index = 0; index < length; ++index) {
    if ((index % 16U) == 0U) {
      Serial.printf("%04u: ", (unsigned)index);
    }

    Serial.printf("%02X ", buffer[index]);

    if (((index % 16U) == 15U) || (index + 1U == length)) {
      Serial.println();
    }
  }
}

static bool ensureWifiConnected(uint32_t timeoutMs) {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  const uint32_t now = millis();
  if (gLastWifiAttemptAt != 0 && now - gLastWifiAttemptAt < WIFI_RETRY_INTERVAL_MS) {
    return false;
  }

  gLastWifiAttemptAt = now;
  Serial.printf("Connecting to WiFi SSID=%s\n", WIFI_SSID);

  WiFi.disconnect(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const uint32_t startedAt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startedAt < timeoutMs) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection timed out");
    return false;
  }

  Serial.printf("WiFi connected, IP=%s RSSI=%d dBm\n",
                WiFi.localIP().toString().c_str(),
                WiFi.RSSI());

  gUdpStarted = false;
  configTime(0, 0, NTP_SERVER);
  return true;
}

static bool ensureUdpStarted(void) {
  if (gUdpStarted) {
    return true;
  }

  if (!Udp.begin(UDP_LOCAL_PORT)) {
    Serial.printf("UDP local bind failed on port %u\n", (unsigned)UDP_LOCAL_PORT);
    return false;
  }

  gUdpStarted = true;
  Serial.printf("UDP local port=%u\n", (unsigned)UDP_LOCAL_PORT);
  return true;
}

static bool waitForUtcTime(uint32_t timeoutMs) {
  configTime(0, 0, NTP_SERVER);

  const uint32_t startedAt = millis();
  while (millis() - startedAt < timeoutMs) {
    const time_t now = time(nullptr);
    if (now > 1700000000) {
      Serial.printf("UTC time synchronized: %lu\n", (unsigned long)now);
      return true;
    }
    delay(500);
  }

  Serial.println("UTC time sync timed out");
  return false;
}

static bool getUtcTimestamp(uint32_t &utcEpoch) {
  const time_t now = time(nullptr);
  if (now <= 1700000000) {
    return false;
  }

  utcEpoch = (uint32_t)now;
  return true;
}

static bool sendBinaryPacketUdp(uint32_t startTimestampUtc,
                                const SensorSample *samples,
                                uint8_t sampleCount) {
  if (!ensureWifiConnected(1)) {
    Serial.println("UDP send skipped: WiFi is not connected");
    return false;
  }

  if (!ensureUdpStarted()) {
    return false;
  }

  uint8_t packet[64];
  const size_t packetLength =
      buildPacket(packet, sizeof(packet), startTimestampUtc, samples, sampleCount);
  if (packetLength == 0) {
    Serial.println("Packet build failed");
    return false;
  }

  Serial.printf("Sending packet counter=%lu samples=%u interval=%u s start=%lu\n",
                (unsigned long)gPacketCounter,
                (unsigned)sampleCount,
                (unsigned)SAMPLE_INTERVAL_S,
                (unsigned long)startTimestampUtc);
  printHexDump(packet, packetLength);

  if (!Udp.beginPacket(UDP_SERVER_IP, UDP_SERVER_PORT)) {
    Serial.println("UDP beginPacket failed");
    return false;
  }

  const size_t written = Udp.write(packet, packetLength);
  if (written != packetLength) {
    Serial.printf("UDP write failed: wrote %u/%u bytes\n", (unsigned)written, (unsigned)packetLength);
    Udp.endPacket();
    return false;
  }

  if (Udp.endPacket() != 1) {
    Serial.println("UDP endPacket failed");
    return false;
  }

  ++gPacketCounter;
  return true;
}

static bool readSensorFrame(SensorSample &sample, uint32_t timeoutMs) {
  const uint32_t startedAt = millis();

  while (millis() - startedAt < timeoutMs) {
    if (RS485Serial.available() <= 0) {
      delay(5);
      continue;
    }

    const uint8_t first = RS485Serial.read();
    if (first != 0xAA) {
      continue;
    }

    const uint32_t headerStartedAt = millis();
    while (RS485Serial.available() < 1 && millis() - headerStartedAt < 50) {
      delay(1);
    }

    if (RS485Serial.available() == 0) {
      Serial.println("RS-485 frame dropped: missing second header byte");
      continue;
    }

    if (RS485Serial.read() != 0x55) {
      Serial.println("RS-485 frame dropped: invalid second header byte");
      continue;
    }

    uint8_t data[5];
    if (RS485Serial.readBytes(data, sizeof(data)) != sizeof(data)) {
      Serial.println("RS-485 frame dropped: incomplete payload");
      continue;
    }

    const uint8_t checksum = data[0] ^ data[1] ^ data[2] ^ data[3];
    if (checksum != data[4]) {
      Serial.printf("RS-485 frame dropped: checksum expected=0x%02X got=0x%02X\n",
                    checksum,
                    data[4]);
      continue;
    }

    const uint16_t turbidityRaw = ((uint16_t)data[0] << 8) | data[1];
    const int16_t temperatureRaw = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    const int32_t turbidityCentiNtu = (int32_t)turbidityRaw * 100L;

    sample.turbidityCentiNtu =
        clampToInt16(turbidityCentiNtu, MIN_TURBIDITY_CENTI_NTU, MAX_TURBIDITY_CENTI_NTU);
    sample.temperatureCentiC =
        clampToInt16(temperatureRaw, MIN_TEMPERATURE_CENTI_C, MAX_TEMPERATURE_CENTI_C);

    if (sample.turbidityCentiNtu != turbidityCentiNtu) {
      Serial.printf("Turbidity clamped from %ld centi-NTU to %d centi-NTU\n",
                    (long)turbidityCentiNtu,
                    sample.turbidityCentiNtu);
    }
    if (sample.temperatureCentiC != temperatureRaw) {
      Serial.printf("Temperature clamped from %d centi-C to %d centi-C\n",
                    temperatureRaw,
                    sample.temperatureCentiC);
    }

    return true;
  }

  return false;
}

static void collectAndSendScheduledSample(void) {
  Serial.println("Sampling sensor");
  flushRs485Input();

  SensorSample sample;
  if (!readSensorFrame(sample, SENSOR_FRAME_TIMEOUT_MS)) {
    Serial.println("Sensor sample timeout, skipping this interval");
    return;
  }

  Serial.printf("Sample read: turbidity=%.2f NTU temperature=%.2f C\n",
                sample.turbidityCentiNtu / 100.0f,
                sample.temperatureCentiC / 100.0f);

  uint32_t timestampUtc = 0;
  if (!getUtcTimestamp(timestampUtc)) {
    Serial.println("UDP send skipped: UTC time is not synchronized");
    return;
  }

  if (sendBinaryPacketUdp(timestampUtc, &sample, SAMPLE_COUNT)) {
    Serial.println("PASS: binary packet sent over WiFi UDP");
  } else {
    Serial.println("FAIL: WiFi UDP binary packet send failed");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("=== Guarda-Rios Station WiFi UDP ===");
  Serial.printf("Sample interval=%u s packet sample count=%u\n",
                (unsigned)SAMPLE_INTERVAL_S,
                (unsigned)SAMPLE_COUNT);

  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);
  RS485Serial.begin(RS485_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  RS485Serial.setTimeout(100);
  Serial.printf("RS-485 RX=%d TX=%d DE=%d baud=%lu\n", RS485_RX, RS485_TX, RS485_DE, RS485_BAUD);

  if (ensureWifiConnected(WIFI_CONNECT_TIMEOUT_MS)) {
    ensureUdpStarted();
    waitForUtcTime(UTC_SYNC_TIMEOUT_MS);
  } else {
    Serial.println("WiFi initialization failed in setup, will retry before sends");
  }

  gNextSampleAt = millis();
}

void loop() {
  const uint32_t now = millis();
  if ((int32_t)(now - gNextSampleAt) >= 0) {
    gNextSampleAt += SAMPLE_INTERVAL_MS;
    collectAndSendScheduledSample();

    if ((int32_t)(millis() - gNextSampleAt) >= 0) {
      gNextSampleAt = millis() + SAMPLE_INTERVAL_MS;
      Serial.println("Sampling schedule slipped, next sample delayed by one interval");
    }
  }

  delay(20);
}
