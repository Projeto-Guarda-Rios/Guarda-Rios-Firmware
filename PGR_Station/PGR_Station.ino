/*
 * Guarda-Rios Station
 *
 * Reads turbidity and temperature data from STM32L053R8 via RS-485.
 * Wakes on a timer, reads the AquaNode over RS-485, sends authenticated
 * binary UDP data over SIM7028 NB-IoT, then returns to ESP32 deep sleep.
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <mbedtls/md.h>
#include <string.h>

#include "station_secrets.h"

/* SIM7028 UART pins */
static const int MODEM_RX_PIN = 16;
static const int MODEM_TX_PIN = 17;
static const unsigned long MODEM_BAUD = 115200;

/* RS-485 pins */
static const int RS485_RX = 4;
static const int RS485_TX = 5;
static const int RS485_DE = 22;
static const unsigned long RS485_BAUD = 9600;

/* Status LED */
static const int IO18_LED_PIN = 18;

/* UDP endpoint */
static const char *UDP_SERVER_IP = "172.233.121.41";
static const uint16_t UDP_SERVER_PORT = 40416;
static const uint16_t UDP_LOCAL_PORT = 5000;

/* Working network profile for this SIM/operator combination. */
static const uint8_t NB_MODE = 0;
static const uint8_t NB_BAND = 20;

/* Binary ingest protocol */
static const uint8_t PROTOCOL_VERSION = 1;
static const uint16_t STATION_ID = 1;
static const uint16_t SAMPLE_INTERVAL_S = 300;
static const uint8_t SAMPLE_COUNT = 12;
static const int32_t TIMESTAMP_OFFSET_S = 0;
static const size_t AUTH_TAG_SIZE = 16;
static const char *NTP_SERVER = "pool.ntp.org";

static const int16_t MIN_TURBIDITY_CENTI_NTU = 0;
static const int16_t MAX_TURBIDITY_CENTI_NTU = 32767;
static const int16_t MIN_TEMPERATURE_CENTI_C = -500;
static const int16_t MAX_TEMPERATURE_CENTI_C = 5000;

static const uint32_t AT_TIMEOUT_MS = 3000;
static const uint32_t SIM_READY_TIMEOUT_MS = 30000;
static const uint32_t NETWORK_REG_TIMEOUT_MS = 300000;
static const uint32_t NETOPEN_TIMEOUT_MS = 300000;
static const uint32_t CIPOPEN_TIMEOUT_MS = 30000;
static const uint32_t CIPSEND_PROMPT_TIMEOUT_MS = 5000;
static const uint32_t CIPSEND_RESULT_TIMEOUT_MS = 30000;
static const uint32_t CCLK_TIMEOUT_MS = 3000;
static const uint32_t CNTP_TIMEOUT_MS = 10000;
static const uint32_t UTC_SYNC_TIMEOUT_MS = 30000;
static const uint32_t QUIET_GAP_MS = 200;
static const uint32_t SENSOR_FRAME_TIMEOUT_MS = 10000;
static const uint64_t SLEEP_INTERVAL_US = (uint64_t)SAMPLE_INTERVAL_S * 1000000ULL;
static const uint32_t RETAINED_STATE_MAGIC = 0x47525331UL;  // "GRS1"

HardwareSerial RS485Serial(1);
HardwareSerial ModemSerial(2);

struct SensorSample {
  int16_t turbidityCentiNtu;
  int16_t temperatureCentiC;
};

struct RetainedState {
  uint32_t magic;
  SensorSample samples[SAMPLE_COUNT];
  uint8_t sampleCount;
  uint32_t packetCounter;
};

/* RTC slow memory survives timer deep-sleep resets, but not power loss. */
RTC_DATA_ATTR static RetainedState gRetained;
static bool gModemReady = false;
static bool gIo18LedOn = false;

static bool sendAT(const char *command,
                   const char *expectA,
                   const char *expectB = nullptr,
                   uint32_t timeoutMs = AT_TIMEOUT_MS);

static void initializeRetainedState(void) {
  if (gRetained.magic == RETAINED_STATE_MAGIC && gRetained.sampleCount <= SAMPLE_COUNT &&
      gRetained.packetCounter != 0) {
    return;
  }

  memset(&gRetained, 0, sizeof(gRetained));
  gRetained.magic = RETAINED_STATE_MAGIC;
  gRetained.packetCounter = 1;
}

static void toggleIo18Led(void) {
  gIo18LedOn = !gIo18LedOn;
  digitalWrite(IO18_LED_PIN, gIo18LedOn ? HIGH : LOW);
}

static int16_t clampToInt16(int32_t value, int16_t minimum, int16_t maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return (int16_t)value;
}

static void flushModemInput(void) {
  while (ModemSerial.available() > 0) {
    ModemSerial.read();
  }
}

static void flushRs485Input(void) {
  while (RS485Serial.available() > 0) {
    RS485Serial.read();
  }
}

static bool responseContains(const String &response, const char *token) {
  return response.indexOf(token) >= 0;
}

static String readModemResponse(uint32_t timeoutMs) {
  String response;
  const uint32_t deadline = millis() + timeoutMs;
  uint32_t lastByteAt = 0;
  bool sawData = false;

  while ((int32_t)(deadline - millis()) > 0) {
    while (ModemSerial.available() > 0) {
      response += (char)ModemSerial.read();
      lastByteAt = millis();
      sawData = true;
    }

    if (sawData && (millis() - lastByteAt) >= QUIET_GAP_MS) {
      break;
    }

    delay(10);
  }

  return response;
}

static String sendATForResponse(const char *command, uint32_t timeoutMs = AT_TIMEOUT_MS) {
  flushModemInput();

  Serial.printf("\n>> %s\n", command);
  ModemSerial.print(command);
  ModemSerial.print("\r");

  String response = readModemResponse(timeoutMs);
  if (response.length() == 0) {
    Serial.println("<< [timeout]");
  } else {
    Serial.print("<< ");
    Serial.print(response);
    if (!response.endsWith("\n")) {
      Serial.println();
    }
  }

  return response;
}

static int32_t parseFixedWidthInt(const char *text, size_t length) {
  int32_t value = 0;

  for (size_t index = 0; index < length; ++index) {
    if (text[index] < '0' || text[index] > '9') {
      return -1;
    }
    value = (value * 10) + (text[index] - '0');
  }

  return value;
}

static int64_t daysFromCivil(int32_t year, uint32_t month, uint32_t day) {
  year -= month <= 2U;
  const int32_t era = (year >= 0 ? year : year - 399) / 400;
  const uint32_t yoe = (uint32_t)(year - era * 400);
  const uint32_t adjustedMonth = month + (month > 2U ? (uint32_t)-3 : 9U);
  const uint32_t doy = (153U * adjustedMonth + 2U) / 5U + day - 1U;
  const uint32_t doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
  return (int64_t)era * 146097LL + (int64_t)doe - 719468LL;
}

static bool parseCclkToUtcEpoch(const String &response, uint32_t &utcEpoch) {
  const int prefixIndex = response.indexOf("+CCLK: \"");
  if (prefixIndex < 0) {
    return false;
  }

  const int startIndex = prefixIndex + 8;
  const int endIndex = response.indexOf('"', startIndex);
  if (endIndex < 0) {
    return false;
  }

  const String timeText = response.substring(startIndex, endIndex);
  const char *text = timeText.c_str();
  const char *firstSlash = strchr(text, '/');
  if (firstSlash == nullptr) {
    return false;
  }

  const size_t yearDigits = (size_t)(firstSlash - text);
  if (yearDigits != 2U && yearDigits != 4U) {
    return false;
  }

  int32_t year = parseFixedWidthInt(text, yearDigits);
  if (year < 0) {
    return false;
  }
  if (yearDigits == 2U) {
    year += 2000;
  }

  const char *monthText = firstSlash + 1;
  if (monthText[2] != '/' || monthText[5] != ',' || monthText[8] != ':' || monthText[11] != ':') {
    return false;
  }

  const int32_t month = parseFixedWidthInt(monthText, 2);
  const int32_t day = parseFixedWidthInt(monthText + 3, 2);
  const int32_t hour = parseFixedWidthInt(monthText + 6, 2);
  const int32_t minute = parseFixedWidthInt(monthText + 9, 2);
  const int32_t second = parseFixedWidthInt(monthText + 12, 2);
  if (month < 1 || month > 12 || day < 1 || day > 31 || hour < 0 || hour > 23 || minute < 0 ||
      minute > 59 || second < 0 || second > 59) {
    return false;
  }

  const char sign = monthText[14];
  if (sign != '+' && sign != '-') {
    return false;
  }

  if (parseFixedWidthInt(monthText + 15, 2) < 0) {
    return false;
  }

  const int64_t days = daysFromCivil(year, (uint32_t)month, (uint32_t)day);
  const int64_t utcSeconds =
      days * 86400LL + (int64_t)hour * 3600LL + (int64_t)minute * 60LL + (int64_t)second +
      (int64_t)TIMESTAMP_OFFSET_S;
  if (utcSeconds < 0 || utcSeconds > 0xFFFFFFFFLL) {
    return false;
  }

  utcEpoch = (uint32_t)utcSeconds;
  return true;
}

static bool tryReadUtcTimestamp(uint32_t &utcEpoch) {
  const String response = sendATForResponse("AT+CCLK?", CCLK_TIMEOUT_MS);
  return response.length() > 0 && parseCclkToUtcEpoch(response, utcEpoch);
}

static bool syncModemTimeWithNtp(void) {
  char command[96];
  snprintf(command, sizeof(command), "AT+CNTP=\"%s\",0", NTP_SERVER);
  if (!sendAT(command, "OK", nullptr, CNTP_TIMEOUT_MS)) {
    return false;
  }

  const String response = sendATForResponse("AT+CNTP", CNTP_TIMEOUT_MS);
  return responseContains(response, "+CNTP: 0");
}

static bool getUtcTimestamp(uint32_t &utcEpoch) {
  if (tryReadUtcTimestamp(utcEpoch)) {
    Serial.printf("UTC timestamp=%lu\n", (unsigned long)utcEpoch);
    return true;
  }

  Serial.println("UTC time not ready from network clock, trying NTP sync");

  const uint32_t startedAt = millis();
  while (millis() - startedAt < UTC_SYNC_TIMEOUT_MS) {
    if (syncModemTimeWithNtp() && tryReadUtcTimestamp(utcEpoch)) {
      Serial.printf("UTC timestamp=%lu\n", (unsigned long)utcEpoch);
      return true;
    }

    delay(2000);
  }

  return false;
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
  appendU32BE(buffer, bufferSize, offset, gRetained.packetCounter);
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

static bool sendAT(const char *command,
                   const char *expectA,
                   const char *expectB,
                   uint32_t timeoutMs) {
  String response = sendATForResponse(command, timeoutMs);
  if (response.length() == 0) {
    return false;
  }

  if (expectA != nullptr && responseContains(response, expectA)) {
    return true;
  }
  if (expectB != nullptr && responseContains(response, expectB)) {
    return true;
  }
  return false;
}

static bool waitForAT(void) {
  for (int attempt = 1; attempt <= 10; ++attempt) {
    Serial.printf("Checking modem AT link (%d/10)\n", attempt);
    if (sendAT("AT", "OK", nullptr, 1500)) {
      return true;
    }
    delay(1000);
  }
  return false;
}

static bool prepareModem(void) {
  if (!sendAT("ATE0", "OK", nullptr, 3000)) {
    return false;
  }

  if (!sendAT("AT+CMEE=2", "OK", nullptr, 3000)) {
    return false;
  }

  if (!sendAT("AT+CFUN=1", "OK", nullptr, 10000)) {
    return false;
  }

  delay(2000);
  return true;
}

static bool configureBand(void) {
  char command[32];

  sendAT("AT+QCBAND?", "+QCBAND:", nullptr, 3000);

  snprintf(command, sizeof(command), "AT+QCBAND=%u,%u", NB_MODE, NB_BAND);
  if (!sendAT(command, "OK", nullptr, 5000)) {
    return false;
  }

  return sendAT("AT+QCBAND?", "+QCBAND:", nullptr, 3000);
}

static bool waitForSimReady(void) {
  const uint32_t startedAt = millis();

  while (millis() - startedAt < SIM_READY_TIMEOUT_MS) {
    if (sendAT("AT+CPIN?", "+CPIN: READY", nullptr, 3000)) {
      return true;
    }
    delay(1000);
  }

  return false;
}

static bool waitForNetworkRegistration(void) {
  const uint32_t startedAt = millis();

  while (millis() - startedAt < NETWORK_REG_TIMEOUT_MS) {
    flushModemInput();

    Serial.println("\n>> AT+CEREG?");
    ModemSerial.print("AT+CEREG?\r");

    String response = readModemResponse(3000);
    if (response.length() == 0) {
      Serial.println("<< [timeout]");
    } else {
      Serial.print("<< ");
      Serial.print(response);
      if (!response.endsWith("\n")) {
        Serial.println();
      }

      if (responseContains(response, "+CEREG: 0,1") ||
          responseContains(response, "+CEREG: 1,1") ||
          responseContains(response, "+CEREG: 2,1") ||
          responseContains(response, "+CEREG: 3,1") ||
          responseContains(response, "+CEREG: 4,1") ||
          responseContains(response, "+CEREG: 5,1") ||
          responseContains(response, "+CEREG: 0,5") ||
          responseContains(response, "+CEREG: 1,5") ||
          responseContains(response, "+CEREG: 2,5") ||
          responseContains(response, "+CEREG: 3,5") ||
          responseContains(response, "+CEREG: 4,5") ||
          responseContains(response, "+CEREG: 5,5")) {
        return true;
      }
    }

    delay(3000);
  }

  return false;
}

static bool openUdpSocket(void) {
  Serial.printf("Opening packet network, this can take up to %lu s\n",
                NETOPEN_TIMEOUT_MS / 1000UL);

  if (!sendAT("AT+NETOPEN", "+NETOPEN: 0", "Network is already opened", NETOPEN_TIMEOUT_MS)) {
    return false;
  }

  char command[64];
  snprintf(command, sizeof(command), "AT+CIPOPEN=0,\"UDP\",,,%u", UDP_LOCAL_PORT);
  return sendAT(command, "+CIPOPEN: 0,0", nullptr, CIPOPEN_TIMEOUT_MS);
}

static bool initializeModem(void) {
  Serial.printf("SIM7028 UART RX=%d TX=%d baud=%lu\n", MODEM_RX_PIN, MODEM_TX_PIN, MODEM_BAUD);

  if (!waitForAT()) {
    Serial.println("FAIL: modem did not answer AT");
    return false;
  }

  if (!prepareModem()) {
    Serial.println("FAIL: modem preparation failed");
    return false;
  }

  if (!configureBand()) {
    Serial.println("FAIL: band configuration failed");
    return false;
  }

  if (!waitForSimReady()) {
    Serial.println("FAIL: SIM never became ready");
    return false;
  }

  if (!waitForNetworkRegistration()) {
    Serial.println("FAIL: network registration timed out");
    return false;
  }

  if (!openUdpSocket()) {
    Serial.println("FAIL: UDP socket open failed");
    return false;
  }

  Serial.println("SIM7028 ready for UDP sends");
  return true;
}

static void closeSession(void) {
  sendAT("AT+CIPCLOSE=0", "+CIPCLOSE: 0,0", nullptr, 10000);
  sendAT("AT+NETCLOSE", "+NETCLOSE: 0", nullptr, 10000);
  // Disable the modem RF/baseband while the ESP32 sleeps.  It is restored by
  // prepareModem() on the next timer wake-up.
  sendAT("AT+CFUN=0", "OK", nullptr, 10000);
  gModemReady = false;
}

static bool ensureModemReady(void) {
  if (gModemReady) {
    return true;
  }

  Serial.println("Initializing SIM7028 session");
  gModemReady = initializeModem();
  return gModemReady;
}

static bool sendBinaryPacketUdp(uint32_t startTimestampUtc,
                                const SensorSample *samples,
                                uint8_t sampleCount) {
  uint8_t packet[128];
  const size_t packetLength =
      buildPacket(packet, sizeof(packet), startTimestampUtc, samples, sampleCount);
  if (packetLength == 0) {
    Serial.println("Packet build failed");
    return false;
  }

  Serial.printf("Sending packet counter=%lu samples=%u interval=%u s start=%lu\n",
                (unsigned long)gRetained.packetCounter,
                (unsigned)sampleCount,
                (unsigned)SAMPLE_INTERVAL_S,
                (unsigned long)startTimestampUtc);
  printHexDump(packet, packetLength);

  char command[96];
  snprintf(command,
           sizeof(command),
           "AT+CIPSEND=0,%u,\"%s\",%u",
           (unsigned)packetLength,
           UDP_SERVER_IP,
           UDP_SERVER_PORT);

  flushModemInput();

  Serial.printf("\n>> %s\n", command);
  ModemSerial.print(command);
  ModemSerial.print("\r");

  String prompt = readModemResponse(CIPSEND_PROMPT_TIMEOUT_MS);
  if (prompt.length() == 0) {
    Serial.println("<< [timeout waiting for prompt]");
    return false;
  }

  Serial.print("<< ");
  Serial.print(prompt);
  if (!prompt.endsWith("\n")) {
    Serial.println();
  }

  if (!responseContains(prompt, ">")) {
    return false;
  }

  Serial.printf(">> [payload] %u bytes\n", (unsigned)packetLength);
  ModemSerial.write(packet, packetLength);

  String response = readModemResponse(CIPSEND_RESULT_TIMEOUT_MS);
  if (response.length() == 0) {
    Serial.println("<< [timeout after payload]");
    return false;
  }

  Serial.print("<< ");
  Serial.print(response);
  if (!response.endsWith("\n")) {
    Serial.println();
  }

  char expectedToken[32];
  snprintf(expectedToken,
           sizeof(expectedToken),
           "+CIPSEND: 0,%u,%u",
           (unsigned)packetLength,
           (unsigned)packetLength);

  if (!responseContains(response, expectedToken)) {
    return false;
  }

  ++gRetained.packetCounter;
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

static void resetBatch(void) {
  gRetained.sampleCount = 0;
}

static bool sendCurrentBatch(void) {
  if (gRetained.sampleCount == 0) {
    Serial.println("No samples queued for send");
    return true;
  }

  if (!ensureModemReady()) {
    Serial.println("Batch send skipped: modem is not ready");
    return false;
  }

  uint32_t timestampUtc = 0;
  if (!getUtcTimestamp(timestampUtc)) {
    Serial.println("Cannot timestamp packet: UTC time read failed");
    return false;
  }

  // The modem clock is read after the last sample.  Earlier batch entries are
  // spaced by the configured interval, matching the ingest protocol.
  const uint32_t batchStartTimestampUtc =
      timestampUtc - ((uint32_t)(gRetained.sampleCount - 1U) * SAMPLE_INTERVAL_S);
  if (sendBinaryPacketUdp(batchStartTimestampUtc, gRetained.samples, gRetained.sampleCount)) {
    Serial.println("PASS: batched binary packet sent over UDP");
    toggleIo18Led();
    resetBatch();
    return true;
  } else {
    Serial.println("FAIL: UDP binary packet send failed, keeping batch for retry");
    closeSession();
    return false;
  }
}

static void collectScheduledSample(void) {
  if (gRetained.sampleCount >= SAMPLE_COUNT) {
    Serial.println("Batch is full, retrying send before collecting another sample");
    sendCurrentBatch();
    return;
  }

  Serial.printf("Sampling sensor for slot %u/%u\n",
                (unsigned)(gRetained.sampleCount + 1U),
                (unsigned)SAMPLE_COUNT);
  flushRs485Input();

  SensorSample sample;
  if (!readSensorFrame(sample, SENSOR_FRAME_TIMEOUT_MS)) {
    Serial.println("Sensor sample timeout, keeping current batch open");
    return;
  }

  gRetained.samples[gRetained.sampleCount] = sample;
  ++gRetained.sampleCount;

  Serial.printf("Sample %u stored: turbidity=%.2f NTU temperature=%.2f C\n",
                (unsigned)gRetained.sampleCount,
                sample.turbidityCentiNtu / 100.0f,
                sample.temperatureCentiC / 100.0f);

  if (gRetained.sampleCount < SAMPLE_COUNT) {
    Serial.printf("Batch progress: %u/%u samples\n", (unsigned)gRetained.sampleCount, (unsigned)SAMPLE_COUNT);
    return;
  }

  Serial.println("Batch complete, sending to server");
  sendCurrentBatch();
}

static void enterDeepSleep(void) {
  // A successful packet does not need an open PDP/socket while waiting for
  // the next acquisition.  CFUN=0 is also sent when opening the session
  // failed part way through, so the modem radio is not left active.
  if (gModemReady) {
    closeSession();
  } else {
    sendAT("AT+CFUN=0", "OK", nullptr, 10000);
  }

  RS485Serial.end();
  ModemSerial.end();

  // Keep board-controlled outputs in their inactive state throughout deep
  // sleep.  The holds are released at the beginning of the next boot.
  digitalWrite(RS485_DE, LOW);
  digitalWrite(IO18_LED_PIN, LOW);
  gpio_hold_en((gpio_num_t)RS485_DE);
  gpio_hold_en((gpio_num_t)IO18_LED_PIN);
  gpio_deep_sleep_hold_en();

  Serial.printf("Sleeping for %u s\n", (unsigned)SAMPLE_INTERVAL_S);
  Serial.flush();
  esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_US);
  esp_deep_sleep_start();
}

void setup() {
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)RS485_DE);
  gpio_hold_dis((gpio_num_t)IO18_LED_PIN);

  Serial.begin(115200);
  delay(200);

  initializeRetainedState();

  Serial.println();
  Serial.println("=== Guarda-Rios Station SIM7028 UDP ===");
  Serial.printf("Sample interval=%u s batch size=%u\n",
                (unsigned)SAMPLE_INTERVAL_S,
                (unsigned)SAMPLE_COUNT);
  Serial.printf("Timestamp offset=%ld s\n", (long)TIMESTAMP_OFFSET_S);

  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);
  pinMode(IO18_LED_PIN, OUTPUT);
  digitalWrite(IO18_LED_PIN, LOW);
  RS485Serial.begin(RS485_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  RS485Serial.setTimeout(100);
  Serial.printf("RS-485 RX=%d TX=%d DE=%d baud=%lu\n", RS485_RX, RS485_TX, RS485_DE, RS485_BAUD);

  ModemSerial.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  ModemSerial.setTimeout(1000);
}

void loop() {
  collectScheduledSample();
  enterDeepSleep();
}
