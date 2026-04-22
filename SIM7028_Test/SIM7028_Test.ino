/*
 * SIM7028 UDP binary packet test for ESP32-WROOM-32D
 *
 * Pure-AT version for the working SIM7028 flow.
 * No TinyGSM.
 * No APN configuration.
 * Default radio band forced to band 20 for this deployment.
 *
 * Packet format:
 * 1 byte  version
 * 2 bytes station_id
 * 4 bytes packet_counter
 * 4 bytes start_timestamp
 * 2 bytes interval_s
 * 1 byte  sample_count
 * N x sensor samples
 * 16 bytes auth_tag
 *
 * Test assumptions:
 * - each sample is 4 bytes
 * - temperature is int16 fixed-point in centi-degC
 * - turbidity is int16 fixed-point in centi-NTU
 * - sample order is temperature first, then turbidity
 * - multi-byte integers are encoded big-endian
 * - start_timestamp uses seconds since boot for this test sketch
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>
#include <mbedtls/md.h>

static const int MODEM_RX_PIN = 16;
static const int MODEM_TX_PIN = 17;
static const unsigned long MODEM_BAUD = 115200;

/* Replace with your server endpoint. */
static const char *UDP_SERVER_IP = "172.233.121.41";
static const uint16_t UDP_SERVER_PORT = 40416;
static const uint16_t UDP_LOCAL_PORT = 5000;

/* Working network profile for this SIM/operator combination. */
static const uint8_t NB_MODE = 0;
static const uint8_t NB_BAND = 20;

static const uint8_t PROTOCOL_VERSION = 1;
static const uint16_t STATION_ID = 1;
static const char *STATION_TOKEN = "a70b521465b960822290c93b7e06b2b5111e140ff17e0dc4e7107c53565775c0";
static const uint16_t SAMPLE_INTERVAL_S = 60;
static const uint8_t SAMPLE_COUNT = 6;
static const size_t AUTH_TAG_SIZE = 16;
static const int16_t MIN_TURBIDITY_CENTI_NTU = 0;
static const int16_t MAX_TURBIDITY_CENTI_NTU = 327;
static const int16_t MIN_TEMPERATURE_CENTI_C = -5;
static const int16_t MAX_TEMPERATURE_CENTI_C = 50;

static const uint32_t AT_TIMEOUT_MS = 3000;
static const uint32_t SIM_READY_TIMEOUT_MS = 30000;
static const uint32_t NETWORK_REG_TIMEOUT_MS = 300000;
static const uint32_t NETOPEN_TIMEOUT_MS = 300000;
static const uint32_t CIPOPEN_TIMEOUT_MS = 30000;
static const uint32_t CIPSEND_PROMPT_TIMEOUT_MS = 5000;
static const uint32_t CIPSEND_RESULT_TIMEOUT_MS = 30000;
static const uint32_t QUIET_GAP_MS = 200;

HardwareSerial ModemSerial(2);
static uint32_t gPacketCounter = 1;

struct SensorSample {
  int16_t turbidityCentiNtu;
  int16_t temperatureCentiC;
};

static void flushModemInput(void) {
  while (ModemSerial.available() > 0) {
    ModemSerial.read();
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

static SensorSample generateRandomSample(void) {
  SensorSample sample;

  sample.turbidityCentiNtu =
      (int16_t)random((long)MIN_TURBIDITY_CENTI_NTU, (long)MAX_TURBIDITY_CENTI_NTU + 1L);
  sample.temperatureCentiC =
      (int16_t)random((long)MIN_TEMPERATURE_CENTI_C, (long)MAX_TEMPERATURE_CENTI_C + 1L);

  return sample;
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

static size_t buildTestPacket(uint8_t *buffer, size_t bufferSize) {
  const size_t expectedSize =
      1 + 2 + 4 + 4 + 2 + 1 + ((size_t)SAMPLE_COUNT * sizeof(SensorSample)) + AUTH_TAG_SIZE;

  if (bufferSize < expectedSize) {
    return 0;
  }

  const uint32_t startTimestamp = millis() / 1000UL;
  size_t offset = 0;

  appendU8(buffer, bufferSize, offset, PROTOCOL_VERSION);
  appendU16BE(buffer, bufferSize, offset, STATION_ID);
  appendU32BE(buffer, bufferSize, offset, gPacketCounter);
  appendU32BE(buffer, bufferSize, offset, startTimestamp);
  appendU16BE(buffer, bufferSize, offset, SAMPLE_INTERVAL_S);
  appendU8(buffer, bufferSize, offset, SAMPLE_COUNT);

  for (uint8_t index = 0; index < SAMPLE_COUNT; ++index) {
    const SensorSample sample = generateRandomSample();
    appendI16BE(buffer, bufferSize, offset, sample.temperatureCentiC);
    appendI16BE(buffer, bufferSize, offset, sample.turbidityCentiNtu);

    Serial.printf("Sample %u turbidity=%.2f NTU temperature=%.2f C\n",
                  (unsigned)(index + 1),
                  sample.turbidityCentiNtu / 100.0f,
                  sample.temperatureCentiC / 100.0f);
  }

  if (!appendAuthTag(buffer, bufferSize, offset)) {
    return 0;
  }

  ++gPacketCounter;
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
                   const char *expectB = nullptr,
                   uint32_t timeoutMs = AT_TIMEOUT_MS) {
  flushModemInput();

  Serial.printf("\n>> %s\n", command);
  ModemSerial.print(command);
  ModemSerial.print("\r");

  String response = readModemResponse(timeoutMs);
  if (response.length() == 0) {
    Serial.println("<< [timeout]");
    return false;
  }

  Serial.print("<< ");
  Serial.print(response);
  if (!response.endsWith("\n")) {
    Serial.println();
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

static bool sendBinaryPacketUdp(void) {
  uint8_t packet[128];
  const size_t packetLength = buildTestPacket(packet, sizeof(packet));
  if (packetLength == 0) {
    Serial.println("Packet build failed");
    return false;
  }

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
  snprintf(expectedToken, sizeof(expectedToken), "+CIPSEND: 0,%u,%u", (unsigned)packetLength,
           (unsigned)packetLength);
  return responseContains(response, expectedToken);
}

static void closeSession(void) {
  sendAT("AT+CIPCLOSE=0", "+CIPCLOSE: 0,0", nullptr, 10000);
  sendAT("AT+NETCLOSE", "+NETCLOSE: 0", nullptr, 10000);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("=== SIM7028 UDP Test ===");
  Serial.printf("UART RX=%d TX=%d baud=%lu\n", MODEM_RX_PIN, MODEM_TX_PIN, MODEM_BAUD);

  ModemSerial.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  ModemSerial.setTimeout(1000);

  delay(1000);

  if (!waitForAT()) {
    Serial.println("FAIL: modem did not answer AT");
    return;
  }

  if (!prepareModem()) {
    Serial.println("FAIL: modem preparation failed");
    return;
  }

  if (!configureBand()) {
    Serial.println("FAIL: band configuration failed");
    return;
  }

  if (!waitForSimReady()) {
    Serial.println("FAIL: SIM never became ready");
    return;
  }

  if (!waitForNetworkRegistration()) {
    Serial.println("FAIL: network registration timed out");
    return;
  }

  if (!openUdpSocket()) {
    Serial.println("FAIL: UDP socket open failed");
    return;
  }

  randomSeed((uint32_t)micros());

  if (!sendBinaryPacketUdp()) {
    Serial.println("FAIL: UDP binary packet send failed");
    return;
  }

  Serial.println("PASS: binary packet sent over UDP");
  closeSession();
}

void loop() {
  delay(1000);
}
