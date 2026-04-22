/*
 * SIM7028 UDP smoke test for ESP32-WROOM-32D
 *
 * Pure-AT version for the working SIM7028 flow.
 * No TinyGSM.
 * No APN configuration.
 * Default radio band forced to band 20 for this deployment.
 *
 * Flow:
 * 1. AT link check
 * 2. Basic modem preparation
 * 3. Band 20 selection
 * 4. SIM ready check
 * 5. NB-IoT registration wait
 * 6. NETOPEN
 * 7. UDP socket open
 * 8. UDP send "hello"
 *
 * Next step:
 * Replace "hello" with a compact binary payload carrying random turbidity and
 * temperature test values.
 */

#include <Arduino.h>
#include <HardwareSerial.h>

static const int MODEM_RX_PIN = 16;
static const int MODEM_TX_PIN = 17;
static const unsigned long MODEM_BAUD = 115200;

/* Replace with your server endpoint. */
static const char *UDP_SERVER_IP = "198.51.100.10";
static const uint16_t UDP_SERVER_PORT = 9000;
static const uint16_t UDP_LOCAL_PORT = 5000;

/* Working network profile for this SIM/operator combination. */
static const uint8_t NB_MODE = 0;
static const uint8_t NB_BAND = 20;

static const uint32_t AT_TIMEOUT_MS = 3000;
static const uint32_t SIM_READY_TIMEOUT_MS = 30000;
static const uint32_t NETWORK_REG_TIMEOUT_MS = 300000;
static const uint32_t NETOPEN_TIMEOUT_MS = 300000;
static const uint32_t CIPOPEN_TIMEOUT_MS = 30000;
static const uint32_t CIPSEND_PROMPT_TIMEOUT_MS = 5000;
static const uint32_t CIPSEND_RESULT_TIMEOUT_MS = 30000;
static const uint32_t QUIET_GAP_MS = 200;

HardwareSerial ModemSerial(2);

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

static bool sendHelloUdp(void) {
  char command[96];
  snprintf(command,
           sizeof(command),
           "AT+CIPSEND=0,5,\"%s\",%u",
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

  Serial.println(">> [payload] hello");
  ModemSerial.print("hello");

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

  return responseContains(response, "+CIPSEND: 0,5,5");
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

  if (!sendHelloUdp()) {
    Serial.println("FAIL: UDP hello send failed");
    return;
  }

  Serial.println("PASS: hello sent over UDP");
  closeSession();
}

void loop() {
  delay(1000);
}
