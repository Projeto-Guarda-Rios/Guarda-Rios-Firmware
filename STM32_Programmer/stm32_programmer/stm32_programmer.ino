/*
 * STM32L053R8 SWD Programmer — Arduino Nano Every
 *
 * Programs STM32L053R8T6 (64KB flash) via SWD debug interface.
 * Receives firmware from PC over USB serial, writes via bit-banged SWD.
 *
 * Wiring (no resistors needed — SWD pins are 5V tolerant):
 *   Nano Every D6  -> STM32 SWDIO  (PA13, 5V tolerant)
 *   Nano Every D7  -> STM32 SWCLK  (PA14, 5V tolerant)
 *   Nano Every D5  -> STM32 NRST   (open-drain drive)
 *   Nano Every GND -> Board GND
 *   Nano Every 5V  -> Board 5V regulator input
 */

// ===================== Pin Config =====================
#define SWDIO_PIN 6
#define SWCLK_PIN 7
#define NRST_PIN  5

#define PC_BAUD 115200

// ===================== SWD Constants =====================
#define SWD_OK    0x01
#define SWD_WAIT  0x02
#define SWD_FAULT 0x04
#define SWD_RETRY 200

// DP registers (A[3:2])
#define DP_IDCODE   0x00
#define DP_ABORT    0x00
#define DP_CTRLSTAT 0x04
#define DP_SELECT   0x08
#define DP_RDBUFF   0x0C

// AHB-AP registers
#define AP_CSW 0x00
#define AP_TAR 0x04
#define AP_DRW 0x0C

// Cortex-M0+ debug
#define DHCSR 0xE000EDF0
#define AIRCR 0xE000ED0C

// STM32L0 flash registers
#define FLASH_PECR    0x40022004
#define FLASH_PEKEYR  0x4002200C
#define FLASH_PRGKEYR 0x40022010
#define FLASH_SR      0x40022018

#define PEKEY1  0x89ABCDEF
#define PEKEY2  0x02030405
#define PRGKEY1 0x8C9DAEBF
#define PRGKEY2 0x13141516

#define PECR_PELOCK  (1UL << 0)
#define PECR_PRGLOCK (1UL << 1)
#define PECR_PROG    (1UL << 3)
#define PECR_ERASE   (1UL << 9)
#define SR_BSY       (1UL << 0)

// STM32L053R8: 64KB flash, 128-byte pages
#define FLASH_BASE  0x08000000
#define FLASH_SIZE  0x10000
#define PAGE_SIZE   128
#define PAGE_COUNT  (FLASH_SIZE / PAGE_SIZE)

// PC <-> Arduino protocol
#define PC_CONNECT 0x01
#define PC_ERASE   0x02
#define PC_WRITE   0x03
#define PC_RESET   0x05
#define PC_PING    0x06

#define RESP_ACK   0x79
#define RESP_NACK  0x1F
#define RESP_READY 0x76

// ===================== SWD Bit-Bang =====================

static inline void swdioOut()  { pinMode(SWDIO_PIN, OUTPUT); }
static inline void swdioIn()   { pinMode(SWDIO_PIN, INPUT); }
static inline void swdioHi()   { digitalWrite(SWDIO_PIN, HIGH); }
static inline void swdioLo()   { digitalWrite(SWDIO_PIN, LOW); }
static inline uint8_t swdioRd(){ return (uint8_t)digitalRead(SWDIO_PIN); }
static inline void swclkHi()   { digitalWrite(SWCLK_PIN, HIGH); }
static inline void swclkLo()   { digitalWrite(SWCLK_PIN, LOW); }

static void swdWriteBit(uint8_t b) {
  if (b) swdioHi(); else swdioLo();
  swclkLo();
  swclkHi();
}

static uint8_t swdReadBit() {
  swclkLo();
  uint8_t v = swdioRd();
  swclkHi();
  return v;
}

// 56+ clocks with SWDIO high
static void swdLineReset() {
  swdioOut();
  swdioHi();
  for (uint8_t i = 0; i < 56; i++) { swclkLo(); swclkHi(); }
}

// Switch from JTAG to SWD mode
static void swdInit() {
  pinMode(SWCLK_PIN, OUTPUT);
  swclkHi();

  swdLineReset();

  // JTAG-to-SWD sequence: 0xE79E, 16 bits LSB first
  swdioOut();
  uint16_t seq = 0xE79E;
  for (uint8_t i = 0; i < 16; i++) swdWriteBit((seq >> i) & 1);

  swdLineReset();

  // Idle cycles
  swdioLo();
  for (uint8_t i = 0; i < 4; i++) { swclkLo(); swclkHi(); }
}

// Build SWD request byte
static uint8_t swdReq(uint8_t ap, uint8_t rw, uint8_t addr) {
  uint8_t a2 = (addr >> 2) & 1;
  uint8_t a3 = (addr >> 3) & 1;
  uint8_t par = ap ^ rw ^ a2 ^ a3;
  return 0x81 | (ap << 1) | (rw << 2) | (a2 << 3) | (a3 << 4) | (par << 5);
}

// Core SWD transfer — returns ACK value
static uint8_t swdTransfer(uint8_t req, uint32_t *data) {
  uint8_t i, ack, bit;
  uint32_t val;
  uint8_t par;

  // Request phase (host drives, 8 bits LSB first)
  swdioOut();
  for (i = 0; i < 8; i++) swdWriteBit((req >> i) & 1);

  // Turnaround (host -> target)
  swdioIn();
  swclkLo(); swclkHi();

  // ACK phase (target drives, 3 bits)
  ack  = swdReadBit();
  ack |= swdReadBit() << 1;
  ack |= swdReadBit() << 2;

  if (ack == SWD_OK) {
    if (req & 0x04) {
      // === READ: target sends 32 data bits + 1 parity ===
      val = 0; par = 0;
      for (i = 0; i < 32; i++) {
        bit = swdReadBit();
        val |= ((uint32_t)bit << i);
        par ^= bit;
      }
      par ^= swdReadBit();  // parity check (ignored for simplicity)

      // Turnaround (target -> host)
      swdioOut();
      swclkLo(); swclkHi();

      if (data) *data = val;
    } else {
      // === WRITE: turnaround then host sends 32 data bits + 1 parity ===
      swclkLo(); swclkHi();  // turnaround
      swdioOut();

      val = data ? *data : 0;
      par = 0;
      for (i = 0; i < 32; i++) {
        bit = (val >> i) & 1;
        swdWriteBit(bit);
        par ^= bit;
      }
      swdWriteBit(par);
    }
  } else {
    // WAIT or FAULT — return to host-controlled
    swclkLo(); swclkHi();  // turnaround
    swdioOut();
  }

  // Idle cycles (keep line low between transfers)
  swdioOut();
  swdioLo();
  for (i = 0; i < 8; i++) { swclkLo(); swclkHi(); }

  return ack;
}

// ===================== DP / AP Access =====================

static bool dpRead(uint8_t addr, uint32_t *data) {
  uint8_t r = swdReq(0, 1, addr);
  for (uint16_t i = 0; i < SWD_RETRY; i++) {
    uint8_t ack = swdTransfer(r, data);
    if (ack == SWD_OK) return true;
    if (ack == SWD_FAULT) {
      uint32_t ab = 0x1E;
      swdTransfer(swdReq(0, 0, DP_ABORT), &ab);
    }
    if (ack != SWD_WAIT && ack != SWD_FAULT) break;
  }
  return false;
}

static bool dpWrite(uint8_t addr, uint32_t data) {
  uint8_t r = swdReq(0, 0, addr);
  for (uint16_t i = 0; i < SWD_RETRY; i++) {
    uint8_t ack = swdTransfer(r, &data);
    if (ack == SWD_OK) return true;
    if (ack == SWD_FAULT) {
      uint32_t ab = 0x1E;
      swdTransfer(swdReq(0, 0, DP_ABORT), &ab);
    }
    if (ack != SWD_WAIT && ack != SWD_FAULT) break;
  }
  return false;
}

// AP reads are posted: first read initiates, RDBUFF gets the data
static bool apRead(uint8_t addr, uint32_t *data) {
  uint8_t r = swdReq(1, 1, addr);
  uint32_t dummy;
  for (uint16_t i = 0; i < SWD_RETRY; i++) {
    uint8_t ack = swdTransfer(r, &dummy);
    if (ack == SWD_OK) break;
    if (ack != SWD_WAIT) return false;
  }
  return dpRead(DP_RDBUFF, data);
}

static bool apWrite(uint8_t addr, uint32_t data) {
  uint8_t r = swdReq(1, 0, addr);
  for (uint16_t i = 0; i < SWD_RETRY; i++) {
    uint8_t ack = swdTransfer(r, &data);
    if (ack == SWD_OK) return true;
    if (ack != SWD_WAIT) break;
  }
  return false;
}

// ===================== Memory Access =====================

static bool memWrite32(uint32_t addr, uint32_t val) {
  if (!apWrite(AP_TAR, addr)) return false;
  return apWrite(AP_DRW, val);
}

static bool memRead32(uint32_t addr, uint32_t *val) {
  if (!apWrite(AP_TAR, addr)) return false;
  return apRead(AP_DRW, val);
}

// ===================== Target Control =====================

static bool targetConnect() {
  // Pulse NRST (open-drain: LOW=assert, INPUT=release)
  pinMode(NRST_PIN, OUTPUT);
  digitalWrite(NRST_PIN, LOW);
  delay(50);
  pinMode(NRST_PIN, INPUT);  // release — internal pull-up brings NRST high
  delay(50);

  // Init SWD
  swdInit();

  // Read IDCODE to verify connection
  uint32_t idcode;
  if (!dpRead(DP_IDCODE, &idcode)) return false;

  // Clear any sticky errors
  dpWrite(DP_ABORT, 0x1E);

  // Power up debug + system
  dpWrite(DP_CTRLSTAT, (1UL << 30) | (1UL << 28));

  // Wait for power-up ACK
  uint32_t stat;
  for (uint16_t i = 0; i < 500; i++) {
    dpRead(DP_CTRLSTAT, &stat);
    if ((stat & ((1UL << 31) | (1UL << 29))) == ((1UL << 31) | (1UL << 29)))
      break;
    delay(1);
  }

  // Select AP0 bank 0
  dpWrite(DP_SELECT, 0x00000000);

  // Configure AHB-AP: 32-bit word access, no auto-increment
  apWrite(AP_CSW, 0x23000002);

  return true;
}

static bool targetHalt() {
  // DHCSR: key=0xA05F, C_DEBUGEN=1, C_HALT=1
  return memWrite32(DHCSR, 0xA05F0003);
}

static void targetReset() {
  // AIRCR: VECTKEY=0x05FA, SYSRESETREQ=1
  memWrite32(AIRCR, 0x05FA0004);
  delay(100);
  // Also pulse NRST for good measure
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  delay(50);
  pinMode(5, INPUT);
}

// ===================== STM32L0 Flash =====================

static bool flashWaitDone(uint16_t timeout_ms) {
  uint32_t sr;
  unsigned long start = millis();
  while (millis() - start < timeout_ms) {
    if (!memRead32(FLASH_SR, &sr)) return false;
    if (!(sr & SR_BSY)) return true;
  }
  return false;
}

static bool flashUnlock() {
  uint32_t pecr;
  memRead32(FLASH_PECR, &pecr);

  if (pecr & PECR_PELOCK) {
    memWrite32(FLASH_PEKEYR, PEKEY1);
    memWrite32(FLASH_PEKEYR, PEKEY2);
    memRead32(FLASH_PECR, &pecr);
    if (pecr & PECR_PELOCK) return false;
  }

  if (pecr & PECR_PRGLOCK) {
    memWrite32(FLASH_PRGKEYR, PRGKEY1);
    memWrite32(FLASH_PRGKEYR, PRGKEY2);
    memRead32(FLASH_PECR, &pecr);
    if (pecr & PECR_PRGLOCK) return false;
  }

  return true;
}

static bool flashErasePage(uint32_t addr) {
  if (!flashWaitDone(500)) return false;

  // Set ERASE + PROG in PECR
  uint32_t pecr;
  memRead32(FLASH_PECR, &pecr);
  pecr |= PECR_ERASE | PECR_PROG;
  memWrite32(FLASH_PECR, pecr);

  // Write 0 to any word in the target page — triggers erase
  memWrite32(addr, 0x00000000);

  // Wait for erase to complete
  if (!flashWaitDone(500)) return false;

  // Clear ERASE + PROG
  memRead32(FLASH_PECR, &pecr);
  pecr &= ~(PECR_ERASE | PECR_PROG);
  memWrite32(FLASH_PECR, pecr);

  return true;
}

static bool flashEraseAll() {
  for (uint16_t i = 0; i < PAGE_COUNT; i++) {
    if (!flashErasePage(FLASH_BASE + (uint32_t)i * PAGE_SIZE))
      return false;
  }
  return true;
}

static bool flashWriteWord(uint32_t addr, uint32_t data) {
  if (!flashWaitDone(100)) return false;
  if (!memWrite32(addr, data)) return false;
  return flashWaitDone(100);
}

// ===================== Serial Protocol =====================

uint8_t dataBuf[128];

bool readFromPC(uint8_t *buf, uint16_t len, unsigned long timeout = 5000) {
  unsigned long start = millis();
  uint16_t received = 0;
  while (received < len) {
    if (millis() - start > timeout) return false;
    if (Serial.available()) {
      buf[received++] = Serial.read();
      start = millis();
    }
  }
  return true;
}

void setup() {
  Serial.begin(PC_BAUD);
  pinMode(NRST_PIN, INPUT);  // NRST high-Z (STM32 pull-up keeps it high)
  delay(100);
  Serial.write(RESP_READY);
}

void loop() {
  if (!Serial.available()) return;
  uint8_t cmd = Serial.read();

  switch (cmd) {
    case PC_PING:
      Serial.write(RESP_ACK);
      break;

    case PC_CONNECT: {
      bool ok = targetConnect() && targetHalt() && flashUnlock();
      Serial.write(ok ? RESP_ACK : RESP_NACK);
      break;
    }

    case PC_ERASE: {
      Serial.write(flashEraseAll() ? RESP_ACK : RESP_NACK);
      break;
    }

    case PC_WRITE: {
      uint8_t header[5];
      if (!readFromPC(header, 5)) { Serial.write(RESP_NACK); break; }

      uint32_t addr = ((uint32_t)header[0] << 24) |
                      ((uint32_t)header[1] << 16) |
                      ((uint32_t)header[2] << 8)  |
                      (uint32_t)header[3];
      uint8_t len = header[4];

      if (len == 0 || len > 128 || (len % 4) != 0 || !readFromPC(dataBuf, len)) {
        Serial.write(RESP_NACK);
        break;
      }

      // Write word by word
      bool ok = true;
      for (uint8_t i = 0; i < len; i += 4) {
        uint32_t word = (uint32_t)dataBuf[i]         |
                        ((uint32_t)dataBuf[i+1] << 8)  |
                        ((uint32_t)dataBuf[i+2] << 16) |
                        ((uint32_t)dataBuf[i+3] << 24);
        if (!flashWriteWord(addr + i, word)) { ok = false; break; }
      }
      Serial.write(ok ? RESP_ACK : RESP_NACK);
      break;
    }

    case PC_RESET: {
      targetReset();
      Serial.write(RESP_ACK);
      break;
    }
  }
}
