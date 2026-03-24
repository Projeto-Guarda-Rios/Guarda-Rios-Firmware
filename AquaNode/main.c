/*
 * STM32L053R8 Sensor Firmware — Bare Metal
 *
 * Reads:
 *   - DFRobot SEN0554 turbidity sensor via USART1 (Modbus RTU)
 *     IO1 (sensor TX) -> PA10 (USART1_RX)
 *     IO2 (sensor RX) -> PA9  (USART1_TX)
 *
 *   - DS18B20 temperature sensor via one-wire
 *     Data -> PA0 (needs 4.7K pull-up to 3.3V)
 *
 * Sends data via RS-485 (ST485EBDR) to ESP32:
 *     RO -> PA3 (USART2_RX)
 *     DI -> PA2 (USART2_TX)
 *     RE + DE -> PA1 (GPIO, HIGH=transmit)
 *
 * Output frame (every 2s on RS-485, 9600 baud):
 *   [0xAA] [0x55] [turb_hi] [turb_lo] [temp_hi] [temp_lo] [checksum]
 *   turb = NTU value from sensor (uint16_t, 0xFFFF = offline)
 *   temp = degrees C x 100 (int16_t, e.g. 2350 = 23.50C, -9999 = offline)
 *   checksum = XOR of bytes 2..5
 */

/* Freestanding type definitions (no libc needed) */
typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;
typedef signed short       int16_t;
typedef signed int         int32_t;

/* ================================================================
 *  Register Definitions
 * ================================================================ */

/* RCC */
#define RCC_BASE      0x40021000UL
#define RCC_CR        (*(volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_CFGR      (*(volatile uint32_t *)(RCC_BASE + 0x0C))
#define RCC_IOPENR    (*(volatile uint32_t *)(RCC_BASE + 0x2C))
#define RCC_APB2ENR   (*(volatile uint32_t *)(RCC_BASE + 0x34))
#define RCC_APB1ENR   (*(volatile uint32_t *)(RCC_BASE + 0x38))

/* GPIOA */
#define GPIOA_BASE    0x50000000UL
#define GPIOA_MODER   (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_OTYPER  (*(volatile uint32_t *)(GPIOA_BASE + 0x04))
#define GPIOA_PUPDR   (*(volatile uint32_t *)(GPIOA_BASE + 0x0C))
#define GPIOA_IDR     (*(volatile uint32_t *)(GPIOA_BASE + 0x10))
#define GPIOA_BSRR    (*(volatile uint32_t *)(GPIOA_BASE + 0x18))
#define GPIOA_AFRL    (*(volatile uint32_t *)(GPIOA_BASE + 0x20))
#define GPIOA_AFRH    (*(volatile uint32_t *)(GPIOA_BASE + 0x24))

/* USART1 (APB2) */
#define USART1_BASE   0x40013800UL
#define USART1_CR1    (*(volatile uint32_t *)(USART1_BASE + 0x00))
#define USART1_BRR    (*(volatile uint32_t *)(USART1_BASE + 0x0C))
#define USART1_ISR    (*(volatile uint32_t *)(USART1_BASE + 0x1C))
#define USART1_ICR    (*(volatile uint32_t *)(USART1_BASE + 0x20))
#define USART1_RDR    (*(volatile uint32_t *)(USART1_BASE + 0x24))
#define USART1_TDR    (*(volatile uint32_t *)(USART1_BASE + 0x28))

/* USART2 (APB1) */
#define USART2_BASE   0x40004400UL
#define USART2_CR1    (*(volatile uint32_t *)(USART2_BASE + 0x00))
#define USART2_BRR    (*(volatile uint32_t *)(USART2_BASE + 0x0C))
#define USART2_ISR    (*(volatile uint32_t *)(USART2_BASE + 0x1C))
#define USART2_ICR    (*(volatile uint32_t *)(USART2_BASE + 0x20))
#define USART2_RDR    (*(volatile uint32_t *)(USART2_BASE + 0x24))
#define USART2_TDR    (*(volatile uint32_t *)(USART2_BASE + 0x28))

/* SysTick */
#define SYSTICK_CSR   (*(volatile uint32_t *)0xE000E010)
#define SYSTICK_RVR   (*(volatile uint32_t *)0xE000E014)
#define SYSTICK_CVR   (*(volatile uint32_t *)0xE000E018)

/* USART status bits */
#define ISR_TXE   (1u << 7)
#define ISR_TC    (1u << 6)
#define ISR_RXNE  (1u << 5)
#define ISR_ORE   (1u << 3)
#define ICR_ORECF (1u << 3)
#define ICR_TCCF  (1u << 6)

/* ================================================================
 *  Globals
 * ================================================================ */

volatile uint32_t ms_ticks;

/* ================================================================
 *  Vector Table & Startup
 * ================================================================ */

extern uint32_t _estack, _sdata, _edata, _sidata, _sbss, _ebss;

void Reset_Handler(void);
int  main(void);

void Default_Handler(void) { while (1); }
void NMI_Handler(void)       __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)       __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)    __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void);

__attribute__((section(".isr_vector"), used))
const uint32_t vector_table[] = {
    (uint32_t)&_estack,
    (uint32_t)Reset_Handler,
    (uint32_t)NMI_Handler,
    (uint32_t)HardFault_Handler,
    0, 0, 0, 0, 0, 0, 0,
    (uint32_t)SVC_Handler,
    0, 0,
    (uint32_t)PendSV_Handler,
    (uint32_t)SysTick_Handler,
};

void __attribute__((noreturn)) Reset_Handler(void) {
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    dst = &_sbss;
    while (dst < &_ebss) *dst++ = 0;
    main();
    while (1);
}

void SysTick_Handler(void) {
    ms_ticks++;
}

/* ================================================================
 *  Timing
 * ================================================================ */

static void delay_ms(uint32_t ms) {
    uint32_t start = ms_ticks;
    while ((ms_ticks - start) < ms);
}

/* Approximate microsecond delay at 16 MHz */
static void delay_us(uint32_t us) {
    uint32_t n = us * 4;
    while (n--) __asm__ volatile("nop");
}

/* ================================================================
 *  Peripheral Init
 * ================================================================ */

static void clock_init(void) {
    RCC_CR |= (1u << 0);                    /* HSI16ON */
    while (!(RCC_CR & (1u << 2)));           /* Wait HSI16RDY */
    RCC_CFGR = (RCC_CFGR & ~3u) | 1u;      /* SW = HSI16 */
    while ((RCC_CFGR & 0x0Cu) != 0x04u);    /* Wait SWS = HSI16 */
}

static void systick_init(void) {
    SYSTICK_RVR = 16000 - 1;   /* 16 MHz / 16000 = 1 kHz */
    SYSTICK_CVR = 0;
    SYSTICK_CSR = 7;           /* enable, interrupt, processor clock */
}

static void gpio_init(void) {
    RCC_IOPENR |= (1u << 0);     /* GPIOA clock */
    (void)RCC_IOPENR;            /* Read-back ensures clock is active */

    /* PA0: output, open-drain (DS18B20 one-wire), start released */
    GPIOA_MODER  = (GPIOA_MODER & ~(3u << 0)) | (1u << 0);
    GPIOA_OTYPER |= (1u << 0);
    GPIOA_BSRR   = (1u << 0);   /* release (high-Z via open-drain) */

    /* PA1: output, push-pull (RS-485 DE/RE), start LOW = receive */
    GPIOA_MODER = (GPIOA_MODER & ~(3u << 2)) | (1u << 2);
    GPIOA_BSRR  = (1u << 17);

    /* PA2: AF4 = USART2_TX */
    GPIOA_MODER = (GPIOA_MODER & ~(3u << 4)) | (2u << 4);
    GPIOA_AFRL  = (GPIOA_AFRL & ~(0xFu << 8)) | (4u << 8);

    /* PA3: AF4 = USART2_RX */
    GPIOA_MODER = (GPIOA_MODER & ~(3u << 6)) | (2u << 6);
    GPIOA_AFRL  = (GPIOA_AFRL & ~(0xFu << 12)) | (4u << 12);

    /* PA9: AF4 = USART1_TX */
    GPIOA_MODER = (GPIOA_MODER & ~(3u << 18)) | (2u << 18);
    GPIOA_AFRH  = (GPIOA_AFRH & ~(0xFu << 4)) | (4u << 4);

    /* PA10: AF4 = USART1_RX */
    GPIOA_MODER = (GPIOA_MODER & ~(3u << 20)) | (2u << 20);
    GPIOA_AFRH  = (GPIOA_AFRH & ~(0xFu << 8)) | (4u << 8);
}

static void usart1_init(void) {
    RCC_APB2ENR |= (1u << 14);           /* USART1 clock */
    USART1_BRR = 1667;                   /* 16 MHz / 9600 */
    USART1_CR1 = (1u << 3) | (1u << 2) | (1u << 0);  /* TE|RE|UE */
}

static void usart2_init(void) {
    RCC_APB1ENR |= (1u << 17);           /* USART2 clock */
    USART2_BRR = 1667;                   /* 16 MHz / 9600 */
    USART2_CR1 = (1u << 3) | (1u << 2) | (1u << 0);  /* TE|RE|UE */
}

/* ================================================================
 *  USART Helpers
 * ================================================================ */

static void u1_tx(uint8_t b) {
    while (!(USART1_ISR & ISR_TXE));
    USART1_TDR = b;
}

static void u1_tx_buf(const uint8_t *buf, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) u1_tx(buf[i]);
    while (!(USART1_ISR & ISR_TC));
    USART1_ICR = ICR_TCCF;
}

/* Receive up to max bytes. Returns after timeout_ms of silence. */
static uint8_t u1_rx_buf(uint8_t *buf, uint8_t max, uint16_t timeout_ms) {
    uint8_t count = 0;
    uint32_t start = ms_ticks;
    uint32_t last_byte = start;

    if (USART1_ISR & ISR_ORE) USART1_ICR = ICR_ORECF;
    while (USART1_ISR & ISR_RXNE) (void)USART1_RDR;  /* flush */

    while (count < max && (ms_ticks - start) < timeout_ms) {
        if (USART1_ISR & ISR_RXNE) {
            buf[count++] = (uint8_t)USART1_RDR;
            last_byte = ms_ticks;
        }
        /* Frame-end: gap > 5 ms after first byte */
        if (count > 0 && (ms_ticks - last_byte) > 5) break;
    }
    return count;
}

static void u2_tx(uint8_t b) {
    while (!(USART2_ISR & ISR_TXE));
    USART2_TDR = b;
}

static void u2_tx_buf(const uint8_t *buf, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) u2_tx(buf[i]);
    while (!(USART2_ISR & ISR_TC));
    USART2_ICR = ICR_TCCF;
}

/* ================================================================
 *  RS-485 DE/RE Control (PA1)
 * ================================================================ */

static void rs485_tx_enable(void) {
    GPIOA_BSRR = (1u << 1);    /* PA1 HIGH = transmit */
    delay_us(10);
}

static void rs485_tx_disable(void) {
    delay_us(10);
    GPIOA_BSRR = (1u << 17);   /* PA1 LOW = receive */
}

/* ================================================================
 *  DS18B20 One-Wire on PA0 (open-drain)
 * ================================================================ */

#define OW_LOW()     GPIOA_BSRR = (1u << 16)  /* drive low */
#define OW_RELEASE() GPIOA_BSRR = (1u << 0)   /* release (pull-up) */
#define OW_READ()    ((GPIOA_IDR >> 0) & 1)

static uint8_t ow_reset(void) {
    OW_LOW();
    delay_us(480);
    OW_RELEASE();
    delay_us(70);
    uint8_t present = !OW_READ();
    delay_us(410);
    return present;
}

static void ow_write_bit(uint8_t bit) {
    OW_LOW();
    if (bit) { delay_us(6);  OW_RELEASE(); delay_us(64); }
    else     { delay_us(60); OW_RELEASE(); delay_us(10); }
}

static uint8_t ow_read_bit(void) {
    OW_LOW();
    delay_us(6);
    OW_RELEASE();
    delay_us(9);
    uint8_t b = OW_READ();
    delay_us(55);
    return b;
}

static void ow_write_byte(uint8_t v) {
    for (uint8_t i = 0; i < 8; i++) { ow_write_bit(v & 1); v >>= 1; }
}

static uint8_t ow_read_byte(void) {
    uint8_t v = 0;
    for (uint8_t i = 0; i < 8; i++) v |= (ow_read_bit() << i);
    return v;
}

static uint8_t ds_crc8(const uint8_t *d, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t b = d[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = ((crc ^ b) & 1) ? (crc >> 1) ^ 0x8C : crc >> 1;
            b >>= 1;
        }
    }
    return crc;
}

static void ds18b20_start_conversion(void) {
    if (!ow_reset()) return;
    ow_write_byte(0xCC);   /* Skip ROM */
    ow_write_byte(0x44);   /* Convert T */
}

/* Returns temperature in degrees C x 100, or -9999 on error */
static int16_t ds18b20_read_temp(void) {
    if (!ow_reset()) return -9999;
    ow_write_byte(0xCC);   /* Skip ROM */
    ow_write_byte(0xBE);   /* Read Scratchpad */

    uint8_t sp[9];
    for (uint8_t i = 0; i < 9; i++) sp[i] = ow_read_byte();
    if (ds_crc8(sp, 8) != sp[8]) return -9999;

    int16_t raw = (int16_t)((uint16_t)sp[1] << 8 | sp[0]);
    /* 12-bit: LSB = 0.0625 C.  raw * 625 / 100 = raw * 25 / 4 */
    return (int16_t)(((int32_t)raw * 25) / 4);
}

/* ================================================================
 * Turbidity Sensor — DFRobot SEN0554 (Proprietary UART)
 *
 * Sends the 5-byte "Read AD Data" command and parses the
 * 5-byte response frame.
 * ================================================================ */

/* Debug buffer — shared with main loop for RS-485 debug output */
static uint8_t dbg_resp[5];
static uint8_t dbg_n;

static uint16_t turbidity_read(void) {
    /* Command to "Read AD data": 0x18 0x05 0x00 0x01 0x0D */
    uint8_t req[5];
    req[0] = 0x18;
    req[1] = 0x05;
    req[2] = 0x00;
    req[3] = 0x01;
    req[4] = 0x0D;

    /* Flush incoming buffer to clear any line noise */
    while (USART1_ISR & ISR_RXNE) (void)USART1_RDR;

    /* Send the 5-byte request */
    delay_ms(4);
    u1_tx_buf(req, 5);

    /* Expected Response (5 bytes):
     * [0x18] [0x05] [Status] [AD_Value] [0x0D]
     */
    dbg_n = u1_rx_buf(dbg_resp, 5, 500);

    if (dbg_n < 5) return (uint16_t)(0xFF00 | dbg_n);

    /* Verify frame header, trailer, and status byte */
    if (dbg_resp[0] != 0x18 || dbg_resp[1] != 0x05 || dbg_resp[4] != 0x0D) return 0xFE00;
    if (dbg_resp[2] == 0xAA) return 0xFD00;

    return (uint16_t)dbg_resp[3];
}

/* Send debug frame over RS-485:
 * [0xDB][0xDB][n_rx][resp0][resp1][resp2][resp3][resp4][xor]
 * n_rx = number of bytes actually received from sensor
 * resp0..4 = raw bytes (0x00-padded if n_rx < 5)
 * xor = XOR of bytes 2..7
 */
static void send_debug_frame(void) {
    uint8_t f[9];
    f[0] = 0xDB;
    f[1] = 0xDB;
    f[2] = dbg_n;
    for (uint8_t i = 0; i < 5; i++)
        f[3 + i] = (i < dbg_n) ? dbg_resp[i] : 0x00;
    f[8] = f[2] ^ f[3] ^ f[4] ^ f[5] ^ f[6] ^ f[7];

    rs485_tx_enable();
    u2_tx_buf(f, 9);
    rs485_tx_disable();
}

/* ================================================================
 *  RS-485 Output Frame to ESP32
 *
 *  [0xAA][0x55][turb_hi][turb_lo][temp_hi][temp_lo][checksum]
 * ================================================================ */

static void send_to_esp32(uint16_t turb, int16_t temp) {
    uint8_t f[7];
    f[0] = 0xAA;
    f[1] = 0x55;
    f[2] = (turb >> 8) & 0xFF;
    f[3] = turb & 0xFF;
    f[4] = ((uint16_t)temp >> 8) & 0xFF;
    f[5] = (uint16_t)temp & 0xFF;
    f[6] = f[2] ^ f[3] ^ f[4] ^ f[5];

    rs485_tx_enable();
    u2_tx_buf(f, 7);
    rs485_tx_disable();
}

/* ================================================================
 *  Main
 * ================================================================ */

int main(void) {
    clock_init();
    systick_init();
    gpio_init();
    usart1_init();
    usart2_init();

    delay_ms(100);

    /* Kick off first DS18B20 conversion */
    ds18b20_start_conversion();
    delay_ms(800);

    while (1) {
        /* Read temperature from previous conversion */
        int16_t temp = ds18b20_read_temp();

        /* Start next conversion (runs ~750 ms in background) */
        ds18b20_start_conversion();

        /* Read turbidity via Modbus */
        uint16_t turb = turbidity_read();

        /* Send to ESP32 over RS-485 */
        send_to_esp32(turb, temp);

        /* Send debug frame with raw sensor response */
        delay_ms(50);
        send_debug_frame();

        /* Wait ~2 s between readings */
        delay_ms(2000);
    }

    return 0;
}
