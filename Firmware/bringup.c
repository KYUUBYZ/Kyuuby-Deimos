/*
 * Kyuuby Deimos v1 - test di bring-up
 *
 * Cosa testa (e stampa sulla porta seriale virtuale USB ogni ~2 s):
 *   [MCU  ] ID del chip, dimensione flash, UID, frequenze di clock
 *   [IMU  ] BMI270 su SPI1: CHIP_ID (0x24), 100 letture di stabilita', soft reset,
 *           poi una lettura INFO di accelerometro e giroscopio
 *   [FLASH] W25Q128 su SPI2: JEDEC ID, e (una volta sola) cancellazione/scrittura/lettura
 *           dell'ultimo settore da 4 kB
 *   [RADIO] SX1280 su SPI3: reset con NRESET, BUSY, GetStatus, registro versione firmware
 *
 * Pin usati (dallo schema):
 *   SPI1 IMU   : PA5 SCK, PA6 MISO, PA7 MOSI, PA4 CS,  PC4 INT1
 *   SPI2 flash : PB13 SCK, PB14 MISO, PB15 MOSI, PB12 CS
 *   SPI3 radio : PB3 SCK, PB4 MISO, PB5 MOSI, PA15 NSS, PB7 NRESET, PB6 BUSY, PB8 DIO1
 *
 * Dipende solo da: CMSIS (registri), HAL_GetTick/HAL_Delay, e dallo stack USB CDC
 * generato da CubeMX (CDC_Transmit_FS, hUsbDeviceFS).
 */

#include "bringup.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

/* 1 = al primo giro cancella/scrive/rilegge l'ultimo settore da 4 kB della flash */
#define FLASH_RW_TEST   1
#define FLASH_TEST_ADDR 0x00FFF000u

/* ------------------------------------------------------------------ PIN --- */
#define IMU_SPI        SPI1
#define IMU_CS_PORT    GPIOA
#define IMU_CS_PIN     4
#define IMU_INT_PORT   GPIOC
#define IMU_INT_PIN    4

#define FL_SPI         SPI2
#define FL_CS_PORT     GPIOB
#define FL_CS_PIN      12

#define RF_SPI         SPI3
#define RF_NSS_PORT    GPIOA
#define RF_NSS_PIN     15
#define RF_RST_PORT    GPIOB
#define RF_RST_PIN     7
#define RF_BUSY_PORT   GPIOB
#define RF_BUSY_PIN    6
#define RF_DIO1_PORT   GPIOB
#define RF_DIO1_PIN    8

/* ------------------------------------------------------------- GPIO ------ */
#define BU_IN  0u
#define BU_OUT 1u
#define BU_AF  2u

static void gpio_cfg(GPIO_TypeDef *p, uint8_t pin, uint32_t mode, uint32_t af)
{
    uint32_t s2 = (uint32_t)pin * 2u;

    p->OSPEEDR |= (3u << s2);            /* very high speed */
    p->OTYPER  &= ~(1u << pin);          /* push-pull */
    p->PUPDR   &= ~(3u << s2);           /* nessun pull interno: i pull-up sono esterni */
    if (mode == BU_AF) {
        uint32_t idx = pin >> 3;
        uint32_t sh  = ((uint32_t)pin & 7u) * 4u;
        p->AFR[idx] = (p->AFR[idx] & ~(0xFu << sh)) | (af << sh);
    }
    p->MODER = (p->MODER & ~(3u << s2)) | (mode << s2);
}

static inline void pin_set(GPIO_TypeDef *p, uint8_t pin, int v)
{
    p->BSRR = v ? (1u << pin) : (1u << (pin + 16u));
}

static inline int pin_get(GPIO_TypeDef *p, uint8_t pin)
{
    return (int)((p->IDR >> pin) & 1u);
}

static void delay_us(uint32_t us)
{
    volatile uint32_t n = us * (SystemCoreClock / 4000000u);
    while (n--) { }
}

/* -------------------------------------------------------------- SPI ------ */
static void spi_init(SPI_TypeDef *s, uint32_t br)
{
    /* master, 8 bit, MSB first, CPOL=0 CPHA=0 (mode 0), NSS software.
       br: 0=/2 1=/4 2=/8 3=/16 ... */
    s->CR1 = 0;
    s->CR2 = 0;
    s->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (br << SPI_CR1_BR_Pos);
    s->CR1 |= SPI_CR1_SPE;
}

static uint8_t spi_byte(SPI_TypeDef *s, uint8_t b)
{
    while (!(s->SR & SPI_SR_TXE)) { }
    *(__IO uint8_t *)&s->DR = b;
    while (!(s->SR & SPI_SR_RXNE)) { }
    return *(__IO uint8_t *)&s->DR;
}

static void spi_xfer(SPI_TypeDef *s, GPIO_TypeDef *csp, uint8_t csn,
                     const uint8_t *tx, uint8_t *rx, uint8_t n)
{
    pin_set(csp, csn, 0);
    for (uint8_t i = 0; i < n; i++) {
        uint8_t r = spi_byte(s, tx ? tx[i] : 0x00);
        if (rx) rx[i] = r;
    }
    while (s->SR & SPI_SR_BSY) { }
    pin_set(csp, csn, 1);
}

void bringup_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN | RCC_APB1ENR_SPI3EN;
    (void)RCC->APB1ENR;

    /* uscite di controllo: prima il livello alto, poi la modalita' (niente glitch) */
    pin_set(IMU_CS_PORT, IMU_CS_PIN, 1);   gpio_cfg(IMU_CS_PORT, IMU_CS_PIN, BU_OUT, 0);
    pin_set(FL_CS_PORT,  FL_CS_PIN,  1);   gpio_cfg(FL_CS_PORT,  FL_CS_PIN,  BU_OUT, 0);
    pin_set(RF_NSS_PORT, RF_NSS_PIN, 1);   gpio_cfg(RF_NSS_PORT, RF_NSS_PIN, BU_OUT, 0);
    pin_set(RF_RST_PORT, RF_RST_PIN, 1);   gpio_cfg(RF_RST_PORT, RF_RST_PIN, BU_OUT, 0);

    /* ingressi */
    gpio_cfg(RF_BUSY_PORT, RF_BUSY_PIN, BU_IN, 0);
    gpio_cfg(RF_DIO1_PORT, RF_DIO1_PIN, BU_IN, 0);
    gpio_cfg(IMU_INT_PORT, IMU_INT_PIN, BU_IN, 0);

    /* SPI1: PA5 SCK, PA6 MISO, PA7 MOSI -> AF5 */
    gpio_cfg(GPIOA, 5,  BU_AF, 5);
    gpio_cfg(GPIOA, 6,  BU_AF, 5);
    gpio_cfg(GPIOA, 7,  BU_AF, 5);
    /* SPI2: PB13 SCK, PB14 MISO, PB15 MOSI -> AF5 */
    gpio_cfg(GPIOB, 13, BU_AF, 5);
    gpio_cfg(GPIOB, 14, BU_AF, 5);
    gpio_cfg(GPIOB, 15, BU_AF, 5);
    /* SPI3: PB3 SCK, PB4 MISO, PB5 MOSI -> AF6 (sovrascrive la funzione JTAG di reset) */
    gpio_cfg(GPIOB, 3,  BU_AF, 6);
    gpio_cfg(GPIOB, 4,  BU_AF, 6);
    gpio_cfg(GPIOB, 5,  BU_AF, 6);

    /* Velocita' (APB2=84 MHz per SPI1, APB1=42 MHz per SPI2/SPI3):
       SPI1 /16 = 5.25 MHz (BMI270 max ~10 MHz)
       SPI2 /4  = 10.5 MHz (flash)
       SPI3 /4  = 10.5 MHz (SX1280 max 18 MHz) */
    spi_init(SPI1, 3);
    spi_init(SPI2, 1);
    spi_init(SPI3, 1);
}

/* ------------------------------------------------------ REPORT USB ------- */
static char     rep[2048];
static uint32_t rl;

static void P(const char *fmt, ...)
{
    if (rl >= sizeof(rep) - 1u) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(rep + rl, sizeof(rep) - rl, fmt, ap);
    va_end(ap);
    if (n > 0) {
        rl += (uint32_t)n;
        if (rl > sizeof(rep) - 1u) rl = sizeof(rep) - 1u;
    }
}

static int usb_tx_idle(void)
{
    USBD_CDC_HandleTypeDef *h = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) && (h != NULL) && (h->TxState == 0);
}

static void usb_flush(void)
{
    uint32_t off = 0;
    while (off < rl) {
        uint32_t n = rl - off;
        if (n > 256u) n = 256u;

        uint32_t t0 = HAL_GetTick();
        while (CDC_Transmit_FS((uint8_t *)(rep + off), (uint16_t)n) != USBD_OK) {
            if (HAL_GetTick() - t0 > 50u) return;      /* nessun host in ascolto */
        }
        t0 = HAL_GetTick();
        while (!usb_tx_idle()) {
            if (HAL_GetTick() - t0 > 50u) return;
        }
        off += n;
    }
}

/* -------------------------------------------------------------- MCU ------ */
static int test_mcu(void)
{
    uint32_t devid  = DBGMCU->IDCODE & 0xFFFu;
    uint32_t sysclk = SystemCoreClock;
    uint16_t fsize  = *(const volatile uint16_t *)FLASHSIZE_BASE;
    const uint32_t *uid = (const uint32_t *)UID_BASE;
    int pass = (devid == 0x413u) && (sysclk == 168000000u);

    P("[MCU  ] DEV_ID=0x%03lX (atteso 0x413)  Flash=%u kB  SYSCLK=%lu MHz  PCLK1=%lu  PCLK2=%lu  -> %s\r\n",
      (unsigned long)devid, (unsigned)fsize, (unsigned long)(sysclk / 1000000u),
      (unsigned long)(HAL_RCC_GetPCLK1Freq() / 1000000u),
      (unsigned long)(HAL_RCC_GetPCLK2Freq() / 1000000u),
      pass ? "PASS" : "FAIL");
    P("         UID=%08lX-%08lX-%08lX\r\n",
      (unsigned long)uid[0], (unsigned long)uid[1], (unsigned long)uid[2]);
    if (!pass) {
        P("         ! atteso 168 MHz con HSE 8 MHz: controlla cristallo e PLL in CubeMX\r\n");
    }
    return pass;
}

/* -------------------------------------------------------------- IMU ------ */
static uint8_t bmi_rd(uint8_t reg)
{
    uint8_t t[3] = { (uint8_t)(reg | 0x80u), 0, 0 }, r[3];
    spi_xfer(IMU_SPI, IMU_CS_PORT, IMU_CS_PIN, t, r, 3);   /* r[1] = byte dummy */
    delay_us(5);
    return r[2];
}

static void bmi_wr(uint8_t reg, uint8_t val)
{
    uint8_t t[2] = { reg, val };
    spi_xfer(IMU_SPI, IMU_CS_PORT, IMU_CS_PIN, t, NULL, 2);
    delay_us(5);
}

static void bmi_rdn(uint8_t reg, uint8_t *d, uint8_t n)    /* n <= 12 */
{
    uint8_t t[14] = { 0 }, r[14];
    t[0] = (uint8_t)(reg | 0x80u);
    spi_xfer(IMU_SPI, IMU_CS_PORT, IMU_CS_PIN, t, r, (uint8_t)(2u + n));
    memcpy(d, r + 2, n);
    delay_us(5);
}

static uint32_t isqrt64(uint64_t v)
{
    uint64_t r = 0, b = 1ULL << 62;
    while (b > v) b >>= 2;
    while (b) {
        if (v >= r + b) { v -= r + b; r = (r >> 1) + b; }
        else            { r >>= 1; }
        b >>= 2;
    }
    return (uint32_t)r;
}

static const char *id_hint(uint8_t v)
{
    if (v == 0xFF) return "0xFF: MISO alto, il chip non risponde (CS, alimentazione, MISO)";
    if (v == 0x00) return "0x00: MISO a massa o chip non alimentato";
    return "valore inatteso: chip diverso oppure modo/velocita' SPI errati";
}

static void imu_info(void)
{
    uint8_t d[12];

    bmi_wr(0x7C, 0x00);              /* PWR_CONF: niente power save */
    HAL_Delay(1);
    bmi_wr(0x7D, 0x06);              /* PWR_CTRL: acc + gyr ON */
    bmi_wr(0x40, 0xA8);              /* ACC_CONF: 100 Hz, normal, performance */
    bmi_wr(0x41, 0x00);              /* ACC_RANGE: +-2 g (16384 LSB/g) */
    bmi_wr(0x42, 0xE9);              /* GYR_CONF: 200 Hz */
    bmi_wr(0x43, 0x00);              /* GYR_RANGE: +-2000 dps */
    HAL_Delay(100);

    bmi_rdn(0x0C, d, 12);            /* ACC X,Y,Z poi GYR X,Y,Z */
    int16_t ax = (int16_t)((d[1]  << 8) | d[0]);
    int16_t ay = (int16_t)((d[3]  << 8) | d[2]);
    int16_t az = (int16_t)((d[5]  << 8) | d[4]);
    int16_t gx = (int16_t)((d[7]  << 8) | d[6]);
    int16_t gy = (int16_t)((d[9]  << 8) | d[8]);
    int16_t gz = (int16_t)((d[11] << 8) | d[10]);

    uint64_t sq = (uint64_t)((int64_t)ax * ax) + (uint64_t)((int64_t)ay * ay) + (uint64_t)((int64_t)az * az);
    uint32_t mg = (uint32_t)(((uint64_t)isqrt64(sq) * 1000u) / 16384u);

    P("         INFO acc x=%d y=%d z=%d |a|=%lu mg (scheda ferma: ~1000)  gyr x=%d y=%d z=%d (fermo: ~0)\r\n",
      ax, ay, az, (unsigned long)mg, gx, gy, gz);
    P("         INFO se acc/gyr sono tutti 0: il BMI270 vuole il suo config file (non incluso), non e' un guasto\r\n");
}

static int test_imu(void)
{
    (void)bmi_rd(0x7F);              /* lettura dummy: il fronte di CS seleziona il modo SPI */
    HAL_Delay(2);

    uint8_t  id  = bmi_rd(0x00);     /* CHIP_ID */
    unsigned bad = 0;
    for (int i = 0; i < 100; i++) {
        if (bmi_rd(0x00) != 0x24u) bad++;
    }

    bmi_wr(0x7E, 0xB6);              /* CMD: soft reset */
    HAL_Delay(10);
    (void)bmi_rd(0x7F);
    HAL_Delay(2);
    uint8_t id2 = bmi_rd(0x00);
    uint8_t err = bmi_rd(0x02);      /* ERR_REG */

    int pass = (id == 0x24u) && (bad == 0u) && (id2 == 0x24u);

    P("[IMU  ] BMI270 CHIP_ID=0x%02X (atteso 0x24)  letture errate=%u/100  dopo reset=0x%02X  ERR_REG=0x%02X  INT1=%d  -> %s\r\n",
      id, bad, id2, err, pin_get(IMU_INT_PORT, IMU_INT_PIN), pass ? "PASS" : "FAIL");
    if (!pass) {
        P("         ! %s\r\n", id_hint(id));
    } else {
        imu_info();
    }
    return pass;
}

/* ------------------------------------------------------------ FLASH ------ */
#define FL_XFER(t, r, n) spi_xfer(FL_SPI, FL_CS_PORT, FL_CS_PIN, (t), (r), (n))

static uint8_t fl_status(void)
{
    uint8_t t[2] = { 0x05, 0 }, r[2];
    FL_XFER(t, r, 2);
    return r[1];
}

static int fl_wait(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    while (fl_status() & 0x01u) {                      /* bit0 = BUSY */
        if (HAL_GetTick() - t0 > ms) return 0;
    }
    return 1;
}

static void fl_wren(void)
{
    uint8_t c = 0x06;
    FL_XFER(&c, NULL, 1);
    delay_us(2);
}

static void fl_erase4k(uint32_t a)
{
    uint8_t t[4] = { 0x20, (uint8_t)(a >> 16), (uint8_t)(a >> 8), (uint8_t)a };
    FL_XFER(t, NULL, 4);
}

static void fl_prog(uint32_t a, const uint8_t *d, uint8_t n)   /* n <= 32 */
{
    uint8_t t[4 + 32];
    t[0] = 0x02; t[1] = (uint8_t)(a >> 16); t[2] = (uint8_t)(a >> 8); t[3] = (uint8_t)a;
    memcpy(t + 4, d, n);
    FL_XFER(t, NULL, (uint8_t)(4u + n));
}

static void fl_read(uint32_t a, uint8_t *d, uint8_t n)         /* n <= 32 */
{
    uint8_t t[4 + 32] = { 0 }, r[4 + 32];
    t[0] = 0x03; t[1] = (uint8_t)(a >> 16); t[2] = (uint8_t)(a >> 8); t[3] = (uint8_t)a;
    FL_XFER(t, r, (uint8_t)(4u + n));
    memcpy(d, r + 4, n);
}

#if FLASH_RW_TEST
static int fl_rw_test(void)
{
    uint8_t pat[16], buf[16];
    for (int i = 0; i < 16; i++) pat[i] = (uint8_t)(0xA5u ^ (uint8_t)(i * 7));

    fl_wren();
    fl_erase4k(FLASH_TEST_ADDR);
    if (!fl_wait(1000)) return 0;

    fl_read(FLASH_TEST_ADDR, buf, 16);
    for (int i = 0; i < 16; i++) if (buf[i] != 0xFFu) return 0;     /* deve essere cancellato */

    fl_wren();
    fl_prog(FLASH_TEST_ADDR, pat, 16);
    if (!fl_wait(50)) return 0;

    fl_read(FLASH_TEST_ADDR, buf, 16);
    return memcmp(buf, pat, 16) == 0;
}
#endif

static int test_flash(void)
{
    uint8_t t[4] = { 0x9F, 0, 0, 0 }, r[4];
    FL_XFER(t, r, 4);                                    /* JEDEC ID */
    uint8_t mf = r[1], ty = r[2], cap = r[3];
    int id_ok = !(mf == 0x00 || mf == 0xFF);
    int pass = id_ok;

    P("[FLASH] JEDEC ID=%02X %02X %02X (Winbond: EF 40 18 oppure EF 70 18)  STATUS=0x%02X",
      mf, ty, cap, fl_status());

#if FLASH_RW_TEST
    static int rw_done = 0, rw_ok = 0;
    if (id_ok && !rw_done) {
        rw_ok = fl_rw_test();
        rw_done = 1;
    }
    if (id_ok) {
        P("  erase/write/read=%s", rw_ok ? "OK" : "ERRORE");
        if (!rw_ok) pass = 0;
    }
#endif
    P("  -> %s\r\n", pass ? "PASS" : "FAIL");
    if (!id_ok) {
        P("         ! %s\r\n", id_hint(mf));
    }
    return pass;
}

/* ------------------------------------------------------------ RADIO ------ */
static int rf_busy_wait(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    while (pin_get(RF_BUSY_PORT, RF_BUSY_PIN)) {
        if (HAL_GetTick() - t0 > ms) return 0;
    }
    return 1;
}

static int rf_xfer(const uint8_t *tx, uint8_t *rx, uint8_t n)
{
    if (!rf_busy_wait(50)) return 0;
    spi_xfer(RF_SPI, RF_NSS_PORT, RF_NSS_PIN, tx, rx, n);
    return 1;
}

static const char *rf_mode_name(uint8_t status)
{
    switch ((status >> 5) & 7u) {
    case 2:  return "STDBY_RC";
    case 3:  return "STDBY_XOSC";
    case 4:  return "FS";
    case 5:  return "RX";
    case 6:  return "TX";
    default: return "?";
    }
}

static int test_radio(void)
{
    uint8_t t[6], r[6];

    /* reset hardware: NRESET basso, poi BUSY deve tornare basso */
    pin_set(RF_RST_PORT, RF_RST_PIN, 0);
    HAL_Delay(3);
    pin_set(RF_RST_PORT, RF_RST_PIN, 1);
    HAL_Delay(3);
    int busy_ok = rf_busy_wait(100);

    /* GetStatus (0xC0): lo stato esce nel 2o byte */
    t[0] = 0xC0; t[1] = 0; t[2] = 0;
    memset(r, 0, sizeof(r));
    int st_ok = rf_xfer(t, r, 3);
    uint8_t st = r[1];

    /* ReadRegister (0x19) 0x0153..0x0154 = versione firmware del chip */
    t[0] = 0x19; t[1] = 0x01; t[2] = 0x53; t[3] = 0; t[4] = 0; t[5] = 0;
    memset(r, 0, sizeof(r));
    int fw_ok = rf_xfer(t, r, 6);
    uint16_t fw = (uint16_t)((r[4] << 8) | r[5]);

    int pass = busy_ok && st_ok && fw_ok && fw != 0x0000u && fw != 0xFFFFu;

    P("[RADIO] SX1280 FW_VERSION=0x%04X (tipico 0xA9xx)  BUSY dopo reset=%s  DIO1=%d  -> %s\r\n",
      fw, busy_ok ? "OK" : "TIMEOUT", pin_get(RF_DIO1_PORT, RF_DIO1_PIN), pass ? "PASS" : "FAIL");
    P("         GetStatus raw=%02X %02X %02X  modo=%s  cmd=%u\r\n",
      r[0], st, r[2], rf_mode_name(st), (unsigned)((st >> 2) & 7u));
    if (!pass) {
        if (!busy_ok)            P("         ! BUSY resta alto: controlla alimentazione, cristallo 52 MHz e NRESET\r\n");
        else if (fw == 0xFFFFu)  P("         ! 0xFFFF: MISO alto, nessuna risposta (NSS, SCK, MISO, alimentazione)\r\n");
        else if (fw == 0x0000u)  P("         ! 0x0000: MISO a massa o chip non pronto\r\n");
        else                     P("         ! risposta SPI incompleta o inattesa\r\n");
    }
    return pass;
}

/* -------------------------------------------------------------- LOOP ----- */
void bringup_loop(void)
{
    static uint32_t round = 0;

    /* si lavora solo con l'USB enumerata e il buffer di trasmissione libero */
    if (!usb_tx_idle()) {
        HAL_Delay(100);
        return;
    }

    rl = 0;
    rep[0] = '\0';
    round++;

    uint32_t t = HAL_GetTick();
    P("\r\n===== Kyuuby Deimos v1 - bring-up #%lu (t=%lu.%lu s) =====\r\n",
      (unsigned long)round, (unsigned long)(t / 1000u), (unsigned long)((t % 1000u) / 100u));

    int n_pass = 0;
    n_pass += test_mcu();
    n_pass += test_imu();
    n_pass += test_flash();
    n_pass += test_radio();

    P("[USB  ] CDC: OK (se leggi questo messaggio funziona)\r\n");
    P("RISULTATO: %d/4 test superati\r\n", n_pass);

    usb_flush();
    HAL_Delay(2000);
}
