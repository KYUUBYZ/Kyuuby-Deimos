# Kyuuby Deimos v1 - base test

Two files (`bringup.c`, `bringup.h`) to add to an STM32CubeIDE project, if you dont know how to use the STM32CubeIDE, search it up.
The test reads the MCU, IMU, flash and radio, and prints a report over the USB virtual serial port.

# 1. CubeMX / CubeIDE project

1. **New STM32 Project** -> select `STM32F405RGTx`.
2. **System Core -> RCC**: High Speed Clock (HSE) = `Crystal/Ceramic Resonator`.
3. **System Core -> SYS**: Debug = `Serial Wire` (if you leave it on "No Debug" you lose SWD access).
4. **Connectivity -> USB_OTG_FS**: Mode = `Device_Only`, and **untick "Activate_VBUS"**.
5. **Middleware -> USB_DEVICE**: Class For FS IP = `Communication Device Class (Virtual Port Com)`.
6. **Clock Configuration**: HSE input = 8 MHz, PLL source = HSE, HCLK = 168 MHz.
   Check that `48 MHz clocks (USB)` shows 48 (PLLM 8, PLLN 336, PLLP /2, PLLQ 7),
   APB1 = 42 MHz (/4), APB2 = 84 MHz (/2).
7. You do not need to enable SPI, GPIO or anything else: `bringup.c` configures them itself.
8. Generate the code.

2. Add the files : 

- `bringup.c` -> `Core/Src/`
- `bringup.h` -> `Core/Inc/`

In `Core/Src/main.c`:

```c
/* USER CODE BEGIN Includes */
#include "bringup.h"
/* USER CODE END Includes */

...
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  bringup_init();
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    bringup_loop();
  }
  /* USER CODE END 3 */
```

Build the project (hammer icon) and keep the `.elf` / `.bin` / `.hex` generated in `Debug/`.


3. Read the report

The board shows up as "STMicroelectronics Virtual COM Port".
Open the port with PuTTY, the Arduino Serial Monitor, `screen` or similar (the baud rate does not matter).
The report refreshes about every 2 seconds.

Example with everything working (numbers are indicative):

```
===== Kyuuby Deimos v1 - bring-up #3 (t=8.1 s) =====
[MCU  ] DEV_ID=0x413 (expected 0x413)  Flash=1024 kB  SYSCLK=168 MHz  PCLK1=42  PCLK2=84  -> PASS
[IMU  ] BMI270 CHIP_ID=0x24 (expected 0x24)  bad reads=0/100  after reset=0x24 ... -> PASS
[FLASH] JEDEC ID=EF 40 18 ...  erase/write/read=OK  -> PASS
[RADIO] SX1280 FW_VERSION=0xA9xx  BUSY after reset=OK  DIO1=0  -> PASS
[USB  ] CDC: OK
RESULT: 4/4 tests passed
```

Note: the report text printed by `bringup.c` is currently in Italian (for example
`RISULTATO`, `atteso`, `letture errate`). The output above is only a translation for
reference; the actual strings are the ones in the source file, im sorry, but it works better for me (as italian).

How to read a FAIL

| Line | Symptom | What to check |
|---|---|---|
| MCU | clock different from 168 MHz | HSE 8 MHz and PLL settings in CubeMX |
| IMU / FLASH | `0xFF` | the chip is not answering: CS, power supply, MISO |
| IMU / FLASH | `0x00` | MISO stuck low, or the chip is not powered |
| RADIO | BUSY TIMEOUT | power supply, 52 MHz crystal, NRESET |
| RADIO | FW_VERSION `0xFFFF` / `0x0000` | NSS, SCK, MISO, power supply |

A FAIL on one line does not block the others: each test is independent.

## Pins used

| Bus | Device | Pins |
|---|---|---|
| SPI1 | BMI270 (IMU) | PA5 SCK, PA6 MISO, PA7 MOSI, PA4 CS, PC4 INT1 |
| SPI2 | W25Q128 (flash) | PB13 SCK, PB14 MISO, PB15 MOSI, PB12 CS |
| SPI3 | SX1280 (radio) | PB3 SCK, PB4 MISO, PB5 MOSI, PA15 NSS, PB7 NRESET, PB6 BUSY, PB8 DIO1 |
