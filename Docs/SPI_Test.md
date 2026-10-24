# SPI3 board test with Arduino Uno

This test bypasses SDCard, SD initialization and FatFs. It checks SPI3 mode 0,
MOSI, MISO, SCK and software CS on the same board pins as the SD module.
The SD sources remain linked but the test does not call them.

## Wiring

Disconnect the SD module before attaching the Uno: both would otherwise share
CS and drive MISO simultaneously. Power each board normally and connect grounds.
Do not connect the two boards' supply rails together.

| STM32 | Direction | Uno R3 / classic Nano (ATmega328P) |
| --- | --- | --- |
| PC12 / SPI3 MOSI | STM32 -> Uno | D11 / MOSI |
| PB4 / SPI3 MISO | Uno -> STM32 | D12 / MISO |
| PB3 / SPI3 SCK | STM32 -> Uno | D13 / SCK |
| PB2 / CS | STM32 -> Uno | D10 / SS |
| GND | Common | GND |

Use a directional SPI-compatible level translator: three signals from 3.3 V to
5 V (MOSI/SCK/CS), one from 5 V to 3.3 V (MISO). Uno R3 uses 5 V logic; do not
assume a bare 5 V MISO connection is suitable for the STM32 board or any components
on it. A 3.3 V input into a 5 V receiver also needs its input thresholds checked;
level translation removes that uncertainty from the diagnosis. Avoid treating an
I2C-only open-drain level shifter as a guaranteed push-pull SPI translator.
Keep wires short. A 10 kohm pull-up from STM32 CS to 3.3 V keeps CS inactive during
MCU reset. The Uno sketch does not enable an internal 5 V SS pull-up.

Official Uno pin/voltage reference:
https://store.arduino.cc/products/arduino-uno-rev3
AVR SPI reference:
https://ww1.microchip.com/downloads/en/Appnotes/Atmel-2585-Setup-and-Use-of-the-SPI_ApplicationNote_AVR151.pdf

## Arduino IDE

1. Open `Tests/arduino/uno_spi_slave/uno_spi_slave.ino`.
2. Select Arduino Uno (Uno R3 / ATmega328P) and its USB port. This register-based
   sketch is not for Uno R4. A classic ATmega328P Nano also works.
3. Upload, then open Serial Monitor at **115200 baud**.
4. Expect `UNO SPI slave ready...`, followed by lines such as:

```text
RX: Hello Arduino UNO! STM32 SPI3 test packet 1.
SPI bytes received (16-bit counter): ...
```

Printing happens in the Arduino loop, never in its SPI interrupt. The master
sends an extra NUL after each sentence, which is not printed. The byte counter
includes that dummy byte and wraps at 65536. Opening Serial Monitor resets many
Uno boards, so an initial partial line/failed packet can occur; judge subsequent
packets after startup.

## STM32

Build the **TestSPI** preset and flash `build/TestSPI/test_spi.elf`:

```powershell
cmake --preset TestSPI
cmake --build --preset TestSPI
```

`Tests/testspi.c` provides the normal `App_Init` and `App_RunOnce` hooks; do not
replace `main.c`. No `.ioc` changes are required for this test. It checks the
existing SPI configuration and changes only the test clock divider to /256.
The resulting SCK is `PCLK1 / 256`, reported on the console. It inserts 1 ms
between bytes to allow the Uno's software ISR to prepare its next response.

An automatic sentence is sent every two seconds. Connect the existing STM32 test
console (USART3 PC10 TX -> USB/UART RX, PC11 RX <- USB/UART TX, common ground,
3.3 V logic), at **115200 8N1**. It prints:

```text
SPI PASS packet=1 HAL=0 error=0x00000000 echo mismatches=0 SCK=... Hz
TX: Hello Arduino UNO! STM32 SPI3 test packet 1.
```

Type `send` + Enter for an immediate packet, or `status` + Enter for counters.
The console is optional: automatic transmissions run regardless, and the debugger
can inspect `spi_test_state` (2 = last packet passed, -1 = failed),
`spi_test_passes`, `spi_test_failures`, `spi_test_hal_status`,
`spi_test_hal_error`, `spi_test_mismatches`, `spi_test_clock_hz`, and `spi_test_rx`.

The Uno loads each received byte into its SPI transmit register, so it returns
that byte during the **next** transfer. The first received byte is ignored and
one final dummy transfer retrieves the last character. STM32 compares every
sentence byte, including the newline. CS remains low for the complete sentence
plus dummy byte, then returns high even if a HAL transfer fails.

## Interpreting results

| Observation | What it narrows down |
| --- | --- |
| Correct Uno sentence and STM32 PASS repeatedly | Basic SPI pins, bidirectional communication, software CS and mode work at this test rate |
| Correct Uno sentence, STM32 echo mismatch | Forward path works; investigate MISO wiring/translation, Uno echo timing and receive path |
| Uno prints nothing, HAL reports OK | HAL success alone does not prove a slave is connected; check CS at Uno D10, clock, MOSI, ground, power and sketch |
| Garbled Uno text | Check mode, signal levels, wiring, clock integrity; occasional corruption may also be ISR timing |
| HAL timeout/error | Inspect STM32 SPI state/error, generated setup and peripheral operation |
| Configuration/init FAIL | Test rejected the generated mode/format or HAL initialization failed; no packets are sent |

A PASS does **not** validate SPI DMA, SD command sequencing, the card socket or
its module-level signal/power path, FAT32, or operation at the faster SD data
clock. The Uno's interrupt-based echo is intentionally slow and is not a way to
qualify 8 MHz transfers. After this passes, reconnect the SD module and locate
the failing SD initialization command; if commands work but CSD/block DMA fails,
investigate DMA completion and IRQ configuration next. Do not conclude that the
SD driver is faulty solely because this low-speed test passes.
