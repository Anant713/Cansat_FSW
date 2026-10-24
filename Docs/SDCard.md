# SD card driver: polling SPI3

All SD operations are synchronous and use polling. There are no SD DMA
callbacks, transfer launches or DMA buffers.

## Integration

Application -> SDCard.c -> sd_mount/sd_unmount and FatFs file functions ->
directly registered SD_Driver -> sd_diskio_spi.c -> supplied sd_spi.c -> HAL polling SPI3.

The supplied files were adapted for:
- hspi3; PB2 CS, initially high and asserted low during transactions.
- Initialization at <=400 kHz, then <=8 MHz, calculated from PCLK1. At
  PCLK1=16 MHz these are 250 kHz and 8 MHz.
- Finite HAL transfer and card-busy/token waits, with HAL errors propagated.
- CMD0 is issued without a prior SPI-ready requirement: it establishes SPI
  mode. Subsequent commands still check ready before transmission.
- CSD-derived capacity instead of the example's fixed 32 MB.
- SDSC byte addressing/CMD16 and SDHC sector addressing.
- Repeated single-sector transactions for multi-sector requests, avoiding
  the supplied CMD12/CMD25 stop/recovery issues. This trades throughput for
  simpler polling transfers.
- One directly registered FatFs SD_Driver, including after unmount/remount.
- Accurate disk status and CTRL_SYNC, const-correct write buffers.
- Text-helper input validation and reporting of close/flush failures.

SPI3: PB3 SCK, PB4 MISO, PC12 MOSI, PB2 GPIO CS. Use master full duplex,
mode 0, 8-bit, MSB first, software NSS, CRC disabled. No .ioc regeneration is
needed for polling. Existing SPI DMA configuration can remain initialized but
is unused by these SD routines; do not remove unrelated UART DMA settings.

MX_GPIO_Init and MX_SPI3_Init must precede SDCard_Init. The generated project
may still initialize DMA for other peripherals. sd_mount links SD_Driver once
using FATFS_LinkDriver. Keep CubeMX's "Do Not Generate Function Call" checked
for MX_FATFS_Init; do not call it manually. The generated function may remain
in fatfs.c without being used. An already registered different driver returns
FR_NOT_ENABLED instead of replacing it or attempting duplicate registration.

Keep HAL tick and interrupts enabled. Call from the main loop, not an ISR;
use one caller. Polling occupies the main loop while commands, bytes and card
programming complete. Low average logging frequency does not eliminate
occasional card latency: measure write + sync duration with the actual card.

## Application use

```c
#include "drivers/SDCard.h"

if (!SDCard_Init() || !SDCard_OpenAppend("0:/flight.csv")) {
    FRESULT error = SDCard_GetLastResult();
    (void)error; /* Record/report according to application policy. */
    return;
}
if (!SDCard_WriteString("1234,25.0,101325\r\n") || !SDCard_Sync()) {
    FRESULT error = SDCard_GetLastResult();
    (void)error;
}
```

SDCard keeps a FIL open for logging. CreateFile refuses to overwrite an
existing file; OpenAppend preserves its contents. Write accepts binary bytes;
ReadFile reads from offset zero without adding a NUL terminator. Sync flushes
its output file. CloseFile and DeInit must be checked before removal.
A successful Write alone may leave bytes in FatFs's sector cache.

The text convenience functions in sd_functions remain available for callers
that want open/write/close per call. SDCard uses their mount lifecycle but
retains FatFs file operations for persistent, binary-safe logging rather than
substituting the text-only helpers. Do not independently unmount a volume
while SDCard owns an open file.

After an error, save SDCard_GetLastResult before calling SDCard_DeInit:
cleanup can replace the last result. DeInit releases stale handles and clears
card initialization; Init then performs physical initialization again.
There is no automatic formatting. Use FAT16/FAT32 media, not exFAT.

## Test and benchmark

### Viewing SPI diagnostics

Use the STM32 test console on USART3: PC10 TX -> adapter RX, PC11 RX <-
adapter TX, common ground, 3.3 V logic, 115200 baud, 8N1. Set the serial
monitor line ending to LF or CRLF. In TestSD or TestSDText, send `diag` or
`status` followed by Enter. The printed `SPI init_stage=... CMD=...` line is
the diagnostics. No debugger is needed, and opening the monitor after startup
is fine: the command repeats the values. Values describe the last operation,
not an ongoing SPI trace; token specifically records the initialization CSD token.

To inspect in a debugger, load the same Debug-built ELF that was flashed and
start a debug session. After the test finishes, pause execution (or break in
App_RunOnce). Add `sd_spi_diagnostics` in Watch and expand it. Individual
expressions such as `sd_spi_diagnostics.init_stage` and
`sd_spi_diagnostics.response` also work. These are global variables, not local
variables of main. A plain serial connection does not provide debugger access.
If the symbol is missing, check the selected ELF/preset and that the firmware
contains the updated SD source. Pause after IO, not in the middle of a transfer.

### CSV open/close/append test

Build the TestSDText preset and flash `build/TestSDText/test_sd_text.elf`.
Source: `Tests/test_sd_text.c`. Do not add its App_Init/App_RunOnce alongside
another application's hooks; the preset selects it automatically.

The test creates a new `SDC0000.CSV` (or next unused number), writes a header
and first row, closes, opens for reading and verifies, closes, reopens for
append, adds a row and closes again. It unmounts/remounts, verifies the entire
file, then unmounts. Existing files are never overwritten. Expected text:

```csv
time_ms,event
0,opened
1000,appended
```

These are fixed test values, not measured timestamps. The monitor shows each
step followed by `SD TEXT PASS state=2 stage=12 FRESULT=0 file=...` on success.
Commands: `status`, `diag`, `run` (creates another new file), `help`.
After PASS, the volume is unmounted; stop/reset the board before removing the
card and open the CSV on a computer. Do not send `run` while the card is removed.
Debugger variables: sd_text_state, sd_text_stage, sd_text_result,
sd_text_filename and sd_text_readback.

### Binary verification test

Build/flash TestSD (build/TestSD/test_sd.elf). The functional test creates a new
SDTnnnn.BIN, writes 1537 bytes, appends 37, remounts and compares all 1574 bytes.
Inspect sd_test_state (2 pass, -1 fail), sd_test_stage and sd_test_result.
This validates SD commands/filesystem operations that the Uno SPI test bypasses.

TestSD also prints sd_spi_diagnostics, available directly in the debugger.
init_stage: 1=preconditions, 2=startup clock, 3=CMD0, 4=CMD8, 5=CMD55/ACMD41,
6=CMD58/OCR, 7=CMD16, 8=CMD9/CSD, 9=run clock, 10=initialization succeeded.
command/response record the latest command and R1. command_sent=0 indicates
wait-ready failed before command transmission was attempted. token records the
CSD data token; R7/OCR retain the returned register values. hal_status retains
the first HAL failure since initialization (0=OK, 1=ERROR, 2=BUSY, 3=TIMEOUT).
Cleanup after a failed mount does not clear these diagnostics. Command fields
continue updating during subsequent successful file IO; init_stage stays at 10.

A failure at the explicit SD_disk_initialize call happens before f_mount and
does not depend on whether the direct driver or a forwarding USER_Driver was
registered. Registration changes alone do not diagnose that physical failure.

Host regression:
```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tests/host/run_sd_tests.ps1
```
Requires GCC on PATH. The polling SPI simulator checks startup speed,
SDHC/SDSC addressing, capacity, multiple sectors, HAL errors, busy timeout
across tick rollover, and operation without DMA handles. The filesystem
suite uses the bundled FatFs with an in-memory disk, not a physical card.

sd_benchmark is an opt-in polling throughput measurement. It creates bench.bin
with FA_CREATE_NEW, so delete/rename an old benchmark file deliberately before
rerunning. It includes close time and returns zero on incomplete IO/close
failure. Its fixed-pattern timing test does not verify payload integrity;
use TestSD for readback comparison. It is never called automatically in flight.

References:
- https://elm-chan.org/docs/mmc/mmc_e.html
- https://elm-chan.org/fsw/ff/doc/dioctl.html
