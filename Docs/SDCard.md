# SD card driver integration

The project now uses this call chain:

```text
Flight/test app
  -> drivers/SDCard.c or sd_functions.c (file operations)
  -> FatFs f_open/f_write/f_read/f_sync/f_close
  -> CubeMX USER_Driver in FATFS/Target/user_diskio.c
  -> SD_disk_* in sd_diskio_spi.c
  -> SD_ReadBlocks/SD_WriteBlocks in sd_spi.c
  -> SPI3 + DMA1
```

`sd_spi.c` already IS the SPI SD driver. Do not call FatFs or `sd_write_file()`
from its low-level transfer functions: that would recurse back into the same
driver. Use `SDCard.c` as the public driver facade for the flight application.
The source files are included in the top-level CMake target for all presets.

## Wiring and CubeMX

| Function | Configuration |
| --- | --- |
| SD SCK | PB3, SPI3 SCK, AF6 |
| SD MISO | PB4, SPI3 MISO, AF6 |
| SD MOSI | PC12, SPI3 MOSI, AF6 |
| SD CS | PB2, `CS`, GPIO push-pull output, initial HIGH |
| SPI | Full duplex master, 8-bit, MSB first, mode 0, software NSS, hardware CRC disabled |
| SPI3 RX DMA | DMA1 Stream0, Channel0, peripheral-to-memory |
| SPI3 TX DMA | DMA1 Stream5, Channel0, memory-to-peripheral |
| Both DMA streams | Normal mode, byte alignment, memory increment ON, peripheral increment OFF, FIFO OFF |
| DMA IRQs | Both enabled, preemption priority 6 |
| SPI3 global IRQ | Enabled, preemption priority 6, calls `HAL_SPI_IRQHandler(&hspi3)` |
| HAL tick (TIM6) | Priority 5, higher urgency than SPI/DMA; `TICK_INT_PRIORITY=5` |

The existing DMA channel/stream choices were correct. SPI3 error IRQ forwarding
and tick priority were added. HAL's DMA completion routine can itself wait for
SPI flags with a tick timeout, so TIM6 must be able to preempt it. Keep interrupts
and the HAL tick running during every SD operation. Do not call SD/FatFs from an
ISR, interrupt-disabled region, or competing callers. `_FS_REENTRANT` is disabled.

`MX_DMA_Init()` must run before `MX_SPI3_Init()`; the generated main already does.
`sd_mount()` lazily calls `MX_FATFS_Init()` if no driver is registered. It also
works if CubeMX calls `MX_FATFS_Init()` once during startup. Do not call it again
manually or separately link `SD_Driver`: only one volume is configured.

The driver selects an SPI divisor giving at most 400 kHz for initialization,
then at most 8 MHz after success. With the current 16 MHz PCLK1 this is 250 kHz
(prescaler 64) and 8 MHz (prescaler 2). It recalculates from PCLK1 when initialized.
Reinitialize after any clock-tree change.

The attached BerryBase MSD-AADP datasheet specifies **4.5–5.5 V module power**, an
onboard 3.3 V regulator, SPI, SDSC up to 2 GB, and SDHC up to 32 GB. Use a common
ground and FAT16/FAT32 media; this build has no exFAT support. The supplied sheet
does not establish the module's SPI input thresholds. Verify 3.3 V MCU signal
compatibility and a MISO pull-up in the actual module/wiring. Module VCC and SD
card logic voltage are different considerations.

## Application use

Call after generated GPIO, DMA and SPI initialization, e.g. from `App_Init()`:

```c
#include "drivers/SDCard.h"

if (!SDCard_Init() || !SDCard_OpenAppend("0:/flight.csv")) {
    FRESULT reason = SDCard_GetLastResult();
    /* Record reason / disable logging according to application policy. */
    (void)reason;
    return;
}
if (!SDCard_WriteString("time,altitude\r\n") || !SDCard_Sync()) {
    /* Inspect SDCard_GetLastResult(); do not assume the row is durable. */
}
```

Write the CSV header only when appropriate for a new flight/file; OpenAppend
preserves existing content. `SDCard_CreateFile()` uses `FA_CREATE_NEW` and returns
`FR_EXIST` instead of overwriting an existing file. Only one output file can be
open through this facade. `SDCard_Write(data, length)` supports binary data.

`SDCard_ReadFile(name, buffer, capacity, &bytes_read)` opens a file for reading
at offset zero and closes it after the call. It does not add a string terminator;
check `bytes_read`, since EOF can produce a successful short read. Close an output
file before reading that same file (FatFs file locking is enabled).

During logging, use `SDCard_Sync()` at an application-chosen interval. At shutdown,
check `SDCard_CloseFile()` and `SDCard_DeInit()` results before removing power.
FatFs can cache partial sectors, so Write success alone is not a durability
guarantee. DMA completion is only a bus-transfer event, not file completion.

All these operations **block until completion**. Sector payloads use DMA, but
commands, token waits, and card programming still take time in the calling task.
The 100 ms transfer timeout and 1 s ready/token timeout bound individual waits;
an entire multi-sector file operation can take longer. Do not treat IsBusy as an
asynchronous queue. A future nonblocking logger needs a queue and a dedicated
task/state machine; it cannot return from FatFs disk_read/write before completion.

After an I/O failure, `SDCard_DeInit()` releases stale file/mount state even if
closing failed (and returns false to report the failure). Then retry Init and
reopen the file. Removal has no dedicated detect input; status reflects successful
initialization until an operation detects an error. There is no automatic format.
Do not independently unmount through `sd_functions` while using an SDCard file.

The convenience functions remain usable directly:

```c
#include "sd_functions.h"

if (sd_mount() == FR_OK) {
    int result = sd_append_file("0:/notes.txt", "Hello\r\n");
    /* result is an FRESULT code; each helper opens and closes its file. */
    (void)result;
    (void)sd_unmount();
}
```

`sd_write_file()` replaces the named file; `sd_append_file()` preserves it.
`sd_read_file()` is a text helper: pass capacity including the NUL terminator.
The CSV helper handles only unquoted `field1,field2,integer` records, no header.
Directory listing is bounded to four levels and rejects truncated paths.
Diagnostic printing requires a retargeted `__io_putchar`; otherwise `_write`
returns an error instead of calling an undefined weak function. Normal mount,
read/write and SDCard APIs do not depend on printf.

## Implementation details and regeneration

- `sd_spi.c` owns `HAL_SPI_TxRxCpltCallback` and `HAL_SPI_ErrorCallback` for SPI3.
  If adding other SPI drivers, dispatch from these existing callbacks instead
  of defining another function with the same name.
- Both read and write payloads use full-duplex DMA, draining RX during writes.
  Two static 512-byte staging buffers reside in DMA-accessible SRAM. Caller data
  is copied, so buffers in CCM are safe; do not move staging buffers into CCM.
- SDSC uses byte addresses and CMD16 for 512-byte blocks. SDHC uses sector
  addresses. Capacity comes from CSD, not the tutorial's fixed 32 MB placeholder.
- CMD12 stuff/busy handling, multi-write stop tokens, timeouts, HAL launch errors,
  and DMA error callbacks are handled. A failed transfer invalidates card status.
- Disk `CTRL_SYNC` actually waits for the card. File helpers propagate short
  writes and close/flush errors.
- The linker and IOC reserve 8 KiB stack for FatFs file objects, bounded directory
  recursion and IRQ headroom. `_USE_LFN=3` uses the existing malloc/free hooks.
- No RTC is configured; file timestamps are fixed to 2026-01-01, not real time.
- Disk glue and the timestamp change are inside CubeMX USER CODE sections.
  NVIC settings and stack reservation are reflected in Documents.ioc. After
  regeneration check `TICK_INT_PRIORITY=5` and the table above. Keep the custom
  source list in the top-level CMake file.

## Verification

Host tests use MinGW GCC and run without a board:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tests/host/run_sd_tests.ps1
```

They exercise wire-level SD sequences against a HAL simulator, including SDSC,
SDHC, multi-block termination, DMA errors, lost interrupts and tick rollover.
A separate test runs the actual bundled FatFs against an in-memory block device
to check file operations, remounts, long names, validation and flush failures.
The only formatted medium in these tests is the in-memory array.

Build the on-board test in the STM32 toolchain environment:

```powershell
cmake --preset TestSD
cmake --build --preset TestSD
```

Flash `build/TestSD/test_sd.elf` manually and inspect `sd_test_state`: 2 means
passed, -1 means failed. On failure inspect `sd_test_stage` and `sd_test_result`.
It creates a new `SDTnnnn.BIN`, writes 1537 bytes, appends 37 bytes, remounts,
then compares all 1574 bytes. Existing files are preserved. Inspect that file on
a computer and verify 250 kHz initialization and 8 MHz run clocks with this
project's current 16 MHz PCLK1.
No hardware test has been run by this change.

`sd_benchmark()` is a separate opt-in diagnostic, never called by flight startup.
It creates `bench.bin` without overwriting, verifies every byte, includes close
time in its write timing, and suppresses speed results on failure.

References: [SPI SD initialization and transfers](https://elm-chan.org/docs/mmc/mmc_e.html),
[FatFs disk_write contract](https://elm-chan.org/fsw/ff/doc/dwrite.html),
[FatFs disk_ioctl contract](https://elm-chan.org/fsw/ff/doc/dioctl.html).
