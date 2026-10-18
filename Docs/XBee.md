# XBee transparent UART DMA driver

The driver transports bytes. Telemetry generation, message framing, checksums,
timestamps, command definitions and command execution belong to the application.
It does not enter AT command mode, change radio configuration or use API frames.
The old placeholder ground-command enum/parser declarations have been removed.

## Board and CubeMX configuration

Confirmed wiring and baud:

| STM32 pin | Connection | Configuration |
| --- | --- | --- |
| PA0 | XBee DIN (module UART RX) | UART4 TX, AF8 |
| PA1 | XBee DOUT (module UART TX) | UART4 RX, AF8 |
| PA2 | XBee RESET (through-hole pin 5) | GPIO output **open drain**, initial high/released, no pull |
| PA3 | XBee ON/SLEEP (through-hole pin 13) | GPIO input, no pull; high = awake |

UART4 is asynchronous **115200 baud, 8 data bits, no parity, 1 stop bit**,
without RTS/CTS. No synchronous clock is needed.

PA3 is a status input, not SLEEP_RQ. `XBee_IsAwake()` reports this pin;
the current wiring cannot request sleep/wake through a GPIO. ON/SLEEP is also
not CTS, network association, or proof that the ground receiver is reachable.
For Zigbee firmware, configure `D9=1` to expose ON/SLEEP. Keep the radio awake
(`SM=0`) if continuous command reception is required. Sleep options depend on
the actual radio firmware and role.

The supplied 802.15.4 manual requires RESET to be driven only open drain/collector;
the module can pull RESET low itself. `GPIO_PIN_SET` releases the line through
the module's pull-up. The previous output push-pull/initial-low configuration
was changed in both `Documents.ioc` and generated initialization code.

| Request | DMA mapping | Mode | Priority |
| --- | --- | --- | --- |
| UART4_RX | DMA1 Stream 2, Channel 4 | Circular | High |
| UART4_TX | DMA1 Stream 4, Channel 4 | Normal | Low |

For both: peripheral increment disabled, memory increment enabled, byte memory
and peripheral alignment, FIFO disabled. UART4, DMA1 Stream2 and Stream4 NVIC
interrupts are enabled at preemption priority 1, subpriority 0. Existing SPI3
streams 0/5 and USART6 DMA2 Stream1 are separate.

These settings are already reflected in the `.ioc`, `main.c`, MSP and IRQ files.
After future CubeMX regeneration, verify they persist. Keep UART4 and its RX DMA
at equal preemption priority: their receive callbacks must not nest. Keep RX
half-transfer and transfer-complete interrupts enabled. UART4 IRQ is needed for
IDLE reception events and final UART TX completion, even with DMA enabled.
The global HAL callbacks dispatch to both GNSS and XBee in `GNSS_HAL.c`; do not
define another copy in application code. Each driver checks the UART handle.

## Application use

Initialize after `MX_GPIO_Init`, `MX_DMA_Init` and `MX_UART4_Init`:

```c
#include "drivers/XBee.h"
extern UART_HandleTypeDef huart4;

XBeeResult result = XBee_Init(&huart4);
/* Check result. On an RX-start IO error, use XBee_Recover(). */
```

Call `XBee_Process()` in the main loop. Startup and reset use a 500 ms local guard;
transmission is then accepted only while ON/SLEEP is high. This guard is a driver
policy, not a bound on the time needed to join a Zigbee network.

To send an application-built telemetry message:

```c
XBeeResult result = XBee_Send(telemetry, telemetry_length);
if (result == XBEE_OK) {
    /* Source buffer can now be reused. */
} else if (result == XBEE_BUSY || result == XBEE_NOT_READY) {
    /* Keep the pending message and retry in a later loop iteration. */
} else {
    /* Record/report the error; do not assume delivery. */
}
```

`XBee_Send` copies 1..256 bytes into a persistent SRAM buffer, starts normal TX
DMA, and returns without waiting for the transfer. A 153-byte call transmits
exactly 153 bytes, not the entire 256-byte capacity. There is one outstanding
transfer and no hidden TX queue. `XBee_SendString` omits the terminating NUL and
does not add a newline. `XBee_IsTxBusy()` becomes false at final UART completion;
`txCompleted` is **not a radio-delivery acknowledgement**. After a UART/DMA error,
busy can remain set until recovery finishes, protecting the DMA buffer.

The application's fixed-width binary telemetry is exactly 153 bytes and fits
the 256-byte TX capacity. RX DMA is also 256 bytes.
See [communication and the telemetry schema](Communication.md).
UART writes are byte streams, not guaranteed one-to-one RF frames.

At 115200 baud with 8N1, 153 bytes take `153 * 10 / 115200 = 13.28 ms` on the
UART wire. CPU work is the short copy/setup and interrupts, not that whole
13.28 ms. RF packetization, channel access and retries add separate delays.

To collect received bytes:

```c
XBee_Process();
uint8_t chunk[128];
uint16_t count;
XBeeResult result = XBee_Read(chunk, sizeof(chunk), &count);
if (result == XBEE_OK) {
    /* Feed these count bytes into the application's persistent command parser.
       count==0 means no queued bytes; the function does not wait. */
} else if (result == XBEE_RX_LOST) {
    /* Discard any partial command and resynchronize at a framing boundary. */
} else if (result == XBEE_IO_ERROR) {
    /* Log the error, request XBee_Recover once, then keep calling Process. */
}
```

RX uses a separate 256-byte circular DMA buffer and a 2048-byte software queue.
HT, TC and IDLE callbacks copy new bytes into that queue. The application reads
them later; it never processes memory currently being overwritten by DMA.
Callbacks use the current DMA counter rather than a possibly stale event size.
An IDLE event only means a UART gap; it is not a complete-command indicator.

The queue holds about 178 ms of continuous full-rate input at 115200 baud, or
many small, infrequent ground commands. Drain it frequently when practical,
independently of the telemetry/logging schedule. Overflow stops queueing until
`XBee_Read` flushes the queue and returns `XBEE_RX_LOST`; this prevents joining
bytes across a known gap. The application must also reject the partial command
it has already accumulated. Diagnostics expose drops, loss events and errors.

RX interrupts must be serviced within the 128-byte half-buffer interval, about
11.1 ms at 115200 baud, with enough margin to copy the data. Do not mask interrupts
or run higher-priority ISRs for that long. A complete DMA lap while interrupts are
blocked cannot be reliably detected from the modulo DMA position. A blocked main
loop with interrupts still enabled is protected by the software queue, up to its
capacity. DMA buffers are static and must remain in DMA-accessible SRAM, not CCM.

## Reset and recovery

- `XBee_Reset()` refuses during active TX. Otherwise it asynchronously stops UART
  DMA, asserts RESET low for at least 2 ms (manual minimum 200 ns), releases it,
  restarts RX and observes the boot guard. `XBee_Process()` advances these steps
  using timestamps; a slow loop lengthens the pulse/guard without blocking.
- `XBee_Recover()` asynchronously aborts TX/RX and restarts reception after an
  error. It can interrupt an active message. It does not reset the radio or replay
  telemetry/commands. Abort completion comes through the shared HAL dispatcher.
- Both operations discard queued RX and report a stream discontinuity through
  `XBEE_RX_LOST`. Call once, then service `XBee_Process()` while status is
  `XBEE_RECOVERING`/`XBEE_BOOTING`. An error during recovery leaves `XBEE_FAULT`.
- `XBee_GetDiagnostics()` snapshots counters and the pending RX size. UART/DMA
  error callbacks only record the fault; they do not parse commands or retry.

All public driver functions are for a single main-loop caller, not ISR use.
No RTOS is required. Initialization does not automatically reset the module.

## Transparent radio configuration and message boundaries

Configure both radios in XCTU for a matching network/destination and transparent
mode (`AP=0`), 115200 (`BD=7`), no parity, one stop bit. Use firmware-specific
network settings: the two supplied guides describe different S2C firmware families
(802.15.4 versus Zigbee). The UART transport driver works with either, but their
network roles and radio settings are not interchangeable. With RTS unwired, do
not configure the radio to wait for an external RTS signal.

A 153-byte telemetry record can span multiple RF packets. The remote UART may
deliver it in several chunks, or adjacent records in one chunk. The application
must frame records, for example with newline-delimited CSV or a length/checksum
protocol. Include sequence numbers and application acknowledgements/timeouts if
ground commands require confirmed execution. UART DMA length does not establish
wireless message boundaries.

There is no wired CTS flow control. Busy handling protects the STM32 TX buffer,
not the radio's serial input buffer. Do not send large back-to-back bursts merely
because UART TX completed; validate the intended telemetry schedule over the
actual radio link. For context, the supplied S2C 802.15.4 guide describes only
174 bytes of serial input capacity. Radio-buffer overflow/RF loss is not visible
in the STM32 driver's counters in transparent mode. The low telemetry frequency
provides idle time, but does not establish a delivery guarantee during RF stalls.
Avoid transmitting the radio's guarded `+++` command-mode escape sequence as
test payload unless entering AT mode is intentional.

## Functional test

Build the isolated test (the flight scaffold does not automatically start XBee):

```powershell
cmake --preset TestXBee
cmake --build --preset TestXBee
```

Flash `build/TestXBee/test_xbee.elf`. Connect a **3.3 V USB/UART adapter** to PC10
(STM32 TX -> adapter RX), PC11 (STM32 RX <- adapter TX) and ground. Open a serial
monitor at 115200 8N1, no flow control, with CR, LF or CRLF line endings. USART3 is
a test console; USART6 remains assigned to GNSS. The shared `Tests/Console`
helper provides bounded blocking console TX and interrupt-driven console RX;
XBee TX/RX both use DMA. The Nucleo ST-LINK virtual COM interface can serve as
the adapter. See [console usage and bounds](../Tests/README.md#shared-test-console).

1. Pair/configure the other XBee and open its ground-side UART monitor.
2. At startup, once awake and after the guard, the STM32 sends a fixed **153-byte**
   message beginning `XBEE_TEST_153_BYTES,`, padded with digits and ending CRLF.
   Verify the full record arrives at the ground radio. `/test` repeats it.
3. Enter text on the STM32 console. The test sends that line with CRLF; verify it
   at the ground radio. Limit: 254 text bytes plus CRLF. One pending line is kept
   while asleep/busy; extra lines are rejected with a console message.
4. Send text from the ground radio. The STM32 monitor prints `RX chunk:` followed
   by received bytes. Multiple chunks for one line are expected.
5. `/status` shows ON/SLEEP, driver state, TX started/completed/aborted counts, RX
   bytes/drops, UART errors and console drops. Status also prints every 2 seconds.
6. `/reset` exercises the reset state machine; `/recover` restarts UART reception
   after an error. These commands cancel an unsent local test message. `/help`
   prints the local test commands; they are not ground-command definitions.

For a loss/throughput test, compare numbered messages at both ends. Check driver
and console drops separately. Actual RF delivery and GPIO polarity/timing still
require this hardware test; host tests cannot establish them.

Host regression:

```powershell
powershell -ExecutionPolicy Bypass -File Tests/host/run_xbee_tests.ps1 -Compiler C:/MinGW/bin/gcc.exe
powershell -ExecutionPolicy Bypass -File Tests/host/run_gnss_tests.ps1 -Compiler C:/MinGW/bin/gcc.exe
```

The XBee suite covers copied TX-buffer lifetime, busy/invalid/error paths, exact
153-byte transfers, partial reads, circular RX wraps, duplicate/out-of-order
event sizes, overflow flush, callback handle filtering, awake/asleep gating,
asynchronous abort/recovery and reset timing across millisecond-counter rollover.

## References

- Supplied `XBee_Techsupport_document.pdf`: Digi 90001500 P, S2C 802.15.4, pin
  signals p35, flow control p66, sleep pins p77, baud configuration p102.
  [Digi publication](https://docs.digi.com/resources/documentation/digidocs/pdfs/90001500.pdf).
- Supplied `XBee_User_Guide.pdf`: Digi 90002002 AP, S2C Zigbee, serial/transparent
  mode p60-61, baud p268, D9 ON/SLEEP p279.
- STM32F405 RM0090 DMA1 request mapping and the project's STM32F4 HAL UART/DMA
  implementations for ReceiveToIdle, final UART TC and asynchronous abort.
