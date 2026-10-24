/* Arduino IDE: board Arduino Uno (ATmega328P, Uno R3), Serial Monitor 115200.
 * Also works on classic ATmega328P Nano; NOT Uno R4.
 * D11 MOSI, D12 MISO, D13 SCK, D10 SS. See Docs/SPI_Test.md for level shifting.
 * Slave mode 0, MSB first. Do not call SPI.begin(): that configures a master.
 * STM32 inserts 1 ms between bytes so the ISR can preload the echo. */
#include <Arduino.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#if !defined(__AVR_ATmega328P__)
#error "Select Arduino Uno / classic Nano with ATmega328P (not Uno R4)."
#endif

static volatile char line[128];
static volatile uint8_t length;
static volatile bool ready, overflow;
static volatile uint16_t byteCount;

ISR(SPI_STC_vect)
{
    uint8_t byte = SPDR;
    SPDR = byte; /* Returned to master on NEXT byte, not the current byte. */
    ++byteCount;
    if (byte == 0 || ready) return; /* Ignore master's trailing dummy byte. */
    if (length < sizeof(line) - 1U) line[length++] = (char)byte;
    else overflow = true;
    if (byte == '\n') {
        line[length] = '\0';
        ready = true;
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(MOSI, INPUT);
    pinMode(SCK, INPUT);
    pinMode(SS, INPUT); /* External pull-up to 3.3V on STM32 CS recommended. */
    pinMode(MISO, OUTPUT);
    SPCR = _BV(SPE) | _BV(SPIE); /* MSTR=0 CPOL=0 CPHA=0 DORD=0. */
    SPDR = 0;
    Serial.println(F("UNO SPI slave ready: mode 0, MSB first. Waiting for STM32..."));
}

void loop()
{
    char copy[128];
    bool haveLine = false, truncated = false;
    uint16_t count;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        count = byteCount;
        if (ready) {
            for (uint8_t i = 0; i <= length; ++i) copy[i] = line[i];
            truncated = overflow;
            length = 0; overflow = false; ready = false;
            haveLine = true;
        }
    }
    if (haveLine) {
        Serial.print(truncated ? F("RX TRUNCATED: ") : F("RX: "));
        Serial.print(copy);
        Serial.print(F("SPI bytes received (16-bit counter): "));
        Serial.println(count);
    }
}
