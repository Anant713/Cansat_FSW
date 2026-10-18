#include "drivers/XBee.h"
#include "main.h"
#include <stddef.h>
#include <string.h>

#define QUEUE_MASK (XBEE_RX_QUEUE_SIZE - 1U)
_Static_assert((XBEE_RX_QUEUE_SIZE & QUEUE_MASK) == 0, "RX queue must be a power of two");
_Static_assert(XBEE_RX_QUEUE_SIZE >= XBEE_RX_DMA_SIZE, "Queue too small");

static UART_HandleTypeDef *port;
static uint8_t dma_rx[XBEE_RX_DMA_SIZE] __attribute__((aligned(4)));
static uint8_t dma_tx[XBEE_TX_CAPACITY] __attribute__((aligned(4)));
static uint8_t queue[XBEE_RX_QUEUE_SIZE];
static volatile uint32_t head, tail;
static uint16_t dma_position;
static volatile bool receiving, tx_busy, fault, rx_loss, abort_done;
static volatile XBeeDiagnostics diagnostics;
typedef enum { RUNNING, STOPPING, RESET_LOW, BOOT_WAIT } Phase;
static volatile Phase phase;
static bool reset_requested;
static uint32_t phase_started;

static uint32_t lock(void) { uint32_t p = __get_PRIMASK(); __disable_irq(); return p; }
static void unlock(uint32_t p) { __set_PRIMASK(p); }
static void mark_loss(void)
{
    if (!rx_loss) ++diagnostics.rxLossEvents;
    rx_loss = true;
}
static bool dma_valid(const DMA_HandleTypeDef *dma, uint32_t mode, uint32_t direction)
{
    return dma != NULL && dma->Init.Mode == mode && dma->Init.Direction == direction &&
        dma->Init.MemInc == DMA_MINC_ENABLE && dma->Init.PeriphInc == DMA_PINC_DISABLE &&
        dma->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE &&
        dma->Init.PeriphDataAlignment == DMA_PDATAALIGN_BYTE;
}
static XBeeResult start_rx(void)
{
    dma_position = 0;
    receiving = true;
    if (HAL_UARTEx_ReceiveToIdle_DMA(port, dma_rx, XBEE_RX_DMA_SIZE) != HAL_OK) {
        receiving = false;
        fault = true;
        return XBEE_IO_ERROR;
    }
    return fault ? XBEE_IO_ERROR : XBEE_OK;
}

XBeeResult XBee_Init(UART_HandleTypeDef *uart)
{
    if (uart == NULL || !dma_valid(uart->hdmarx, DMA_CIRCULAR, DMA_PERIPH_TO_MEMORY) ||
        !dma_valid(uart->hdmatx, DMA_NORMAL, DMA_MEMORY_TO_PERIPH) ||
        uart->Init.WordLength != UART_WORDLENGTH_8B || uart->Init.StopBits != UART_STOPBITS_1 ||
        uart->Init.Parity != UART_PARITY_NONE || uart->Init.Mode != UART_MODE_TX_RX ||
        uart->Init.HwFlowCtl != UART_HWCONTROL_NONE || uart->Init.BaudRate == 0)
        return XBEE_INVALID_ARGUMENT;
    if (port != NULL) return XBEE_BUSY; /* Use Recover after an error. */
    port = uart;
    diagnostics = (XBeeDiagnostics){0};
    head = tail = 0;
    tx_busy = fault = rx_loss = abort_done = false;
    HAL_GPIO_WritePin(Xbee_Reset_GPIO_Port, Xbee_Reset_Pin, GPIO_PIN_SET);
    phase = BOOT_WAIT;
    phase_started = HAL_GetTick();
    return start_rx();
}

bool XBee_IsAwake(void)
{
    return port != NULL && HAL_GPIO_ReadPin(Wake_Sleep_XBee_GPIO_Port,
                                          Wake_Sleep_XBee_Pin) == GPIO_PIN_SET;
}
bool XBee_IsTxBusy(void) { return tx_busy; }
XBeeStatus XBee_GetStatus(void)
{
    if (port == NULL) return XBEE_UNINITIALIZED;
    if (phase == STOPPING) return XBEE_RECOVERING;
    if (fault) return XBEE_FAULT;
    if (phase != RUNNING) return XBEE_BOOTING;
    if (tx_busy) return XBEE_TRANSMITTING;
    return XBee_IsAwake() ? XBEE_READY : XBEE_ASLEEP;
}

XBeeResult XBee_Send(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0 || length > XBEE_TX_CAPACITY) return XBEE_INVALID_ARGUMENT;
    if (port == NULL) return XBEE_NOT_READY;
    if (fault) return XBEE_IO_ERROR;
    if (tx_busy) return XBEE_BUSY;
    if (phase != RUNNING || !XBee_IsAwake()) return XBEE_NOT_READY;
    memcpy(dma_tx, data, length);
    /* Error/TC IRQs must not race the busy flag or HAL's initial setup. */
    uint32_t p = lock();
    if (fault) { unlock(p); return XBEE_IO_ERROR; }
    tx_busy = true;
    HAL_StatusTypeDef result = HAL_UART_Transmit_DMA(port, dma_tx, length);
    if (result == HAL_OK) ++diagnostics.txStarted;
    else {
        tx_busy = false;
        if (result != HAL_BUSY) fault = true;
    }
    unlock(p);
    return result == HAL_OK ? XBEE_OK : result == HAL_BUSY ? XBEE_BUSY : XBEE_IO_ERROR;
}
XBeeResult XBee_SendString(const char *text)
{
    if (text == NULL) return XBEE_INVALID_ARGUMENT;
    uint16_t length = 0;
    while (length <= XBEE_TX_CAPACITY && text[length] != '\0') ++length;
    return XBee_Send((const uint8_t *)text, length);
}

void XBee_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    (void)size;
    if (port == NULL || uart != port || !receiving) return;
    /* Current NDTR, not historic callback Size: pending IDLE/HT/TC events
     * can arrive out of order. Equal UART/RX-DMA priority prevents nesting.
     * Service IRQs within a half-buffer interval to avoid undetectable laps. */
    uint16_t position = (uint16_t)(XBEE_RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(port->hdmarx));
    if (position >= XBEE_RX_DMA_SIZE) position = 0;
    uint16_t count = (uint16_t)((position + XBEE_RX_DMA_SIZE - dma_position) % XBEE_RX_DMA_SIZE);
    uint32_t write = head;
    diagnostics.receivedBytes += count;
    if (rx_loss || (uint32_t)(write - tail) + count > XBEE_RX_QUEUE_SIZE) {
        if (count != 0) { diagnostics.droppedBytes += count; mark_loss(); }
    } else {
        for (uint16_t i = 0; i < count; ++i)
            queue[(write + i) & QUEUE_MASK] = dma_rx[(dma_position + i) % XBEE_RX_DMA_SIZE];
        __DMB();
        head = write + count;
    }
    dma_position = position;
}

XBeeResult XBee_Read(uint8_t *data, uint16_t capacity, uint16_t *length)
{
    if (length == NULL) return XBEE_INVALID_ARGUMENT;
    *length = 0;
    if (data == NULL || capacity == 0) return XBEE_INVALID_ARGUMENT;
    if (port == NULL || phase == STOPPING || phase == RESET_LOW) return XBEE_NOT_READY;
    if (fault) return XBEE_IO_ERROR;
    uint32_t read = tail;
    uint32_t count = head - read;
    if (count > capacity) count = capacity;
    __DMB();
    /* Producer cannot reuse these slots until tail advances. */
    for (uint32_t i = 0; i < count; ++i) data[i] = queue[(read + i) & QUEUE_MASK];
    uint32_t p = lock();
    if (fault) { unlock(p); return XBEE_IO_ERROR; }
    if (rx_loss) {
        diagnostics.droppedBytes += head - tail;
        tail = head;
        rx_loss = false;
        unlock(p);
        return XBEE_RX_LOST;
    }
    __DMB();
    tail = read + count;
    *length = (uint16_t)count;
    unlock(p);
    return XBEE_OK;
}

void XBee_TxCompleteCallback(UART_HandleTypeDef *uart)
{
    if (port != NULL && uart == port && tx_busy && !fault && phase != STOPPING) {
        ++diagnostics.txCompleted;
        tx_busy = false;
    }
}
void XBee_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (port == NULL || uart != port) return;
    diagnostics.lastUartError = uart->ErrorCode;
    ++diagnostics.uartErrors;
    receiving = false;
    fault = true;
    mark_loss();
    /* No restart or parsing in an ISR. Application calls Recover. */
}
void XBee_AbortCompleteCallback(UART_HandleTypeDef *uart)
{
    if (port != NULL && uart == port) abort_done = true;
}
static XBeeResult stop_async(bool reset)
{
    if (port == NULL) return XBEE_NOT_READY;
    if (phase == STOPPING || phase == RESET_LOW) return XBEE_BUSY;
    if (reset && tx_busy) return XBEE_BUSY;
    phase = STOPPING;
    reset_requested = reset;
    abort_done = false;
    receiving = false;
    if (HAL_UART_Abort_IT(port) != HAL_OK) {
        phase = RUNNING;
        fault = true;
        return XBEE_IO_ERROR;
    }
    return XBEE_OK;
}
XBeeResult XBee_Reset(void) { return stop_async(true); }
XBeeResult XBee_Recover(void) { return stop_async(false); }

void XBee_Process(void)
{
    if (port == NULL) return;
    uint32_t now = HAL_GetTick();
    if (phase == STOPPING && abort_done) {
        uint32_t p = lock();
        if (tx_busy) ++diagnostics.txAborted;
        tx_busy = false;
        diagnostics.droppedBytes += head - tail;
        head = tail = 0;
        mark_loss();
        fault = false;
        unlock(p);
        if (reset_requested) {
            HAL_GPIO_WritePin(Xbee_Reset_GPIO_Port, Xbee_Reset_Pin, GPIO_PIN_RESET);
            phase = RESET_LOW;
            phase_started = now;
        } else {
            phase = BOOT_WAIT;
            phase_started = now;
            (void)start_rx();
        }
    } else if (phase == RESET_LOW && (uint32_t)(now - phase_started) >= XBEE_RESET_PULSE_MS) {
        /* GPIO SET releases the open-drain line; never push-pull drive it. */
        HAL_GPIO_WritePin(Xbee_Reset_GPIO_Port, Xbee_Reset_Pin, GPIO_PIN_SET);
        phase = BOOT_WAIT;
        phase_started = now;
        (void)start_rx();
    } else if (phase == BOOT_WAIT && (uint32_t)(now - phase_started) >= XBEE_BOOT_GUARD_MS) {
        phase = RUNNING;
    }
}
void XBee_GetDiagnostics(XBeeDiagnostics *out)
{
    if (out == NULL) return;
    uint32_t p = lock();
    *out = diagnostics;
    out->rxAvailable = (uint16_t)(head - tail);
    out->receiving = receiving;
    out->txBusy = tx_busy;
    unlock(p);
}
