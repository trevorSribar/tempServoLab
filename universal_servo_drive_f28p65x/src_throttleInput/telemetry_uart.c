#include "telemetry_uart.h"

#include "driverlib.h"
#include "device.h"

// ---- Choose which SCI is routed to XDS110 VCOM ----
// LaunchPad doc: default VCOM uses SCIA on GPIO42 (TX) / GPIO43 (RX). :contentReference[oaicite:1]{index=1}
#define TL_SCI_BASE        SCIA_BASE

static inline uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i = 0;
    for(i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        uint16_t b = 0;
        for(b = 0; b < 8; b++)
        {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static inline void sci_writeByte(uint8_t b)
{
    SCI_writeCharBlockingFIFO(TL_SCI_BASE, b);
}

static inline void sci_writeBuf(const uint8_t *buf, uint16_t n)
{
    uint16_t i = 0;
    for(i = 0; i < n; i++) sci_writeByte(buf[i]);
}

void TelemetryUart_init(TelemetryUart_t *t)
{
    t->numCh = 0;
    t->seq = 0;
    t->div = TL_SEND_DIV;
    t->divCount = 0;

    // Device_init();
    // Device_initGPIO();

    // --- GPIO mux for SCIA on GPIO42/43 (LaunchPad default VCOM route) ---

    GPIO_setPinConfig(GPIO_42_SCIA_TX);
    GPIO_setPinConfig(GPIO_43_SCIA_RX);
    GPIO_setDirectionMode(42, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);


    SCI_performSoftwareReset(TL_SCI_BASE);


    SCI_setConfig(TL_SCI_BASE, DEVICE_LSPCLK_FREQ, TL_UART_BAUD,
                  (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE));
    SCI_enableFIFO(TL_SCI_BASE);
    SCI_resetTxFIFO(TL_SCI_BASE);
    SCI_resetRxFIFO(TL_SCI_BASE);
    SCI_clearInterruptStatus(TL_SCI_BASE, SCI_INT_TXFF | SCI_INT_RXFF);

    SCI_enableModule(TL_SCI_BASE);
}

bool TelemetryUart_addChannel(TelemetryUart_t *t, volatile float *var) // string toSend = "%x",variable;
{
    if(t->numCh >= TL_MAX_CH) return false;
    t->ch[t->numCh++] = var;
    return true;
}

// Packet format:
// [SOF0][SOF1][lenLo][lenHi][seqLo][seqHi][numChLo][numChHi][payload...][crcLo][crcHi]
//
// payload = numCh * float32 (little-endian)
void TelemetryUart_service(TelemetryUart_t *t)
{
    // decimate to reduce CPU/serial bandwidth
    if(++t->divCount < t->div) return;
    t->divCount = 0;

    const uint16_t numCh = t->numCh;
    if(numCh == 0) return;

    // Build header+payload in a stack buffer (size bounded)
    // Max bytes = 2 + 2 + 2 + 2 + (16*4) + 2 = 74 bytes
    uint8_t buf[2 + 2 + 2 + 2 + (TL_MAX_CH * 4) + 2];

    uint16_t idx = 0;
    buf[idx++] = TL_SOF0;
    buf[idx++] = TL_SOF1;

    const uint16_t payloadBytes = (uint16_t)(numCh * 4u);
    const uint16_t headerBytes  = 2u /*SOF*/ + 2u /*len*/ + 2u /*seq*/ + 2u /*numCh*/;
    const uint16_t totalNoCrc   = (uint16_t)(headerBytes + payloadBytes);
    const uint16_t totalLen     = (uint16_t)(totalNoCrc + 2u /*crc*/);

    buf[idx++] = (uint8_t)(totalLen & 0xFFu);
    buf[idx++] = (uint8_t)(totalLen >> 8);

    const uint16_t seq = t->seq++;
    buf[idx++] = (uint8_t)(seq & 0xFFu);
    buf[idx++] = (uint8_t)(seq >> 8);

    buf[idx++] = (uint8_t)(numCh & 0xFFu);
    buf[idx++] = (uint8_t)(numCh >> 8);

    // payload floats (little-endian)
    uint16_t c = 0;
    for(c = 0; c < numCh; c++)
    {
        union { float f; uint8_t b[4]; } u;
        u.f = *(t->ch[c]);   // snapshot
        buf[idx++] = u.b[0];
        buf[idx++] = u.b[1];
        buf[idx++] = u.b[2];
        buf[idx++] = u.b[3];
    }

    // CRC over everything except CRC itself
    const uint16_t crc = crc16_ccitt(buf, idx);
    buf[idx++] = (uint8_t)(crc & 0xFFu);
    buf[idx++] = (uint8_t)(crc >> 8);

    sci_writeBuf(buf, idx);
}
