#ifndef TELEMETRY_UART_H
#define TELEMETRY_UART_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TL_MAX_CH          (16u)      // number of float channels to stream
#define TL_UART_BAUD       (921600u)  
#define TL_SEND_DIV        (10u)      // send once every N calls 

// Packet framing
#define TL_SOF0            (0xAAu)
#define TL_SOF1            (0x55u)

typedef struct
{
    volatile float *ch[TL_MAX_CH];
    uint16_t        numCh;
    uint16_t        seq;
    uint16_t        div;
    uint16_t        divCount;
} TelemetryUart_t;

// Call once at init
void TelemetryUart_init(TelemetryUart_t *t);

// Register a float variable pointer to be streamed
bool TelemetryUart_addChannel(TelemetryUart_t *t, volatile float *var);

// Call from a periodic context
void TelemetryUart_service(TelemetryUart_t *t);

#ifdef __cplusplus
}
#endif

#endif
