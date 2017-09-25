#ifndef __M4MID_COMMON
#define __M4MID_COMMON

#include "m4_driverlib.h"

#define DEBUG_LEVEL     2

#define M4M_UART_BASE   0
#define M4M_UART_BAUD   115200

#define M4M_SPI_BASE_LCD    1

#define MAX_LOG_BUFFER  128 // bytes


#define ML_HELLO_WORLD  "Hello M4MID"

typedef enum __M4M_DEBUG_LEVEL_ {
    M4M_NODEBUG,
    M4M_ERROR,
    M4M_INFO,
} M4M_DEBUG_LEVEL;

typedef enum __M4M_BOOT_REASON_ {
    M4M_NORMAL,
    M4M_TFREADER,
    M4M_UNEXPECTED,
} M4M_BOOT_REASON;

void app_video(void);
void app_midi(void);
void app_tfreader(void);

#endif // __M4MID_COMMON
