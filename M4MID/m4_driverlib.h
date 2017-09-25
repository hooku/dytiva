#ifndef __M4_DRIVERLIB
#define __M4_DRIVERLIB

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"

#define TIVA_DMA_I2C    0
#define TIVA_DMA_SPI    0
#define TIVA_DMA_UART   0

#define DRIVER_TIMEOUT  50 // ms?

extern void PortFunctionInit(void);

/* driverlib */
// cpu:
extern void fpu_enable(bool );
extern void cpu_pll_enable(bool );
// gpio:
extern inline uint32_t gpio_read(uint8_t );
extern inline void gpio_write(uint8_t , uint32_t );

// i2c:

// spi:
extern void spi_init(uint32_t );
extern inline void spi_put(uint8_t , uint32_t );
extern inline uint32_t spi_get(uint8_t );
extern void spi_write(uint8_t , uint8_t *, uint32_t );
extern uint32_t spi_read(uint8_t , uint32_t *, uint32_t );

// uart:
extern void uart_init(uint8_t , uint32_t );
extern inline void uart_put(uint8_t , uint8_t );
extern inline uint8_t uart_get(uint8_t );
extern void uart_write(uint8_t , uint8_t *, uint32_t );
extern uint32_t uart_read(uint8_t , uint8_t *, uint32_t );

#endif // __M4_DRIVERLIB
