#include <stdint.h>
#include <stdbool.h>

#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/uart.h"

#include "m4_driverlib.h"

uint32_t tiva_gpio[] = {
    GPIO_PORTA_BASE, GPIO_PORTB_BASE,
    GPIO_PORTC_BASE, GPIO_PORTD_BASE,
};

uint32_t tiva_i2c[] = {
    I2C0_BASE, I2C1_BASE,
    I2C2_BASE, I2C3_BASE,
};

uint32_t tiva_pwm[] = {
    PWM0_BASE, PWM1_BASE,
};

uint32_t tiva_spi[] = {
    SSI0_BASE, SSI1_BASE,
    SSI2_BASE, SSI3_BASE,
};

uint32_t tiva_uart[] = {
    UART0_BASE, UART1_BASE,
    UART2_BASE, UART3_BASE,
};

/* cpu */
void fpu_enable(bool enable)
{
    if (enable)
    {
        MAP_FPUEnable();
        MAP_FPULazyStackingEnable();
    }
    else
    {
        MAP_FPUDisable();
    }
}

void cpu_pll_enable(bool enable)
{
    if (enable)
    {
        MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    }
    else
    {
        MAP_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    }
}

/* gpio */
void gpio_init(uint8_t base)
{
    
}

inline uint32_t gpio_read(uint8_t base)
{
    uint32_t value;
    
    value = MAP_GPIOPinRead(tiva_gpio[base], UINT8_MAX);
    
    return value;
}

inline void gpio_write(uint8_t base, uint32_t value)
{
    // only modify the diff bits
    uint32_t mask, old_value;
    
    old_value = MAP_GPIOPinRead(tiva_gpio[base], UINT8_MAX);   // 1111111b
    mask = (old_value ^ value);
    MAP_GPIOPinWrite(tiva_gpio[base], mask, value);
}

void gpio_cb()
{
    
}

/* i2c */
void i2c_init(uint32_t clock)
{
    
}

void i2c_put()
{
    
}

uint32_t i2c_get()
{
    uint32_t value;
    
    return value;
}

/* pwm */
void pwm_init()
{
    
}



/* spi */
void spi_init(uint32_t clock)
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0))
    {
    }
}

inline void spi_put(uint8_t base, uint32_t value)
{
    MAP_SSIDataPut(tiva_spi[base], value);
}

inline uint32_t spi_get(uint8_t base)
{
    uint32_t value;
    
    MAP_SSIDataGet(tiva_spi[base], &value);
    
    return value;
}

void spi_write(uint8_t base, uint8_t *buffer, uint32_t len)
{
#if TIVA_DMA_SPI
    
#else // TIVA_DMA_SPI
    
#endif // TIVA_DMA_SPI
}

uint32_t spi_read(uint8_t base, uint32_t *buffer, uint32_t len)
{
    uint32_t read_len = 0;
#if TIVA_DMA_SPI
    
#else // TIVA_DMA_SPI
    while(len --)
    {
        while(!MAP_UARTCharsAvail(tiva_uart[base]))
        {
            // timeout
        }
        
        MAP_SSIDataGetNonBlocking(tiva_uart[base], buffer);
        buffer ++;
    }    
#endif // TIVA_DMA_SPI
    
    return read_len;
}

inline void spi_dma_set()
{
}

/* uart */
void uart_init(uint8_t base, uint32_t baud)
{
    MAP_UARTConfigSetExpClk(tiva_uart[base], MAP_SysCtlClockGet(), baud,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
        UART_CONFIG_PAR_NONE));
    
    // enable fifo
    MAP_UARTFIFOEnable(tiva_uart[base]);
}

inline void uart_put(uint8_t base, uint8_t value)
{
    MAP_UARTCharPut(tiva_uart[base], value);
}

inline uint8_t uart_get(uint8_t base)
{
    uint8_t value;
    
    value = MAP_UARTCharGet(tiva_uart[base]);
    
    return value;
}

void uart_write(uint8_t base, uint8_t *buffer, uint32_t len)
{
#if TIVA_DMA_UART
    // dma version
#else // TIVA_DMA_UART
    while (len --)
    {
        // wait for fifo empty
        while (!MAP_UARTSpaceAvail(tiva_uart[base]))
        {
            // timeout
        }
        
        MAP_UARTCharPutNonBlocking(tiva_uart[base], *buffer);
        buffer ++;
    }
#endif // TIVA_DMA_UART
}
    
uint32_t uart_read(uint8_t base, uint8_t *buffer, uint32_t len)
{
    int read_len = 0;
    
#if TIVA_DMA_UART
    // dma version
#else // TIVA_DMA_UART
    while(len --)
    {
        while(!MAP_UARTCharsAvail(tiva_uart[base]))
        {
            // timeout
        }
        
        *buffer = MAP_UARTCharGetNonBlocking(tiva_uart[base]);
        buffer ++;
    }
#endif // TIVA_DMA_UART
    
    return read_len;
}

void uart_cb()
{
    
}
