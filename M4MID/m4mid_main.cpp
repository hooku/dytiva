/*
    This project incorporate the following functions:
    * a video app to play bitmap files form sd card
    * a midi app layer to play midi files
    * a usb tf card reader

    Compare to the M4MP3 project, what is improved in the M4MID project
    * DMA SPI is utilized to improve the peripheral speed: TF Card, GT20, ILI9341
    * 
    
    Memory Management
    
*/

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "m4mid_common.h"

void setup_cpu()
{
    fpu_enable(true);
    cpu_pll_enable(false);
}

void setup_pinmux()
{
    PortFunctionInit();
}

void setup_uart()
{
    uart_init(M4M_UART_BASE, M4M_UART_BAUD);
}

void setup_key()
{
    // enable key interrupt
    
}

void setup_led()
{
    led_rgb(0, 0, 0);
}

void setup_sd()
{
    
}

void setup_display()
{
    
}

void m4m_log(M4M_DEBUG_LEVEL level, const char *format, ...)
{
    char log_buffer[MAX_LOG_BUFFER] = { 0 };
    uint32_t log_len;
    
    if (level <= DEBUG_LEVEL)
    {
        va_list vl;
        va_start(vl, format);
        log_len = vsnprintf(log_buffer, MAX_LOG_BUFFER, format, vl);

        va_end(vl);
        
        if (log_len < (MAX_LOG_BUFFER - 2))
            strcat(log_buffer, "\r\n");

        uart_write(M4M_UART_BASE, (uint8_t *)log_buffer, log_len);
    }
}

void m4m_info(const char *format, ...)
{
    va_list vl; // takes twice vl could add extra footprint
    va_start(vl, format);

    m4m_log(M4M_INFO, format, vl);
    
    va_end(vl);
}

void m4m_reset()
{
    
}

int main(void)
{
    __disable_irq();
    
    setup_cpu();
    setup_pinmux();
    
    setup_uart();
    m4m_info(ML_HELLO_WORLD);
    
    setup_key();
    
    /* rgb led */
    setup_led();
    
    setup_display();
    
    // everything is ready, now enable the interrupt
    __enable_irq();
    
    app_video();
    app_midi();
    app_tfreader();
    
    while (1)
    {
        __wfi();
    }
    
    return 0;
}
