#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/hibernate.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"

#include "m4_pinmux.h"
#include "m4_common.h"
#include "m4_config.h"
#include "m4_ver.h"

#include "tlv320aic23b.h"

// global systick count
uint32_t sys_tick_count = 0;
uint8_t dy_pb_version = 4;

// print the debug info to uart
void m4_dbgprt(const char *format, ...)
{
#ifdef DEBUG
    char msg_buffer[MAX_MSG_LEN] = { 0 };

    va_list vl;
    va_start(vl, format);
    vsprintf(msg_buffer, format, vl);
    va_end(vl);

    uart_send(DEBUG_PORT, (const char *)msg_buffer, strlen(msg_buffer));
#endif
}

#ifdef DEBUG
// The error routine that is called if the driver library encounters an error.
void __error__(char *pcFilename, uint32_t ui32Line)
{
    m4_dbgprt("library error: %s:%d\r\n", pcFilename, ui32Line);
}
#endif

#if 0
void sleep(uint32_t ms)
{
    uint32_t sys_ctl_clock, tick_delay;
    sys_ctl_clock = SysCtlClockGet();
    tick_delay = (sys_ctl_clock/(1000*4))*ms;
    SysCtlDelay(tick_delay);
}
#else
void sleep(uint32_t ms)
{
    uint32_t delay;
    while (ms --)
        for (delay = 18000; delay; delay --)
            ;
}
#endif

void boot_reason_update(Boot_Reason *boot_reason)
{
    uint32_t rtc_val;
    
    HibernateDataGet(&rtc_val, 1);
    HibernateDataSet((uint32_t *)boot_reason, 1);
    
    (*boot_reason) = rtc_val;
}

void systick_int_isr(void)
{
    sys_tick_count += 10;
#ifdef M4_USBMSC
    usb_msc_timeout_cb();
#endif // M4_USBMSC
}

#ifdef UART_PROXY
void uart_proxy(char ascii)
{
    uart_send(UART1_BASE, (const char *)&ascii, 1);
}
#endif // UART_PROXY

//volatile float a, b;
int main(void)
{
    char uart_char = 'a';
    
    uint8_t boot_pin;
    Boot_Reason boot_reason;
    
    uint16_t boot_count_down; 
    uint8_t boot_count_down_sec, last_boot_count_down_sec;
    
    // enabling FPU
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();

    // set the clocking to run directly from the crystal.
#ifdef M4_80MHZ
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
#else
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
#endif
    
    // read boot reason from rtc memory
    boot_reason = BR_NORMAL;    // values to update
    boot_reason_update(&boot_reason);
    
    // enable processor interrupts
    ROM_IntMasterEnable();
    
    // enable the GPIO pin mux
    PortFunctionInit();
    
#if (defined(M4_SYSTICK) && (!defined(M4_RTX)) && (!defined(M4_FREERTOS)) && (!defined(M4_TIRTOS)))
    // enable systick (10ms)
    ROM_SysTickPeriodSet(SysCtlClockGet() / 100);
    ROM_SysTickIntEnable();
    ROM_SysTickEnable();
#endif // M4_SYSTICK
    
#ifdef DEBUG
    // init UART0 for debug purpose
    uart_init(UART0_BASE, UART0_BAUD_RATE);
#endif // DEBUG

#ifdef M4_UART
    uart_init(UART1_BASE, UART5_BAUD_RATE);
#ifdef UART_PROXY
    uart_int_register(&uart_proxy);
    while (true)
    {
        uart_char = UARTCharGet(UART1_BASE);
        uart_send(UART0_BASE, (const char *)&uart_char, 1);
    }
#endif // UART_PROXY
#endif // M4_UART

    m4_dbgprt("+++");
    
    sleep(100);
    
    //m4_dbgprt("clock=%u\r\n", SysCtlClockGet());

#ifdef M4_LCD
    lcd_init();
    
    if (boot_reason == BR_NORMAL)
    {
        lcd_flash();
        lcd_clear();
    }
#endif // M4_LCD

#ifdef M4_AC
    lcd_printf(0, 0, "AC init..       ");
    ac_init();
#endif // M4_AC

#ifdef M4_DAC
    lcd_printf(0, 0, "DAC init..      ");
    dac_init();
#endif // M4_DAC

#ifdef M4_GPIO
    lcd_printf(0, 0, "GPIO init..     ");
    gpio_init();
#endif // M4_GPIO

#ifdef M4_TEMP
    lcd_printf(0, 0, "TENP init..     ");
    temp_init();
    
    dy_pb_version = temp_addr_test();
#endif // M4_TEMP

#ifdef M4_SPWM
    lcd_printf(0, 0, "SPWM init..     ");
    spwm_init();
#endif // M4_SPWM

#ifdef M4_PWM
    lcd_printf(0, 0, "PWM init..      ");
    pwm_init();
#endif // M4_PWM

#ifdef M4_RTC
    lcd_printf(0, 0, "RTC init..      ");
    rtc_init();
#endif // M4_RTC

#ifdef M4_TMR_CCP
    lcd_printf(0, 0, "TMR CCP init..  ");
    timer_ccp_init();
#endif // M4_TMR_CCP

#ifdef M4_WDT
    lcd_printf(0, 0, "WDT init..      ");
    wdt_init();
#endif // M4_WDT

#ifdef M4_CC1101
    lcd_printf(0, 0, "CC1101 init..   ");
    //cc1101_init();
#endif // M4_CC1101

#ifdef M4_CODEC
    lcd_printf(0, 0, "CODEC init..    ");
    // the dac shares ssi with codec, at reset we must disable codec!
    TLV320AIC23BDisable();
#endif // M4_CODEC

#ifdef M4_SDC
    lcd_printf(0, 0, "SDC init..    ");
    sdc_init();
#endif // M4_SDC

#if defined (M4_RTX)
    rtx_entry(app_main);
#elif defined (M4_FREERTOS)
    freertos_entry(app_main);
#elif defined (M4_TIRTOS)
    tirtos_entry(app_main);
#else // default boot:

    // printf debug info to lcd
    lcd_clear();
    lcd_printf(0, 0, "Build %d", BUILD_NUM);
    lcd_printf(0, 1, "Tiva-PB %d", dy_pb_version);
    
#if defined(M4_RTX)
    lcd_printf(0, 2, "With Keil RTX");
#elif defined(M4_FREERTOS)
    lcd_printf(0, 2, "With FreeRTOS");
#endif

    lcd_printf(0, 3, "Press BOOT KEY");
    
    m4_dbgprt("boot=%d\r\n", boot_reason);
    
    if (boot_reason == BR_NORMAL)
    {
        for (boot_count_down = BOOT_COUNTDOWN; boot_count_down > 0; boot_count_down --)
        {
            boot_count_down_sec = boot_count_down/1000;
            if (boot_count_down_sec != last_boot_count_down_sec)
            {
                lcd_printf(15, 3, "%d", boot_count_down_sec);
                last_boot_count_down_sec = boot_count_down_sec;
            }
            
            // scan the key manually
            boot_pin = gpio_bootpin_test();
            if (boot_pin > 0)
            {
                break;
            }
            
            sleep(1);
        }

        // handle boot routine:
        switch (boot_pin)
        {
            case 0:
                // no key pressed
                break;
            case 1: // K1
                boot_reason = BR_PLAYER;
                break;
            case 2: // K2
                boot_reason = BR_PHONE;
                break;
            case 3: // K3
                boot_reason = BR_SDLOAD;
                break;
            case 4: // K4
            default:
                boot_reason = BR_NORMAL;
                break;
        }
    }
    
    lcd_clear();
    
    //boot_reason = BR_PLAYER;
    //boot_reason = BR_PHONE;
    
    switch (boot_reason)
    {
        case BR_PLAYER:
        {
#ifdef M4_USBMSC
            usb_gpio_detect();
#endif // M4_USBMSC
            
            app_main_player();
            break;
        }
#ifdef M4_USBMSC
        case BR_USB:
        {
            usb_msc_init();
            
            // once usb is plugged in, this function will never return
            usb_msc_loop();
            
            break;
        }
#endif // M4_USBMSC
        case BR_PHONE:
        {
            app_main_phone();
            break;
        }
        case BR_SDLOAD:
        {
            app_main_sdload();
            break;
        }
        case BR_BOOTLOADER:
        {
            boot_loader_entry();
            break;
        }
        case BR_NORMAL:
        default:
        {
            app_main_demo();
            break;
        }
    }
#endif

    return 0;
}
