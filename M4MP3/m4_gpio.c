#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#include "m4_config.h"
#include "m4_common.h"

// callback array
void (*gpio_int_handler)(Button );

void gpio_init(void)
{
    // init onboard leds, D2~D6
    MAP_SysCtlPeripheralEnable(LED_PERIPH_D2);
    MAP_SysCtlPeripheralEnable(LED_PERIPH_D3);
    MAP_SysCtlPeripheralEnable(LED_PERIPH_D4);
#ifndef USE_DY_PB_FINAL
    MAP_SysCtlPeripheralEnable(LED_PERIPH_D5);
#endif // USE_DY_PB_FINAL

#ifndef USE_DY_PB_2
    MAP_SysCtlPeripheralEnable(LED_PERIPH_D6);
#endif // USE_DY_PB_2
#ifdef USE_DY_PB_3
    MAP_SysCtlPeripheralEnable(THERMAL_RESISTOR);
#endif // USE_DY_PB_3
    
    // Unlock PF0 so we can change it to a GPIO input
    // Once we have enabled (unlocked) the commit register then re-lock it
    // to prevent further changes.  PF0 is muxed with NMI thus a special case.
    //
    HWREG(LED_GPIO_D2 + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(LED_GPIO_D2 + GPIO_O_CR) |= 0x01;
    HWREG(LED_GPIO_D2 + GPIO_O_LOCK) = 0;
    
    MAP_GPIOPinTypeGPIOOutput(LED_GPIO_D2 , LED_PIN_D2);    // PF0
    MAP_GPIOPinTypeGPIOOutput(LED_GPIO_D3 , LED_PIN_D3);    // PA4
    MAP_GPIOPinTypeGPIOOutput(LED_GPIO_D4 , LED_PIN_D4);    // PD6
#ifndef USE_DY_PB_FINAL
    MAP_GPIOPinTypeGPIOOutput(LED_GPIO_D5 , LED_PIN_D5);    // PB3
#endif // USE_DY_PB_FINAL
#ifndef USE_DY_PB_2
    MAP_GPIOPinTypeGPIOOutput(LED_GPIO_D6 , LED_PIN_D6);    // PF3
#endif // USE_DY_PB_2
#ifdef USE_DY_PB_3
    MAP_GPIOPinTypeGPIOOutput(THERMAL_GPIO , THERMAL_PIN);  // PF2
    gpio_thermal_on(0);
#endif // USE_DY_PB_3
    
    HWREG(BUTTON_GPIO_K1 + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(BUTTON_GPIO_K1 + GPIO_O_CR) |= 0x80;
    HWREG(BUTTON_GPIO_K1 + GPIO_O_LOCK) = 0;
    
#ifdef M4_BUTTON
    // init push buttons, K1~K4
    // the push button shares the same peripherals with led
    MAP_GPIOPinTypeGPIOInput(BUTTON_GPIO_K1 , BUTTON_PIN_K1);   // PD7
    MAP_GPIOPinTypeGPIOInput(BUTTON_GPIO_K2 , BUTTON_PIN_K2);   // PF4
    MAP_GPIOPinTypeGPIOInput(BUTTON_GPIO_K3 , BUTTON_PIN_K3);   // PA3
    MAP_GPIOPinTypeGPIOInput(BUTTON_GPIO_K4 , BUTTON_PIN_K4);   // PA2
    
    // enable interrupts
    MAP_IntEnable(BUTTON_INT_K1);
    MAP_IntEnable(BUTTON_INT_K2);
    MAP_IntEnable(BUTTON_INT_K3);
    MAP_IntEnable(BUTTON_INT_K4);
    
#if 0
#define DETECT_EDGE     GPIO_RISING_EDGE
#else
#define DETECT_EDGE     GPIO_FALLING_EDGE
#endif
    
    MAP_GPIOIntTypeSet(BUTTON_GPIO_K1, BUTTON_PIN_K1, DETECT_EDGE);
    MAP_GPIOIntTypeSet(BUTTON_GPIO_K2, BUTTON_PIN_K2, DETECT_EDGE);
    MAP_GPIOIntTypeSet(BUTTON_GPIO_K3, BUTTON_PIN_K3, DETECT_EDGE);
    MAP_GPIOIntTypeSet(BUTTON_GPIO_K4, BUTTON_PIN_K4, DETECT_EDGE);
    
    GPIOIntEnable(BUTTON_GPIO_K1, BUTTON_PIN_K1);
    GPIOIntEnable(BUTTON_GPIO_K2, BUTTON_PIN_K2);
    GPIOIntEnable(BUTTON_GPIO_K3, BUTTON_PIN_K3);
    GPIOIntEnable(BUTTON_GPIO_K4, BUTTON_PIN_K4);
#endif // M4_BUTTON
}

void gpio_led_runner(int8_t direction)
{
    int8_t i_led_num = 1, j_led_num;
    if (direction < 0)
    {
        i_led_num = MAX_LED_NUM - 1;
    }
    
    for (; (i_led_num < MAX_LED_NUM) && (i_led_num >= 0) ; i_led_num += direction)
    {
        for (j_led_num = 0; j_led_num < MAX_LED_NUM; j_led_num ++)
        {
            MAP_GPIOPinWrite(LED_GPIO_D2, LED_PIN_D2, LED_PIN_D2*(0 != i_led_num));   // D2 overlapped
            MAP_GPIOPinWrite(LED_GPIO_D3, LED_PIN_D3, LED_PIN_D3*(1 != i_led_num));
            MAP_GPIOPinWrite(LED_GPIO_D4, LED_PIN_D4, LED_PIN_D4*(2 != i_led_num));
#ifndef USE_DY_PB_FINAL
            MAP_GPIOPinWrite(LED_GPIO_D5, LED_PIN_D5, LED_PIN_D5*(3 != i_led_num));
#endif // USE_DY_PB_FINAL
#ifdef USE_DY_PB_1
            MAP_GPIOPinWrite(LED_GPIO_D6, LED_PIN_D6, LED_PIN_D6*(4 != i_led_num));
#endif // USE_DY_PB_1
        }
        sleep(50);
    }
}

void gpio_led_test()
{
    gpio_led_runner(1);
    gpio_led_runner(-1);
}

void gpio_led_all(uint8_t on)
{
    MAP_GPIOPinWrite(LED_GPIO_D2, LED_PIN_D2, ~(LED_PIN_D2*on));
    MAP_GPIOPinWrite(LED_GPIO_D3, LED_PIN_D3, ~(LED_PIN_D3*on));
    MAP_GPIOPinWrite(LED_GPIO_D4, LED_PIN_D4, ~(LED_PIN_D4*on));
}

void gpio_thermal_on(uint8_t on)
{
#ifdef USE_DY_PB_4
    on = 1 - on;
#endif
    MAP_GPIOPinWrite(THERMAL_GPIO, THERMAL_PIN, THERMAL_PIN*on);
}

uint8_t gpio_bootpin_test(void)
{
    uint32_t gpio_val = 0;
    uint8_t boot_pin = 0;
    
    // K1~K4 use different gpio pins, use bit or to merge them
    gpio_val |= GPIOPinRead(GPIO_PORTD_BASE, BUTTON_PIN_K1);
    gpio_val |= GPIOPinRead(GPIO_PORTF_BASE, BUTTON_PIN_K2);
    
    gpio_val |= GPIOPinRead(GPIO_PORTA_BASE, BUTTON_PIN_K3 | BUTTON_PIN_K4);
    
    switch ((~gpio_val) & (BUTTON_PIN_K1 | BUTTON_PIN_K2 | BUTTON_PIN_K3 | BUTTON_PIN_K4))
    {
    case BUTTON_PIN_K1:
        boot_pin = 1;
        break;
    case BUTTON_PIN_K2:
        boot_pin = 2;
        break;
    case BUTTON_PIN_K3:
        boot_pin = 3;
        break;
    case BUTTON_PIN_K4:
        boot_pin = 4;
        break;
    case (BUTTON_PIN_K3 | BUTTON_PIN_K4):
    default:
        boot_pin = 0;
        break;
    }
    
    m4_dbgprt("boot pin=%d\r\n", boot_pin);
    
    return boot_pin;
}

void gpioa_int_isr(void)
{
    uint32_t gpio_ints;
    uint32_t gpio_val;
    
    gpio_ints = GPIOIntStatus(GPIO_PORTA_BASE, true);    
    GPIOIntClear(GPIO_PORTA_BASE, gpio_ints);
    
    // k3 get unexpected pressed some time:
k3_K4_judge:
    
    gpio_val = GPIOPinRead(GPIO_PORTA_BASE, BUTTON_PIN_K3 | BUTTON_PIN_K4);
    switch ((~gpio_val) & (BUTTON_PIN_K3 | BUTTON_PIN_K4))
    {
    case BUTTON_PIN_K3:
        m4_dbgprt("K3 pressed\r\n");
        if (gpio_int_handler)
            (*gpio_int_handler)(Key_3);
        break;
    case BUTTON_PIN_K4:
        m4_dbgprt("K4 pressed\r\n");
        if (gpio_int_handler)
            (*gpio_int_handler)(Key_4);
        break;
    default:
        m4_dbgprt("K3/K4\r\n");
        goto k3_K4_judge;
        if (gpio_int_handler)
            (*gpio_int_handler)(Key_3);
    }
}

void gpiod_int_isr(void)
{
    uint32_t gpio_ints;
    uint32_t gpio_val;

#ifdef M4_USBMSC
    Boot_Reason boot_reason;
#endif // M4_USBMSC
    
    gpio_ints = GPIOIntStatus(GPIO_PORTD_BASE, true);
    GPIOIntClear(GPIO_PORTD_BASE, gpio_ints);
    
    gpio_val = GPIOPinRead(GPIO_PORTD_BASE, BUTTON_PIN_K1 | USB_PIN_DP);
    switch ((~gpio_val) & (BUTTON_PIN_K1 | USB_PIN_DP))
    {
#ifdef M4_USBMSC
    case USB_PIN_DP:
        m4_dbgprt("USB Connected\r\n");
        boot_reason = BR_USB;
        boot_reason_update(&boot_reason);
        SysCtlReset();
        break;
#endif // M4_USBMSC
    case BUTTON_PIN_K1:
        m4_dbgprt("K1 pressed\r\n");
        if (gpio_int_handler)
            (*gpio_int_handler)(Key_1);
        break;
    default:    // gpio unable to figure out which pin
        break;
    }
}

void gpiof_int_isr(void)
{
    uint32_t gpio_ints;
    uint32_t gpio_val;
    
    gpio_ints = GPIOIntStatus(GPIO_PORTF_BASE, true);
    GPIOIntClear(GPIO_PORTF_BASE, gpio_ints);
    
    gpio_val = GPIOPinRead(GPIO_PORTF_BASE, BUTTON_PIN_K2 | CC1101_PIN_GDO0);
    switch ((~gpio_val) & (BUTTON_PIN_K2 | CC1101_PIN_GDO0))
    {
#ifdef M4_CC1101
    case CC1101_PIN_GDO0:
        cc1101_int_isr();
        break;
#endif // M4_CC1101
    case BUTTON_PIN_K2:
    default:
        m4_dbgprt("K2 pressed\r\n");
        if (gpio_int_handler)
            (*gpio_int_handler)(Key_2);
        break;
    }
}

void gpio_ssi2_deselect()
{
    // LCD:
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);
    
    // SDC:
    MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
    
    // CC1101:
    MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);
}

void gpio_int_register(void (*func)(Button button))
{
    gpio_int_handler = func;
}
