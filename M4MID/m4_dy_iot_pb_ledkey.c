#include "m4_driverlib.h"

#define PWM_LED_R
#define PWM_LED_G
#define GPIO_LED_B

void key_cb()
{
    
}

void led_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    /* r,g are pwm pins, and b is gpio pin */
    
    if (b > 0x7F)
    {
        
    }
    else
    {
        
    }
}
