#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "driverlib/rom.h"

#include "driverlib/pwm.h"

#include "m4_common.h"

typedef struct __Music_Note
{
    uint8_t pitch;
    uint16_t duration;   // ms
} Music_Note;

uint16_t pitch_freq_table[] = { 0, // C6~B8
    1047, 1175, 1319, 1397, 1568, 1760, 1976,
    2093, 2349, 2637, 2794, 3136, 3520, 3951,
    4186, 4699, 5274, 5588, 6272, 7040, 7902,
};

Music_Note mum_good[] = {
/* 6535     */  { 0x16, 0x300 }, { 0x15, 0x100 }, { 0x13, 0x200 }, { 0x15, 0x200 },
/* 1656     */  { 0x21, 0x200 }, { 0x16, 0x100 }, { 0x15, 0x100 }, { 0x16, 0x400 },
/* 35653    */  { 0x13, 0x200 }, { 0x15, 0x100 }, { 0x16, 0x100 }, { 0x15, 0x200 }, { 0x13, 0x200 },
/* 165320   */  { 0x11, 0x100 }, { 0x06, 0x100 }, { 0x15, 0x100 }, { 0x13, 0x100 }, { 0x12, 0x400 }, { 0x00, 0x050 },
/* 23556    */  { 0x12, 0x300 }, { 0x13, 0x100 }, { 0x15, 0x200 }, { 0x15, 0x100 }, { 0x16, 0x100 },
/* 321      */  { 0x13, 0x200 }, { 0x12, 0x200 }, { 0x11, 0x400 }, 
/* 532161   */  { 0x15, 0x300 }, { 0x13, 0x100 }, { 0x12, 0x100 }, { 0x11, 0x100 }, { 0x06, 0x100 }, { 0x11, 0x100 }, 
/* 50       */  { 0x05, 0x600 }, { 0x00, 0x200 },
};

//uint16_t sinewave_sample[] = {
//    0x0800, 0x0A00, 0x0C00, 0x0E00, 0x0FFE, 0x0E00, 0x0C00, 0x0A00,
//    0x0800, 0x0500, 0x0300, 0x0100, 0x0001, 0x0100, 0x0300, 0x0500,
//};  // complement code
uint16_t sinewave_sample[] = { 0x0824, 0x9B3, 0xB31, 0xC8F, 0xDC1, 0xEBA, 0xF71, 0xFDF,
                               0x1000, 0xFD1, 0xF56, 0xE93, 0xD8F, 0xC54, 0xAEF, 0x96C,
                               0x7DC, 0x64D, 0x4CF, 0x0371, 0x23F, 0x0146, 0x8F, 0x0021,
                               0x0000, 0x2F, 0xAA, 0x16D, 0x0271, 0x3AC, 0x0511, 0x0694
                             };

uint8_t note_ptr;

void spwm_init(void)
{
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, SysCtlClockGet()/4000);
    
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 0);
    
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
}

void spwm_play(void)
{
    int i_sin, delay, ip;
    
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, SysCtlClockGet()/100000); // 120 kHz
    
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
    
    for (i_sin = 0; i_sin < 32; i_sin ++)
    {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3)*(sinewave_sample[i_sin])/4096);
        
        delay = 200;    // 2.6 K
        delay = 400;    // 1.3 K?
        while (delay --)
            ;
    }
    
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);
}

void spwm_scanfreq(int8_t direction)
{
#define PWM_FREQ_LOW    1500
#define PWM_FREQ_HI     5000
#define PWM_FREQ_DELTA  25
    
    static uint32_t pwm_freq = PWM_FREQ_LOW;
    int8_t pwm_freq_delta = PWM_FREQ_DELTA*direction;
    
#if 0 // spwm test
    spwm_play();
    
    //PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 10000);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, SysCtlClockGet()/100000);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3)/100*99);
    //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, SysCtlClockGet() - 1);
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
    
    while (1)
        ;
#endif
    
    for (pwm_freq += pwm_freq_delta; ((pwm_freq > PWM_FREQ_LOW) && (pwm_freq < PWM_FREQ_HI)); pwm_freq += pwm_freq_delta)
    {
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, SysCtlClockGet()/pwm_freq);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3)/2);

        sleep(1);
    }
    
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);
}

void pwm_init(void)
{
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, SysCtlClockGet()/100);
    
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 0);
    
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);
    
    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);
}

void pwm_set(uint8_t percent)
{
    uint32_t width;
    
    width = PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3)/100*percent;
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, width);
}

void pwm_set_ex(uint16_t adc_value)
{
    uint32_t width;

    width = PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3)*adc_value/10000;
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, width);
}

char pwm_play_reset()
{
    note_ptr = 0;
    
    return '0' + (mum_good[note_ptr].pitch & 0x0F);
}

char pwm_play_next()
{
    uint32_t pwm_freq;
    
    uint8_t pitch_freq_index;
    
    pitch_freq_index = 7*((mum_good[note_ptr].pitch & 0xF0) >> 8) + (mum_good[note_ptr].pitch & 0x0F);
    
    pwm_freq = pitch_freq_table[pitch_freq_index];
    
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, SysCtlClockGet()/pwm_freq);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3)/2);
    
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);

    sleep(mum_good[note_ptr].duration/5*4);
    
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);

    note_ptr ++;
    if (note_ptr == (sizeof(mum_good)/sizeof(Music_Note)))
    {
        note_ptr = 0;
    }
    
    return '0' + (mum_good[note_ptr].pitch & 0x0F);
}
