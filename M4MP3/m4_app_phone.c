#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "driverlib/gpio.h"
#include "sound.h"

#include "m4_common.h"
#include "m4_app_res.h"

//#define RECEIVER
#if 0
void wireless_test(void)
{
#define TEST_BUFFER_LEN         60  // 204.15Hz @ sleep=1ms | speed=12240Byte/s
    
    char send_buffer[TEST_BUFFER_LEN] = { 
                0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
                0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23, 0x24,
                0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31, 0x32,
                0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40,
                0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
                0x49, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56,
                0x57, 0x58, 0x59, 0x60,
            };
    
    char cur_ascii = 0x11;

#ifdef RECEIVER
    lcd_printf(0, 3, "Receiver");
    while (1)
    {
        //cc1101_test();
        sleep(200);
    }
#else
    lcd_printf(0, 3, "Sender");
    sleep(100);
    while (1)
    {
#if 1
        memset(send_buffer, cur_ascii, 1);
        cur_ascii ++;
        if (cur_ascii == 99)
            cur_ascii = 11;
        
        memcpy(cc_tx_buffer, send_buffer, TEST_BUFFER_LEN);
#endif
        
        cc1101_send(send_buffer, TEST_BUFFER_LEN);
        
        sleep(1);
    }
#endif
}
#endif

void update_phone_status()
{
    static uint8_t last_phone_mode = 0xFF;
    uint8_t phone_mode;
    
    phone_mode = phone_status_get();
    
    if (phone_mode != last_phone_mode)
    {
        switch (phone_mode)
        {
            case 1:
                lcd_printf(0, 1, "Speaking.. ");
                break;
            case 2:
                lcd_printf(0, 1, "Receiving..");
                break;
            default:
                lcd_printf(0, 1, "Idle..     ");
                break;
        }
        
        last_phone_mode = phone_mode;
    }
}

void update_signal_status()
{
    static uint8_t last_phone_signal = 0xFF;
    int8_t phone_signal;
    
    uint8_t pic_sig_index = 0;
    
    phone_signal = cc1101_get_signal()/10;
    
    if (phone_signal != last_phone_signal)
    {
        if (phone_signal > 5)
        {
            pic_sig_index = 4;
        }
        else if (phone_signal > 0)
        {
            pic_sig_index = 3;
        }
        else if (phone_signal > -2)
        {
            pic_sig_index = 2;
        }
        else if (phone_signal > -4)
        {
            pic_sig_index = 1;
        }
        else
        {
            pic_sig_index = 0;
        }
        
        lcd_printf(0, 2, "=%d", phone_signal);
        
        lcd_draw(14, 0, ICON_WIDTH, ICON_WIDTH, pic_sig[pic_sig_index]);
        
        last_phone_signal = phone_signal;
    }
}

void app_main_phone(void)
{
    // phone mode:
    lcd_clear();
    lcd_printf(0xFF, 0, "Phone");
    
    // initialize sound driver:
    SoundInit();
    SoundVolumeSet(50);
    
    // register receiver callback:
    cc1101_gpio_detect();
    
#if 0
    wireless_test();
#endif
    
    // enter interphone:
    phone_init();
    
    update_signal_status();
    
    while (1)
    {
        update_phone_status();
        update_signal_status();
        //sleep(1000);
        __wfi();
    }
}
