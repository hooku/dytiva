#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"

#include "m4_common.h"

uint8_t g_sd_err = 0;

char g_pcFilenames[NUM_LIST_STRINGS][MAX_FILENAME_STRING_LEN];

FATFS g_sFatFs;
FILINFO g_sFileInfo;
DIR g_sDirObject;

void sdc_init()
{
    FRESULT fresult;
    volatile uint32_t gpio_val;
    
    // prepare sd card:
    fresult = f_mount(&g_sFatFs, "0:", 1);
    g_sd_err = fresult;
    
    if (fresult != FR_OK)
    {
        lcd_printf(0, 1, "SD Card Err=%d", fresult);
        
        switch (fresult)
        {
            case FR_DISK_ERR:
                lcd_printf(0, 2, "disk I/O err");
                break;
            case FR_NOT_READY:
                lcd_printf(0, 2, "phy drive err");
                break;
            case FR_NO_FILESYSTEM:
                lcd_printf(0, 2, "no valid FATFS");
                break;
            default:
                break;
        }
        
        while (fresult != FR_NOT_READY)
            ;
    }
}
