#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <string.h>

//#include "core_cm4.h"

#include "inc/hw_nvic.h"
#include "inc/hw_types.h"

#include "driverlib/flash.h"
#include "driverlib/sysctl.h"

#include "m4_common.h"

#define GUEST_APP_OFFSET        0x00020000  // user application start address
#define GUEST_APP_SIZE          0x00008000  // 32K

static uint8_t g_bin_count;

const uint32_t sys_periph_table[] = { SYSCTL_PERIPH_ADC0, SYSCTL_PERIPH_ADC1,
    SYSCTL_PERIPH_CAN0, SYSCTL_PERIPH_CAN1, SYSCTL_PERIPH_CCM0,
    SYSCTL_PERIPH_COMP0, SYSCTL_PERIPH_EEPROM0, SYSCTL_PERIPH_EPI0, SYSCTL_PERIPH_GPIOA,
    SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOC, SYSCTL_PERIPH_GPIOD,
    SYSCTL_PERIPH_GPIOE, SYSCTL_PERIPH_GPIOF, SYSCTL_PERIPH_GPIOG,
    SYSCTL_PERIPH_GPIOH, SYSCTL_PERIPH_GPIOJ, SYSCTL_PERIPH_GPIOK,
    SYSCTL_PERIPH_GPIOL, SYSCTL_PERIPH_GPIOM, SYSCTL_PERIPH_GPION,
    SYSCTL_PERIPH_GPIOP, SYSCTL_PERIPH_GPIOQ, SYSCTL_PERIPH_GPIOR,
    SYSCTL_PERIPH_GPIOS, SYSCTL_PERIPH_GPIOT, SYSCTL_PERIPH_HIBERNATE,
    SYSCTL_PERIPH_I2C0, SYSCTL_PERIPH_I2C1, SYSCTL_PERIPH_I2C2,
    SYSCTL_PERIPH_I2C3, SYSCTL_PERIPH_I2C4, SYSCTL_PERIPH_I2C5,
    SYSCTL_PERIPH_I2C6, SYSCTL_PERIPH_I2C7, SYSCTL_PERIPH_I2C8,
    SYSCTL_PERIPH_I2C9, SYSCTL_PERIPH_LCD0, SYSCTL_PERIPH_ONEWIRE0,
    SYSCTL_PERIPH_PWM0, SYSCTL_PERIPH_PWM1, SYSCTL_PERIPH_QEI0,
    SYSCTL_PERIPH_QEI1, SYSCTL_PERIPH_SSI0, SYSCTL_PERIPH_SSI1,
    SYSCTL_PERIPH_SSI2, SYSCTL_PERIPH_SSI3, SYSCTL_PERIPH_TIMER0,
    SYSCTL_PERIPH_TIMER1, SYSCTL_PERIPH_TIMER2, SYSCTL_PERIPH_TIMER3,
    SYSCTL_PERIPH_TIMER4, SYSCTL_PERIPH_TIMER5, SYSCTL_PERIPH_TIMER6,
    SYSCTL_PERIPH_TIMER7, SYSCTL_PERIPH_UART0, SYSCTL_PERIPH_UART1,
    SYSCTL_PERIPH_UART2, SYSCTL_PERIPH_UART3, SYSCTL_PERIPH_UART4,
    SYSCTL_PERIPH_UART5, SYSCTL_PERIPH_UART6, SYSCTL_PERIPH_UART7,
    SYSCTL_PERIPH_UDMA, SYSCTL_PERIPH_USB0, SYSCTL_PERIPH_WDOG0,
    SYSCTL_PERIPH_WDOG1, SYSCTL_PERIPH_WTIMER0, SYSCTL_PERIPH_WTIMER1,
    SYSCTL_PERIPH_WTIMER2, SYSCTL_PERIPH_WTIMER3,
    SYSCTL_PERIPH_WTIMER4, SYSCTL_PERIPH_WTIMER5,
};

void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}

void boot_loader_entry(void)
{
    void (*guest_app_reset_handler)(void);
    
    // perform a clean up on sys percipherals (like soft reset):
    uint8_t i_periph;
    
    for (i_periph = 0; i_periph < (sizeof(sys_periph_table)/sizeof(uint32_t)); i_periph ++)
    {
        SysCtlPeripheralReset(sys_periph_table[i_periph]);
    }
    
    // boot into user code:
    HWREG(NVIC_VTABLE) = GUEST_APP_OFFSET;
    
    // no need to reconfigure sp pointer?
    //__set_MSP(*(volatile uint32_t*) 0x00020000);
    
    guest_app_reset_handler = (void (*)(void))(*(uint32_t *)(GUEST_APP_OFFSET + 4));
    
    // time to call reset handler:
    (*guest_app_reset_handler)();
}

void burn_app(uint8_t bin_index)
{
#define FLASH_BLOCK_SIZE        SysCtlFlashSectorSizeGet()  // 1k
#define FLASH_PROGRAM_SIZE      0x4
    
    FIL file;
    FRESULT file_result;
    uint16_t file_size, erase_size;
    uint32_t read_bytes;
    
    union __binary
    {
        uint8_t buffer[FLASH_PROGRAM_SIZE];
        uint32_t reset_handler_addr;
    } binary;
    uint8_t flash_buffer[FLASH_PROGRAM_SIZE];   // content retrieve from flash
    
    uint32_t flash_addr;
    
    Boot_Reason boot_reason;
    
    file_result = f_open(&file, g_pcFilenames[g_bin_count], FA_READ);
    file_size = f_size(&file);

    lcd_printf(0, 1, "%s                ", g_pcFilenames[bin_index]);
    lcd_printf(0, 3, "%d KB             ", file_size/1024);
    
    // === check ===:
    // we need to read the second 4 bytes to check reset_handler's address > 
    f_lseek(&file, FLASH_PROGRAM_SIZE);
    file_result = f_read(&file, &binary.reset_handler_addr, FLASH_PROGRAM_SIZE, &read_bytes);
    
    if (binary.reset_handler_addr < GUEST_APP_OFFSET)
    {
        lcd_printf(0, 2, "ROM1 Addr Err   ");
        
        while (1)
        {
            __wfi();
        }
    }
    
    // === erase ===:
    // we only need to erase size > guest app size (all bytes 0xFF)
    // tm4c123x use 1k block
    if (file_size % FLASH_BLOCK_SIZE == 0)
    {
        erase_size = file_size;
    }
    else
    {
        erase_size = file_size - (file_size % FLASH_BLOCK_SIZE) + FLASH_BLOCK_SIZE;
    }

    for (flash_addr = GUEST_APP_OFFSET; flash_addr < (GUEST_APP_OFFSET + erase_size);
        flash_addr += FLASH_BLOCK_SIZE)
    {
        FlashErase(flash_addr);
    }
    
    sleep(250);
    
    // === burn ===:
    lcd_printf(0, 2, "Burning..       ");
    lcd_printf(8, 3, "        ");
    
    f_lseek(&file, 0);
    
    for (flash_addr = GUEST_APP_OFFSET; flash_addr < (GUEST_APP_OFFSET + file_size);
        flash_addr += FLASH_PROGRAM_SIZE)
    {
        // it seems that the size of binary file is always a multiple of 4:
        file_result = f_read(&file, binary.buffer, FLASH_PROGRAM_SIZE, &read_bytes);
        
        // programme sizemust be a multiple of 4:
        FlashProgram((uint32_t *)binary.buffer, flash_addr, FLASH_PROGRAM_SIZE);
        
        *(uint32_t *)flash_buffer = *((uint32_t *)flash_addr);
    }
    
    sleep(250);
    
    // === verify ===:
    lcd_printf(0, 2, "Verifying..     ");
    lcd_printf(8, 3, "        ");
    
    f_lseek(&file, 0);
    
    for (flash_addr = GUEST_APP_OFFSET; flash_addr < (GUEST_APP_OFFSET + file_size);
        flash_addr += FLASH_PROGRAM_SIZE)
    {
        // we read 4 bytes(a word) from flash
        *(uint32_t *)flash_buffer = *((uint32_t *)flash_addr);
        
        file_result = f_read(&file, binary.buffer, FLASH_PROGRAM_SIZE, &read_bytes);
        
        if (memcmp(binary.buffer, flash_buffer, FLASH_PROGRAM_SIZE) != 0)  // not equal
        {
            lcd_printf(0, 2, "Content corrupt ");
            while (1)
            {
                __wfi();
            }
        }
    }
    
    lcd_printf(0, 2, "OK            ");
    
    sleep(250);
    
#if 1
    boot_reason = BR_BOOTLOADER;
    boot_reason_update(&boot_reason);
    SysCtlReset();
#else
    boot_loader_entry();
#endif
    
    while (1)
    {
        __wfi();
    }
}

void gpio_button_down_sdload(Button button)
{
    static uint32_t last_key_down_tick;

    if ((sys_tick_count - last_key_down_tick) < KEY_INTERVAL_THRESHOLD)
    {
        m4_dbgprt("Jitter\r\n");
        return ;
    }
    last_key_down_tick = sys_tick_count;
    
    switch (button)
    {
        case Key_1:
        case Key_2:
        case Key_3:
            burn_app(button);
            break;
        case Key_4:
        default:
            break;
    }
}

void app_main_sdload()
{
    FRESULT file_result;
    
    char *file_name;

#define MAX_BINARY_COUNT    3
    
    // phone mode:
    lcd_clear();
    lcd_printf(0xFF, 0, "SDLoad");
    
    // register key call back:
    gpio_int_register(&gpio_button_down_sdload);
    
    // traverse root directory to find binary file
    
    file_result = f_opendir(&g_sDirObject, "0:/");
    
    while (1)
    {
        file_result = f_readdir(&g_sDirObject, &g_sFileInfo);
        if (file_result != FR_OK || g_sFileInfo.fname[0] == 0)
            break;  // Break on error or end of dir

        if (g_sFileInfo.fname[0] == '.')
            continue;
        
        file_name = g_sFileInfo.fname;
        
        if (g_sFileInfo.fattrib & AM_DIR)   // directory
        {
            ;
        }
        else                                // file
        {
            if ((strstr(file_name, ".bin") != NULL) || (strstr(file_name, ".BIN") != NULL))
            {
                g_bin_count ++;
                
                strcpy(g_pcFilenames[g_bin_count], file_name);
                
                lcd_printf(0, g_bin_count, "K%d: %s", g_bin_count, file_name);
                
                if (g_bin_count > MAX_BINARY_COUNT)
                {
                    break;
                }
            }
        }
    }
    
    f_closedir(&g_sDirObject);
    
    while (1)
    {
        __wfi();
    }
}
