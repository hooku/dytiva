#include "cc1101/CC1100-CC2500.h"
#include "cc1101/TI_CC_spi.h"
#include "cc1101/TI_CC_CC1100-CC2500.h"

#include "m4_config.h"
#include "m4_common.h"

extern char paTable[];
extern char paTableLen;

// PKT Format:
//  Preamble    | Sync      | Length    | Address   | Data      | CRC16
//  nB          | 1/2B      | 1B        | 1B        | 254B      | 2B

#ifdef CC1101_NO_BUFF
char *cc_tx_buffer;
char *cc_rx_buffer;
#else // CC1101_NO_BUFF
char cc_tx_buffer[MAX_TX_RX_BUFF];
char cc_rx_buffer[MAX_TX_RX_BUFF];
#endif // CC1101_NO_BUFF

// callback array
void (*cc1101_int_handler)();

// pin:
// PF2=GDO(ATEST)
// PF3=LED

void cc1101_init(void)
{
    TI_CC_SPISetup(CC1101_RATE);
    
    TI_CC_PowerupResetCCxxxx();
    
    writeRFSettings();
    
    TI_CC_SPIWriteBurstReg(TI_CCxxx0_PATABLE, paTable, paTableLen);//Write PATABLE

    TI_CC_SPIStrobe(TI_CCxxx0_SRX);           // Initialize CCxxxx in RX mode
}

#ifdef CC1101_NO_BUFF
#else // CC1101_NO_BUFF
void cc1101_send(unsigned char len)
{
    RFSendPacket(mem_tx_buffer, len);   // send length including length byte
}

uint8_t cc1101_receive(char *data)
{
    char len = MAX_TX_RX_BUFF;
    
//    // switch to tx mode:
//    TI_CC_SPIStrobe(TI_CCxxx0_SRX);
//    // wait until gdo0 become low:
//    while (GPIOPinRead(CC1101_GPIO_GDO0, CC1101_PIN_GDO0) == CC1101_PIN_GDO0)
//        ;
    
    RFReceivePacket(mem_rx_buffer, &len);
    
    data = cc_rx_buffer;
    
    return len;
}
#endif // CC1101_NO_BUFF

int8_t cc1101_get_signal()
{
    return (int8_t)TI_CC_SPIReadStatus(TI_CCxxx0_RSSI);
}

void cc1101_test(void)
{
//    char testBuffer[0x3A] = { 0 };
//    TI_CC_SPIReadBurstReg(TI_CCxxx0_IOCFG2, testBuffer, 0x3A);

//    while (1)
//        ;
    TI_CC_SPIStrobe(TI_CCxxx0_SFRX);
    TI_CC_SPIStrobe(TI_CCxxx0_SRX);
    
}

void cc1101_gpio_detect(void)
{
    MAP_IntEnable(CC1101_INT_GDO0);
    
    MAP_GPIOIntTypeSet(CC1101_GPIO_GDO0, CC1101_PIN_GDO0, GPIO_FALLING_EDGE);

    MAP_GPIOIntEnable(CC1101_GPIO_GDO0, CC1101_PIN_GDO0);
}

void cc1101_int_isr(void)
{
    //TI_CC_LED_PxOUT ^= rxBuffer[1];               // Toggle LEDs according to pkt data
    //MAP_GPIOPinWrite(CC1101_GPIO_LED, CC1101_PIN_LED, CC1101_PIN_LED | cc_rx_buffer[1]);

    //TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;          // Clear flag
    if (cc1101_int_handler)
    {
        (*cc1101_int_handler)();
    }
    else
    {
        //TI_CC_SPIStrobe(TI_CCxxx0_SFRX);
    }
}
    
void cc1101_int_register(void (*func)(void))
{
    cc1101_int_handler = func;
}
