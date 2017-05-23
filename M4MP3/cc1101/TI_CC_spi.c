/* --COPYRIGHT--,BSD
 * Copyright (c) 2011, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//------------------------------------------------------------------------------
//  Description:  This file contains functions that allow the MSP430 device to
//  access the SPI interface of the CC1100/CC2500.  There are multiple
//  instances of each function; the one to be compiled is selected by the
//  system variable TI_CC_RF_SER_INTF, defined in "TI_CC_hardware_board.h".
//
//  MSP430/CC1100-2500 Interface Code Library v1.1
//
//  W. Goh
//  Texas Instruments, Inc.
//  December 2009
//  IAR Embedded Workbench v4.20
//------------------------------------------------------------------------------
// Change Log:
//------------------------------------------------------------------------------
// Version:  1.1
// Comments: Fixed several bugs where it is stuck in a infinite while loop
//           Added support for 5xx
//
// Version:  1.00
// Comments: Initial Release Version
//------------------------------------------------------------------------------

#include "TI_CC_spi.h"
#include "TI_CC_CC1100-CC2500.h"

#if 1
#include "m4_common.h"
#endif

//------------------------------------------------------------------------------
//  void TI_CC_SPISetup(void)
//
//  DESCRIPTION:
//  Configures the assigned interface to function as a SPI port and
//  initializes it.
//------------------------------------------------------------------------------
//  void TI_CC_SPIWriteReg(char addr, char value)
//
//  DESCRIPTION:
//  Writes "value" to a single configuration register at address "addr".
//------------------------------------------------------------------------------
//  void TI_CC_SPIWriteBurstReg(char addr, char *buffer, char count)
//
//  DESCRIPTION:
//  Writes values to multiple configuration registers, the first register being
//  at address "addr".  First data byte is at "buffer", and both addr and
//  buffer are incremented sequentially (within the CCxxxx and MSP430,
//  respectively) until "count" writes have been performed.
//------------------------------------------------------------------------------
//  char TI_CC_SPIReadReg(char addr)
//
//  DESCRIPTION:
//  Reads a single configuration register at address "addr" and returns the
//  value read.
//------------------------------------------------------------------------------
//  void TI_CC_SPIReadBurstReg(char addr, char *buffer, char count)
//
//  DESCRIPTION:
//  Reads multiple configuration registers, the first register being at address
//  "addr".  Values read are deposited sequentially starting at address
//  "buffer", until "count" registers have been read.
//------------------------------------------------------------------------------
//  char TI_CC_SPIReadStatus(char addr)
//
//  DESCRIPTION:
//  Special read function for reading status registers.  Reads status register
//  at register "addr" and returns the value read.
//------------------------------------------------------------------------------
//  void TI_CC_SPIStrobe(char strobe)
//
//  DESCRIPTION:
//  Special write function for writing to command strobe registers.  Writes
//  to the strobe at address "addr".
//------------------------------------------------------------------------------


// Delay function. # of CPU cycles delayed is similar to "cycles". Specifically,
// it's ((cycles-15) % 6) + 15.  Not exact, but gives a sense of the real-time
// delay.  Also, if MCLK ~1MHz, "cycles" is similar to # of useconds delayed.
void TI_CC_Wait(unsigned int cycles)
{
  while(cycles>15)                          // 15 cycles consumed by overhead
    cycles = cycles - 6;                    // 6 cycles consumed each iteration
}

//void TI_CC_SPISetup(void)
void TI_CC_SPISetup(unsigned long ulSpiClock)
{
//  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
//  TI_CC_CSn_PxDIR |= TI_CC_CSn_PIN;         // /CS disable

//  ME1 |= USPIE0;                            // Enable USART0 SPI mode
//  UCTL0 = SWRST;                            // Disable USART state machine
//  UCTL0 |= CHAR + SYNC + MM;                // 8-bit SPI Master **SWRST**
//  UTCTL0 |= CKPH + SSEL1 + SSEL0 + STC;     // SMCLK, 3-pin mode
//  UBR00 = 0x02;                             // UCLK/2
//  UBR10 = 0x00;                             // 0
//  UMCTL0 = 0x00;                            // No modulation
//  TI_CC_SPI_USART0_PxSEL |= TI_CC_SPI_USART0_SIMO
//                          | TI_CC_SPI_USART0_SOMI
//                          | TI_CC_SPI_USART0_UCLK;
//                                            // SPI option select
//  TI_CC_SPI_USART0_PxDIR |= TI_CC_SPI_USART0_SIMO + TI_CC_SPI_USART0_UCLK;
//                                            // SPI TX out direction
//  UCTL0 &= ~SWRST;                          // Initialize USART state machine
    
    uint32_t ulTemp;
    
    MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_PWM0);
    MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_PWM1);
    
    // configure gdo0:
    MAP_SysCtlPeripheralEnable(CC1101_PERIPH_GDO0);
#if 1
    MAP_GPIOPinTypeGPIOInput(CC1101_GPIO_GDO0, CC1101_PIN_GDO0);
#else
    MAP_GPIODirModeSet(CC1101_GPIO_GDO0, CC1101_PIN_GDO0, GPIO_DIR_MODE_IN);
    MAP_GPIOPadConfigSet(CC1101_GPIO_GDO0, CC1101_PIN_GDO0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
#endif
    
    // configure led:
    MAP_SysCtlPeripheralEnable(CC1101_PERIPH_LED);
    
#if 0
    MAP_GPIOPinTypeGPIOOutput(CC1101_GPIO_LED, CC1101_PIN_LED);
    MAP_GPIOPinWrite(CC1101_GPIO_LED, CC1101_PIN_LED, CC1101_PIN_LED);
#else
    MAP_GPIOPinTypeGPIOInput(CC1101_GPIO_LED, CC1101_PIN_LED);
#endif

#ifdef CC_GPIO_TEST // configure pc5 gpio test port
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5);
#endif // CC_GPIO_TEST
    
    // configure spi:
    MAP_SysCtlPeripheralEnable(CC1101_PERIPH_SPI_PINS);
    MAP_SysCtlPeripheralEnable(CC1101_PERIPH_SPI);
    
    MAP_GPIOPinTypeSSI(CC1101_GPIO_SPI, CC1101_PIN_SPI_TX | CC1101_PIN_SPI_RX | CC1101_PIN_SPI_CLK);
    
    // MISO as weak pull up:
    MAP_GPIOPadConfigSet(CC1101_GPIO_SPI, CC1101_PIN_SPI_RX, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    
    MAP_GPIOPinTypeGPIOOutput(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS);
    
    MAP_SSIConfigSetExpClk(CC1101_PIN_SPI_PORT, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3,
        SSI_MODE_MASTER, ulSpiClock, 8);
    MAP_SSIEnable(CC1101_PIN_SPI_PORT);
    
    while(MAP_SSIDataGetNonBlocking(CC1101_PIN_SPI_PORT, &ulTemp))
        ;
    
    // deselect cs:
    MAP_GPIOPinWrite(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS, CC1101_PIN_SPI_FSS);  // CS
}

void TI_CC_SPIWriteReg(unsigned char addr, unsigned char value)
{
    uint32_t ulTemp;
    
    //MAP_IntMasterDisable();
    MAP_IntPriorityMaskSet(0xFF);
    
  //TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
#if 1
    gpio_ssi2_deselect();
#endif

    MAP_GPIOPinWrite(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS, ~CC1101_PIN_SPI_FSS);  // CS
    
    // warning: must wait MISO to go low (CHIP_RDYn):
    while (GPIOPinRead(CC1101_GPIO_SPI, CC1101_PIN_SPI_RX))
        ;

  //while (!(IFG1&UTXIFG0));                  // Wait for TX to finish
  //U0TXBUF = addr;                           // Send address
  //while (!(IFG1&UTXIFG0));                  // Wait for TX to finish

    ulTemp = addr;
    MAP_SSIDataPut(CC1101_PIN_SPI_PORT, ulTemp);
    MAP_SSIDataGet(CC1101_PIN_SPI_PORT, &ulTemp); // flush data
    
    // TODO: use isr handle this
    while(MAP_SSIBusy(CC1101_PIN_SPI_PORT))
        ;
  
  //U0TXBUF = value;                          // Send value
  //while(!(UTCTL0&TXEPT));                   // Wait for TX complete
    ulTemp = value;
    MAP_SSIDataPut(CC1101_PIN_SPI_PORT, ulTemp);
    MAP_SSIDataGet(CC1101_PIN_SPI_PORT, &ulTemp); // flush data
    
    // TODO: use isr handle this
    while(MAP_SSIBusy(CC1101_PIN_SPI_PORT))
        ;
    
  //TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
    MAP_GPIOPinWrite(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS, CC1101_PIN_SPI_FSS);
    
    //MAP_IntMasterEnable();
    MAP_IntPriorityMaskSet(0);
}

void TI_CC_SPIWriteBurstReg(unsigned char addr, unsigned char *buffer, unsigned char count)
{
    unsigned char i;
    uint32_t ulTemp;
    
    //MAP_IntMasterDisable();
    MAP_IntPriorityMaskSet(0xFF);

  //TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
#if 1
    gpio_ssi2_deselect();
#endif
    MAP_GPIOPinWrite(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS, ~CC1101_PIN_SPI_FSS);  // CS
    
    while (GPIOPinRead(CC1101_GPIO_SPI, CC1101_PIN_SPI_RX))
        ;
    
  //while (!(IFG1 & UTXIFG0));                // Wait for TX to finish
  //U0TXBUF = addr | TI_CCxxx0_WRITE_BURST;   // Send address
    ulTemp = (addr | TI_CCxxx0_WRITE_BURST);
    MAP_SSIDataPut(CC1101_PIN_SPI_PORT, ulTemp);
    MAP_SSIDataGet(CC1101_PIN_SPI_PORT, &ulTemp);
    
    while(MAP_SSIBusy(CC1101_PIN_SPI_PORT))
        ;
    
    for (i = 0; i < count; i++)
    {
    //while (!(IFG1 & UTXIFG0));              // Wait for TX to finish
    //U0TXBUF = buffer[i];                    // Send data
        ulTemp = buffer[i];
        MAP_SSIDataPut(CC1101_PIN_SPI_PORT, ulTemp);
        MAP_SSIDataGet(CC1101_PIN_SPI_PORT, &ulTemp);
      
        while(MAP_SSIBusy(CC1101_PIN_SPI_PORT))
            ;
    }
    //while(!(UTCTL0 & TXEPT));
    //TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
    MAP_GPIOPinWrite(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS, CC1101_PIN_SPI_FSS);
  
    //MAP_IntMasterEnable();
    MAP_IntPriorityMaskSet(0);
}

unsigned char TI_CC_SPIReadReg(unsigned char addr)
{
  char x;
    uint32_t ulTemp;
    
    //MAP_IntMasterDisable();
    MAP_IntPriorityMaskSet(0xFF);

  //TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
#if 1
    gpio_ssi2_deselect();
#endif
    MAP_GPIOPinWrite(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS, ~CC1101_PIN_SPI_FSS);  // CS
    
    while (GPIOPinRead(CC1101_GPIO_SPI, CC1101_PIN_SPI_RX))
        ;
    
  //while (!(IFG1 & UTXIFG0));                // Wait for TX to finish
  //U0TXBUF = (addr | TI_CCxxx0_READ_SINGLE); // Send address
    ulTemp = (addr | TI_CCxxx0_READ_SINGLE);
    MAP_SSIDataPut(CC1101_PIN_SPI_PORT, ulTemp);
    MAP_SSIDataGet(CC1101_PIN_SPI_PORT, &ulTemp);
    
    while(MAP_SSIBusy(CC1101_PIN_SPI_PORT))
        ;
    
    // TODO: read garbage data
    
  //while (!(IFG1 & UTXIFG0));                // Wait for TX to finish
  //U0TXBUF = 0;                              // Dummy write so we can read data
  //while(!(UTCTL0 & TXEPT));                 // Wait for TX complete
  //x = U0RXBUF;                              // Read data
    
    ulTemp = 0x0;
    MAP_SSIDataPut(CC1101_PIN_SPI_PORT, ulTemp);
    MAP_SSIDataGet(CC1101_PIN_SPI_PORT, &ulTemp);
    x = ulTemp;
    
    while(MAP_SSIBusy(CC1101_PIN_SPI_PORT))
        ;
    
  //TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
    MAP_GPIOPinWrite(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS, CC1101_PIN_SPI_FSS);

    //MAP_IntMasterEnable();
    MAP_IntPriorityMaskSet(0);
    
  return x;
}

void TI_CC_SPIReadBurstReg(unsigned char addr, unsigned char *buffer, unsigned char count)
{
  unsigned int i;
    uint32_t ulTemp;
    
    //MAP_IntMasterDisable();
    MAP_IntPriorityMaskSet(0xFF);

//  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
#if 1
    gpio_ssi2_deselect();
#endif
    MAP_GPIOPinWrite(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS, ~CC1101_PIN_SPI_FSS);  // CS
    
    while (GPIOPinRead(CC1101_GPIO_SPI, CC1101_PIN_SPI_RX))
        ;

  //while (!(IFG1 & UTXIFG0));                // Wait for TXBUF ready
  //U0TXBUF = (addr | TI_CCxxx0_READ_BURST);  // Send address
    ulTemp = (addr | TI_CCxxx0_READ_BURST);
    MAP_SSIDataPut(CC1101_PIN_SPI_PORT, ulTemp);
    MAP_SSIDataGet(CC1101_PIN_SPI_PORT, &ulTemp);
    
    while(MAP_SSIBusy(CC1101_PIN_SPI_PORT))
        ;
    
  //while(!(UTCTL0 & TXEPT));                 // Wait for TX complete
  //U0TXBUF = 0;                              // Dummy write to read 1st data byte
//    ulTemp = 0xFF;
//    MAP_SSIDataPut(CC1101_PIN_SPI_PORT, ulTemp);
//    MAP_SSIDataGet(CC1101_PIN_SPI_PORT, &ulTemp);
//    
//    while(MAP_SSIBusy(CC1101_PIN_SPI_PORT))
//        ;
    
  // Addr byte is now being TX'ed, with dummy byte to follow immediately after
  //IFG1 &= ~URXIFG0;                         // Clear flag
  //while (!(IFG1&URXIFG0));                  // Wait for end of 1st data byte TX
  // First data byte now in RXBUF
  for (i = 0; i < count; i++)
  {
    //U0TXBUF = 0;                            // Initiate next data RX, meanwhile
    //buffer[i] = U0RXBUF;                    // Store data from last data RX
    //while (!(IFG1&URXIFG0));                // Wait for end of data RX
        ulTemp = 0x0;
        MAP_SSIDataPut(CC1101_PIN_SPI_PORT, ulTemp);
        MAP_SSIDataGet(CC1101_PIN_SPI_PORT, &ulTemp);

        buffer[i] = ulTemp;
      
        while(MAP_SSIBusy(CC1101_PIN_SPI_PORT))
            ;
  }
  //buffer[count-1] = U0RXBUF;                // Store last RX byte in buffer
  
  //TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
    MAP_GPIOPinWrite(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS, CC1101_PIN_SPI_FSS);

    //MAP_IntMasterEnable();
    MAP_IntPriorityMaskSet(0);
}

// For status/strobe addresses, the BURST bit selects between status registers
// and command strobes.
unsigned char TI_CC_SPIReadStatus(unsigned char addr)
{
  char status;
    uint32_t ulTemp;
    
    //MAP_IntMasterDisable();
    MAP_IntPriorityMaskSet(0xFF);

  //TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
#if 1
    gpio_ssi2_deselect();
#endif
    MAP_GPIOPinWrite(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS, ~CC1101_PIN_SPI_FSS);  // CS
    
    while (GPIOPinRead(CC1101_GPIO_SPI, CC1101_PIN_SPI_RX))
        ;

  //while (!(IFG1 & UTXIFG0));                // Wait for TX to finish
  //U0TXBUF = (addr | TI_CCxxx0_READ_BURST);  // Send address
    ulTemp = (addr | TI_CCxxx0_READ_BURST);
    MAP_SSIDataPut(CC1101_PIN_SPI_PORT, ulTemp); 
    MAP_SSIDataGet(CC1101_PIN_SPI_PORT, &ulTemp);
    
    while(MAP_SSIBusy(CC1101_PIN_SPI_PORT))
        ;

  //while (!(IFG1 & UTXIFG0));                // Wait for TX to finish
  //U0TXBUF = 0;                              // Dummy write so we can read data
  //while(!(UTCTL0 & TXEPT));                 // Wait for TX complete
  //status = U0RXBUF;                         // Read data
    ulTemp = 0x0;
    MAP_SSIDataPut(CC1101_PIN_SPI_PORT, ulTemp);     
    MAP_SSIDataGet(CC1101_PIN_SPI_PORT, &ulTemp);
    status = ulTemp;
    
    while(MAP_SSIBusy(CC1101_PIN_SPI_PORT))
        ;
    
  //TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
    MAP_GPIOPinWrite(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS, CC1101_PIN_SPI_FSS);

    //MAP_IntMasterEnable();
    MAP_IntPriorityMaskSet(0);

  return status;
}

void TI_CC_SPIStrobe(unsigned char strobe)
{
    uint32_t ulTemp;
    
    //MAP_IntMasterDisable();
    MAP_IntPriorityMaskSet(0xFF);

  //TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
#if 1
    gpio_ssi2_deselect();
#endif
    MAP_GPIOPinWrite(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS, ~CC1101_PIN_SPI_FSS);  // CS
    
    while (GPIOPinRead(CC1101_GPIO_SPI, CC1101_PIN_SPI_RX))
        ;

  //while (!(IFG1 & UTXIFG0));                // Wait for TX to finish
  //U0TXBUF = strobe;                         // Send strobe
  // Strobe addr is now being TX'ed
  //IFG1 &= ~URXIFG0;                         // Clear flag
  //while(!(UTCTL0 & TXEPT));                 // Wait for TX complete
    
    ulTemp = strobe;
    MAP_SSIDataPut(CC1101_PIN_SPI_PORT, ulTemp);
    MAP_SSIDataGet(CC1101_PIN_SPI_PORT, &ulTemp);
    
    while(MAP_SSIBusy(CC1101_PIN_SPI_PORT))
        ;

  //TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
    MAP_GPIOPinWrite(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS, CC1101_PIN_SPI_FSS);

    //MAP_IntMasterEnable();
    MAP_IntPriorityMaskSet(0);
}

void TI_CC_PowerupResetCCxxxx(void)
{
    uint32_t ulTemp;
    
    //MAP_IntMasterDisable();
    MAP_IntPriorityMaskSet(0xFF);
    
//  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
    MAP_GPIOPinWrite(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS, CC1101_PIN_SPI_FSS);

  TI_CC_Wait(300);
  //TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;
    MAP_GPIOPinWrite(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS, ~CC1101_PIN_SPI_FSS);  // CS

  TI_CC_Wait(300);
  //TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN
    MAP_GPIOPinWrite(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS, CC1101_PIN_SPI_FSS);
    
  TI_CC_Wait(450);

  //TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
    MAP_GPIOPinWrite(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS, ~CC1101_PIN_SPI_FSS);  // CS

    while (GPIOPinRead(CC1101_GPIO_SPI, CC1101_PIN_SPI_RX))
        ;
    
  //while (!(IFG1 & UTXIFG0));                // Wait for TX to finish
  //U0TXBUF = TI_CCxxx0_SRES;                 // Send strobe
    ulTemp = TI_CCxxx0_SRES;
    MAP_SSIDataPut(CC1101_PIN_SPI_PORT, ulTemp);
    MAP_SSIDataGet(CC1101_PIN_SPI_PORT, &ulTemp);
    
    while(MAP_SSIBusy(CC1101_PIN_SPI_PORT))
        ;
    
  // Strobe addr is now being TX'ed
  //while(!(UTCTL0 & TXEPT));                 // Wait for TX complete
  //TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
    MAP_GPIOPinWrite(CC1101_GPIO_SPI, CC1101_PIN_SPI_FSS, CC1101_PIN_SPI_FSS);

    //MAP_IntMasterEnable();
    MAP_IntPriorityMaskSet(0);
}
