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
//  Description:  Header file for TI_CC_spi.c
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
// Comments: Fixed function bugs
//           Added support for 5xx
//
// Version:  1.00
// Comments: Initial Release Version
//------------------------------------------------------------------------------
#ifndef __TI_CC_SPI_H__
#define __TI_CC_SPI_H__

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/rom_map.h"

//#define CC_GPIO_TEST

#define CC1101_PERIPH_GDO0              SYSCTL_PERIPH_GPIOF
#define CC1101_PERIPH_LED               SYSCTL_PERIPH_GPIOF
#define CC1101_PERIPH_SPI_PINS          SYSCTL_PERIPH_GPIOB
#define CC1101_PERIPH_SPI               SYSCTL_PERIPH_SSI2

#define CC1101_GPIO_GDO0                GPIO_PORTF_BASE
#define CC1101_GPIO_LED                 GPIO_PORTF_BASE
#define CC1101_GPIO_SPI                 GPIO_PORTB_BASE

#define CC1101_PIN_GDO0                 GPIO_PIN_3
#define CC1101_PIN_LED                  GPIO_PIN_2

#define CC1101_PIN_SPI_PORT             SSI2_BASE

#define CC1101_PIN_SPI_TX               GPIO_PIN_7
#define CC1101_PIN_SPI_RX               GPIO_PIN_6
#define CC1101_PIN_SPI_CLK              GPIO_PIN_4
#define CC1101_PIN_SPI_FSS              GPIO_PIN_5

#define CC1101_INT_GDO0                 INT_GPIOF

//void TI_CC_SPISetup(void);
void TI_CC_SPISetup(unsigned long ulSpiClock);
void TI_CC_PowerupResetCCxxxx(void);
void TI_CC_SPIWriteReg(unsigned char, unsigned char);
void TI_CC_SPIWriteBurstReg(unsigned char, unsigned char*, unsigned char);
unsigned char TI_CC_SPIReadReg(unsigned char);
void TI_CC_SPIReadBurstReg(unsigned char, unsigned char *, unsigned char);
unsigned char TI_CC_SPIReadStatus(unsigned char);
void TI_CC_SPIStrobe(unsigned char);
void TI_CC_Wait(unsigned int);

#endif // __TI_CC_SPI_H__
