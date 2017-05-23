//*****************************************************************************
//
// sound.c - Functions supporting playback of .wav audio files on LM4F232.
//
// Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 8049 of the Stellaris Firmware Development Package.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup sound_api
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "inc/hw_ssi.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/udma.h"
#include "driverlib/timer.h"
#include "fatfs/ff.h"
#include "fatfs/diskio.h"
#include "utils/ustdlib.h"
#include "tlv320aic23b.h"
#include "sound.h"

#include "cc1101/TI_CC_CC1100-CC2500.h"

#include "m4_common.h"

#include "m4_config.h"

#if 1//def M4_USBMSC
#else // M4_USBMSC
//*****************************************************************************
//
// The DMA control structure table.  This is required by the sound driver.
//
//*****************************************************************************
#ifdef ewarm
#pragma data_alignment=1024
tDMAControlTable sDMAControlTable[64];
#elif defined(ccs)
#pragma DATA_ALIGN(sDMAControlTable, 1024)
tDMAControlTable sDMAControlTable[64];
#else
tDMAControlTable sDMAControlTable[64] __attribute__ ((aligned(1024)));
#endif
#endif // M4_USBMSC

#ifdef M4_I2S
//*****************************************************************************
//
// Definitions of psuedo I2S pins used for the sound driver.
//
//*****************************************************************************
#define I2S_CHAN0_PERIPH        (SYSCTL_PERIPH_SSI1)
#define I2S_CHAN0_GPIO          (SYSCTL_PERIPH_GPIOF)
#define I2S_CHAN0_CLK           (GPIO_PF2_SSI1CLK)
#define I2S_CHAN0_FSS           (GPIO_PF3_SSI1FSS)
#define I2S_CHAN0_RX            (GPIO_PF0_SSI1RX)
#define I2S_CHAN0_TX            (GPIO_PF1_SSI1TX)
#define I2S_CHAN0_PORT          (GPIO_PORTF_BASE)
#define I2S_CHAN0_BASE          (SSI1_BASE)
#define I2S_CHAN0_UDMA_TX       (UDMA_CHANNEL_SSI1TX)
#define I2S_CHAN0_INT           (INT_SSI1)

#define I2S_UDMA_PERIPH         (SYSCTL_PERIPH_UDMA)

#define I2S_CHAN1_PERIPH        (SYSCTL_PERIPH_SSI3)
#define I2S_CHAN1_GPIO          (SYSCTL_PERIPH_GPIOH)
#define I2S_CHAN1_CLK           (GPIO_PH0_SSI3CLK)
#define I2S_CHAN1_FSS           (GPIO_PH1_SSI3FSS)
#define I2S_CHAN1_RX            (GPIO_PH2_SSI3RX)
#define I2S_CHAN1_TX            (GPIO_PH3_SSI3TX)
#define I2S_CHAN1_PORT          (GPIO_PORTH_BASE)
#define I2S_CHAN1_BASE          (SSI3_BASE)
#define I2S_CHAN1_UDMA_TX       (UDMA_CH15_SSI3TX)
#define I2S_CHAN1_INT           (INT_SSI3)

#define I2S_MASTERCLK_PERIPH    (SYSCTL_PERIPH_TIMER5)
#define I2S_MASTERCLK_GPIO      (SYSCTL_PERIPH_GPIOG)
#define I2S_MASTERCLK_PORT      (GPIO_PG3_T5CCP1)
#define I2S_MASTERCLK_BASE      (GPIO_PORTG_BASE)
#define I2S_MASTERCLK_PIN       (GPIO_PIN_3)
#define I2S_MASTERCLK_TMR       (TIMER_B)
#define I2S_MASTERCLK_TMR_B     (TIMER5_BASE)
#else // M4_I2S

#define SSI_AIC23_PERIPH        SYSCTL_PERIPH_SSI1
#define SSI_AIC23_UDMA          SYSCTL_PERIPH_UDMA

#define SSI_UDMA_TX				UDMA_CH11_SSI1TX

#define SSI_AIC23_CLK			GPIO_PD0_SSI1CLK
#define SSI_AIC23_FSS			GPIO_PD1_SSI1FSS
#define SSI_AIC23_RX			GPIO_PD2_SSI1RX
#define SSI_AIC23_TX			GPIO_PD3_SSI1TX

#define SSI_AIC23_PORT			GPIO_PORTD_BASE
#define SSI_AIC23_PIN			(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)

#define SSI_AIC23_BASE			SSI1_BASE
#define SSI_AIC23_INT           INT_SSI1

#define SSI_AIC23_MODE          SSI_FRF_MOTO_MODE_2

#endif // M4_I2S

//*****************************************************************************
//
// This table must be defined in any application wishing to use the sound
// driver since I2S makes use of DMA.
//
//*****************************************************************************
extern tDMAControlTable sDMAControlTable[];

//*****************************************************************************
//
// The number of buffers to use for buffering audio playback.
//
//*****************************************************************************
#define NUM_BUFFERS             2

//*****************************************************************************
//
// The rate at which to trigger the periodic interrupt to continue audio
// playback. 
//
//*****************************************************************************
unsigned long g_ulAudioUpdateRate;

//*****************************************************************************
//
// Flag values used to track the uDMA state.
//
//*****************************************************************************
#define FLAG_RX_PENDING         0
#define FLAG_TX_PENDING         1
static volatile unsigned long g_ulDMAFlags;

//*****************************************************************************
//
// Global header information parsed from the opened .wav file.
//
//*****************************************************************************
tWaveHeader *g_WaveHeader;

//*****************************************************************************
//
// Buffer management structures and definition.
//
//*****************************************************************************
static struct
{
    //
    // Pointer to the buffer.
    //
    unsigned long *pulData;

    //
    // Size of the buffer.
    //
    unsigned long ulSize;

    //
    // Callback function for this buffer.
    //
    void (* pfnBufferCallback)(void *pvBuffer, unsigned long ulEvent);
}
g_sOutBuffers[NUM_BUFFERS];

//*****************************************************************************
//
// The current volume of the audio.
//
//*****************************************************************************
static unsigned short g_ucVolume = 50;

//*****************************************************************************
//
// The buffer index that is currently playing.
//
//*****************************************************************************
static unsigned long g_ulPlaying;

//*****************************************************************************
//
// Information on the sound currently being played.
//
//*****************************************************************************
static unsigned long g_ulSampleRate;
static unsigned short g_usChannels;
static unsigned long g_usBitsPerSample;

//*****************************************************************************
//
// Global state variable to signify if the playback is paused.
//
//*****************************************************************************
bool g_Paused = false;

//*****************************************************************************
//
// Basic wav file RIFF header information used to open and read a wav file.
//
//*****************************************************************************
#define RIFF_CHUNK_ID_RIFF      0x46464952
#define RIFF_CHUNK_ID_FMT       0x20746d66
#define RIFF_CHUNK_ID_DATA      0x61746164

#define RIFF_TAG_WAVE           0x45564157

#define RIFF_FORMAT_UNKNOWN     0x0000
#define RIFF_FORMAT_PCM         0x0001
#define RIFF_FORMAT_MSADPCM     0x0002
#define RIFF_FORMAT_IMAADPCM    0x0011

//*****************************************************************************
//
// Audio buffer size and flags definitions  for indicating state of the buffer.
//
//*****************************************************************************
#define AUDIO_BUFFER_SIZE       4096

#define PHONE_BUFFER_SIZE       2040    // a multiple of 60 (two buffers, recording + playing)

#define BUFFER_BOTTOM_EMPTY     0x00000001
#define BUFFER_TOP_EMPTY        0x00000002
#define BUFFER_PLAYING          0x00000004

#define AUDIO_IDLE              0x00000000
#define AUDIO_RECORD            0x00000007
#define AUDIO_PHONE             0x0000000F

#define PHONE_IDLE              0x00
#define PHONE_RECORDING         0x01
#define PHONE_PLAYING           0x02    // receiving

//*****************************************************************************
//
// Allocate the audio clip buffer.
//
//*****************************************************************************
static unsigned char g_pucBuffer[AUDIO_BUFFER_SIZE];
#define phone_mic_buffer        g_pucBuffer
#define phone_play_buffer       g_pucBuffer + PHONE_BUFFER_SIZE

static unsigned char *g_pucDataPtr;
static unsigned long g_ulMaxBufferSize;

static unsigned char *mic_data_ptr;
static unsigned char *play_data_ptr;

static unsigned char mic_store_ing ;

//*****************************************************************************
//
// Holds the state of the audio buffer.
//
//*****************************************************************************
static volatile unsigned long g_ulFlags;

static unsigned char phone_mode     = PHONE_IDLE;       // recording or playing
static unsigned char phone_signal   = 0xFF;             // rssi

//*****************************************************************************
//
// Variables used to track playback position and time.
//
//*****************************************************************************
static unsigned long g_ulBytesPlayed;
static unsigned long g_ulBytesRemaining;
static unsigned short g_usMinutes;
static unsigned short g_usSeconds;

//******************************************************************************
//
// The following are data structures used by FatFs.
//
//******************************************************************************
static FIL g_sFileObject;

//*****************************************************************************
//
// The interrupt handler for Timer2.  This handler will 
// periodically call SoundPlayContinue() - to continue playback of 
// audio data. 
//
//*****************************************************************************
void
Timer2AIntHandler(void)
{
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    SoundPlayContinue(g_WaveHeader);
}

//*****************************************************************************
//
//! Initialize the sound driver.
//!
//! This function initializes the hardware components of the LM4F board,
//! necessary for audio playback. 
//!
//! \return None.
//
//*****************************************************************************
void
SoundInit(void)
{
    //
    // Set the current active buffer to zero.
    //
    g_ulPlaying = 0;

#ifdef M4_I2S
    //
    // Enable and reset the necessary peripherals.
    //
    SysCtlPeripheralEnable(I2S_CHAN1_PERIPH);
    SysCtlPeripheralEnable(I2S_CHAN0_PERIPH);
    SysCtlPeripheralEnable(I2S_UDMA_PERIPH);
    SysCtlPeripheralEnable(I2S_CHAN0_GPIO);
    SysCtlPeripheralEnable(I2S_CHAN1_GPIO);    
    SysCtlPeripheralReset(I2S_CHAN0_GPIO);
    SysCtlPeripheralReset(I2S_CHAN1_GPIO);
#else // M4_I2S
    SysCtlPeripheralEnable(SSI_AIC23_PERIPH);
	SysCtlPeripheralEnable(SSI_AIC23_UDMA);

#endif // M4_I2S

    //
    // Set up Timer to periodically call WavePlayContinue().
    // Timer is enabled when playback is started.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);

#ifdef M4_I2S
    //
    // Set up the pin mux for  both SSI ports.
    //
    GPIOPinConfigure(I2S_CHAN1_CLK);
    GPIOPinConfigure(I2S_CHAN1_FSS);
    GPIOPinConfigure(I2S_CHAN1_RX);
    GPIOPinConfigure(I2S_CHAN1_TX);
    GPIOPinConfigure(I2S_CHAN0_RX);
    GPIOPinConfigure(I2S_CHAN0_TX);
    GPIOPinConfigure(I2S_CHAN0_CLK);
    GPIOPinConfigure(I2S_CHAN0_FSS);

    //
    // Select alternate functions for all of the SSI pins.
    //
    GPIOPinTypeSSI(I2S_CHAN0_PORT,
                   I2S_MASTERCLK_PIN|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);
    GPIOPinTypeSSI(I2S_CHAN1_PORT, 
                   I2S_MASTERCLK_PIN|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);
#else // M4_I2S
	
	GPIOPinConfigure(SSI_AIC23_CLK);
	GPIOPinConfigure(SSI_AIC23_FSS);
	GPIOPinConfigure(SSI_AIC23_RX);
	GPIOPinConfigure(SSI_AIC23_TX);

	GPIOPinTypeSSI(SSI_AIC23_PORT, SSI_AIC23_PIN);

#endif // M4_I2S

    //
    // Set up the DMA.
    //
    ROM_uDMAControlBaseSet(&sDMAControlTable[0]);
    ROM_uDMAEnable();

#ifdef M4_AIC3107
    //
    // Initialize the DAC.
    //
    DACInit();
#else // M4_AIC3107
    TLV320AIC23BInit();
#endif // M4_AIC3107

#if 0
    //
    // Configure the SSI clock (CCP timer).
    //
    setupCCP();
#endif

#ifdef M4_I2S
    //
    // Clear out all pending interrupts.
    // SSI_TXFF: TX FIFO half full or less
    //
    SSIIntClear(I2S_CHAN0_BASE, SSI_TXFF );
    SSIIntClear(I2S_CHAN1_BASE, SSI_TXFF );

    //
    // Enables individual SSI interrupt source.
    // We'll use SSI1's empty interrupt status to drive the FIFO transfers
    // for both SSI1 and SSI3. This interrupt occurs whenever the TX FIFO
    // is half full or less. 
    //
    SSIIntEnable(I2S_CHAN0_BASE, SSI_TXFF);
    //SSIIntEnable(I2S_CHAN1_BASE, SSI_TXFF);

    //
    // Disable all uDMA attributes.
    //
    uDMAChannelAttributeDisable(I2S_CHAN0_UDMA_TX, UDMA_ATTR_ALL);
    uDMAChannelAttributeDisable(I2S_CHAN1_UDMA_TX, UDMA_ATTR_ALL);

    //
    // Configure the synchronous serial interfaces, SSI1 and SSI3.
    // Clock each at the system clock rate of 50MHz.
    // Use the Motorola Mode 2 protocol.
    // As the DAC is the master, set the SSI as a slave device.
    // Set the bit rate to 1M.
    // Set the width of the data transfers to the maximum : 16bits.
    //
    SSIConfigSetExpClk(I2S_CHAN0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_2,
                       SSI_MODE_SLAVE, 1000000, 16);
    SSIConfigSetExpClk(I2S_CHAN1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_2,
                       SSI_MODE_SLAVE, 1000000, 16);
    //
    // Enable the SSI controller.
    //
    SSIEnable(I2S_CHAN0_BASE);
    SSIEnable(I2S_CHAN1_BASE);

    //
    // Enable SSI DMA transmit operation.
    //
    SSIDMAEnable(I2S_CHAN0_BASE, SSI_DMA_TX);
    SSIDMAEnable(I2S_CHAN1_BASE, SSI_DMA_TX);

#else // M4_I2S

	SSIIntClear(SSI_AIC23_BASE, SSI_TXFF);
	SSIIntEnable(SSI_AIC23_BASE, SSI_TXFF);
	uDMAChannelAttributeDisable(SSI_UDMA_TX, UDMA_ATTR_ALL);
	SSIConfigSetExpClk(SSI_AIC23_BASE, SysCtlClockGet(), SSI_AIC23_MODE,
		SSI_MODE_SLAVE, SSI1_RATE, 16);
	SSIEnable(SSI_AIC23_BASE);
	SSIDMAEnable(SSI_AIC23_BASE, SSI_DMA_TX);

#if 1

    // make other percipheral priority lower:
    IntPriorityGroupingSet(1);
    IntPrioritySet(INT_SSI1     , 0x00);
    
    IntPrioritySet(INT_SSI0     , 0xF0);
    IntPrioritySet(INT_SSI2     , 0xF0);
    IntPrioritySet(INT_GPIOA    , 0xF0);
    IntPrioritySet(INT_GPIOB    , 0xF0);
    IntPrioritySet(INT_GPIOC    , 0xF0);
    IntPrioritySet(INT_GPIOD    , 0xF0);
    IntPrioritySet(INT_GPIOE    , 0xF0);
    IntPrioritySet(INT_GPIOF    , 0xF0);
    IntPrioritySet(INT_GPIOG    , 0xF0);
    IntPrioritySet(INT_TIMER0A  , 0xF0);
    IntPrioritySet(INT_TIMER0B  , 0xF0);
    IntPrioritySet(INT_TIMER1A  , 0xF0);
    IntPrioritySet(INT_TIMER1B  , 0xF0);
    //IntPrioritySet(INT_TIMER2A  , 0xF0);
    IntPrioritySet(INT_TIMER2B  , 0xF0);
    IntPrioritySet(INT_TIMER3A  , 0xF0);
    IntPrioritySet(INT_TIMER3B  , 0xF0);
    IntPrioritySet(INT_UART0    , 0xF0);
    
#endif

#endif // M4_I2S
}

//*****************************************************************************
//
// Interrupt handler for the SSI sound driver.
//
// This interrupt function is called by the processor due to an interrupt from
// the SSI peripheral. uDMA is used in ping-pong mode to keep sound buffer
// data flowing to the dual-SSI audio output.  As each buffer transfer is
// complete, the client callback function that was specified in the call
// to SoundBufferPlay() will be called.  The client can then take action
// to start the next buffer playing.
//
// This function is called by the interrupt system and should not be
// called directly from application code.
//
//*****************************************************************************
void
SoundIntHandler(void)
{
    unsigned long ulStatus;
    unsigned long *pulTemp;

    static uint32_t mic_data;
    static unsigned char *play_cursor_ptr = phone_play_buffer;
    
    static uint32_t sample_count;
    static uint32_t mic_presample_delta_average,
                    mic_presample_last;
    
    static uint8_t play_delay_compensate = 0;
    
    uint32_t mic_presample_delta;
    
#ifdef M4_I2S
    //
    // Get the interrupt status and clear any pending interrupts.
    //
    ulStatus = SSIIntStatus(I2S_CHAN0_BASE, 1);
    SSIIntClear(I2S_CHAN0_BASE, ulStatus); 
#else // M4_I2S
    ulStatus = SSIIntStatus(SSI_AIC23_BASE, 1);
    SSIIntClear(SSI_AIC23_BASE, ulStatus); 
#endif // M4_I2S

#define DC_OFFSET_8                 8
    
    // test whether it is record or sth else
    if (g_ulFlags == AUDIO_RECORD)
    {
        // since we're in ssi interrupt, we just store the stupid ssi byte into
        // record buffer and return
        // note: the "record buffer"="play buffer"

        SSIDataPut(SSI_AIC23_BASE, mic_data);
        SSIDataGet(SSI_AIC23_BASE, &mic_data);

        // we only store the msb of mic data
        (*g_pucDataPtr) = ((mic_data >> 8) - 128 + DC_OFFSET_8);
        g_pucDataPtr ++;
        
        //mic_data &= 0xFF00; // convert to 8bit
        //mic_data = mic_data - 0x7FFF
        
        // once we've collect half of the record buffer, we'd store them to sdcard
        if (g_pucDataPtr == (g_pucBuffer + AUDIO_BUFFER_SIZE/2))
        {
            if (mic_store_ing == 1)
            {
                m4_dbgprt("!");
                return ;
            }
            
            mic_data_ptr = g_pucBuffer;
            
            //IntTrigger(FAULT_PENDSV);
            *((uint32_t volatile *)0xE000ED04) = 0x10000000;    // trigger PendSV
        }
        else if (g_pucDataPtr == (g_pucBuffer + AUDIO_BUFFER_SIZE))
        {
            if (mic_store_ing == 1)
            {
                m4_dbgprt("!");
                return ;
            }
            
            mic_data_ptr = (g_pucBuffer + AUDIO_BUFFER_SIZE/2);
            
            //IntTrigger(FAULT_PENDSV);
            *((uint32_t volatile *)0xE000ED04) = 0x10000000;
            
            // make a round trip
            g_pucDataPtr = g_pucBuffer;
        }
    }
    else if (g_ulFlags == AUDIO_PHONE)
    {
//#define HALF_DUPLEX_TEST    // require pb3 (receiver) + pb4 (sender)
        
#define ALPHA_PRESAMPLE         0.02        // toggle to make sound transmssion continuous
#define THRESHOLD_DELTA         (1 << 12)   // toggle sensitivity of microphone
        
#define NO_CONVERSION               // disable conversion to signed wave
#define DC_OFFSET_16        0x7FDE  // old:0x73DE
        
#define PRESAMPLE_INTERVAL  0x3F    // must end with 1111..
        
        SSIDataPut(SSI_AIC23_BASE, mic_data);   // we'd put data from cc1101
        SSIDataGet(SSI_AIC23_BASE, &mic_data);  // microphone data

#ifdef HALF_DUPLEX_TEST
        if (dy_pb_version == 3)
            phone_mode = PHONE_PLAYING;
#endif // HALF_DUPLEX_TEST
        
        if (phone_mode == PHONE_PLAYING)
        {
            // simply update mic data from playing buffer:
            if (play_cursor_ptr != play_data_ptr)
            {
                play_cursor_ptr ++;
#ifdef NO_CONVERSION
                mic_data = ((*play_cursor_ptr) << 8);
#else // NO_CONVERSION
                mic_data = ((*play_cursor_ptr) + 128 - DC_OFFSET_8) << 8;
#endif // NO_CONVERSION
                
                if (play_cursor_ptr == (phone_play_buffer + PHONE_BUFFER_SIZE))
                {
                    play_cursor_ptr = phone_play_buffer;
                }
                play_delay_compensate = 0;
            }
            else
            {
                if (play_delay_compensate == 0xFF)
                {
                    // reach the tail of play buffer:
                    phone_mode = PHONE_IDLE;
                }
                play_delay_compensate ++;
            }
        }
        else // we need presample when idle/recording
        {
            sample_count ++;
            
            // presample every 128 samples (64Hz will be filtered) to decrease cpu usage:
            if ((sample_count & PRESAMPLE_INTERVAL) == PRESAMPLE_INTERVAL)
            {
                // we just low-pass filter the presample deltas
                mic_presample_delta = ABS((int32_t)(mic_data - mic_presample_last));
                mic_presample_delta_average = ALPHA_PRESAMPLE*mic_presample_delta + (1 - ALPHA_PRESAMPLE)*mic_presample_delta_average;
                
                mic_presample_last = mic_data;
                
                // if microphone volume > threshold, then recording mode; else playing mode:
                if (mic_presample_delta_average > THRESHOLD_DELTA)
                {
                    phone_mode = PHONE_RECORDING;
                }
                else
                {
                    phone_mode = PHONE_IDLE;
                }
            }
            
#ifdef HALF_DUPLEX_TEST
            if (dy_pb_version == 4)
                phone_mode = PHONE_RECORDING;
#endif // HALF_DUPLEX_TEST
            
            if (phone_mode == PHONE_RECORDING)
            {
            /*
                 phone mic buffer: 2040 Byte
                +----------------------------------------------------------------+
                |xxxxxxxxxxxxxxxxxxxxxxxooooooooooooooooooo                      |
                +----------------------------------------------------------------+
                                        ^                 ^
                                        |                 |
                                 mic_data_cursor     mic_data_ptr
                                    (1101 PTR)        (MIC PTR)

                */
                // store mic data into "big" buffer
                // note: we just transmit 8bit data, and restore 8bit->16bit when playback?
#ifdef NO_CONVERSION
                (*mic_data_ptr) = (mic_data >> 8);
#else // NO_CONVERSION
                (*mic_data_ptr) = ((mic_data >> 8) - 128 + DC_OFFSET_8);
#endif // NO_CONVERSION
                mic_data_ptr ++;
                
                // notify pendsv for cc1101 transmission every 60B:
                if (((mic_data_ptr - phone_mic_buffer) % CC_NOTIFY_WINDOW) == 0)
                {
                    // reach the end of buffer, rewind:
                    if (mic_data_ptr == (phone_mic_buffer + PHONE_BUFFER_SIZE))
                    {
                        mic_data_ptr = phone_mic_buffer;
                    }
                    
                    // if pendsv is still servicing, this won't harm it
                    *((uint32_t volatile *)0xE000ED04) = 0x10000000;
                }
            }
        }
    }
    else // dma hi-quality player
    {
        //
        // Handle the TX channel interrupt
        //
        if(HWREGBITW(&g_ulDMAFlags, FLAG_TX_PENDING))
        {
#ifdef M4_I2S
            //
            // If the TX DMA is done, then call the callback if present.
            //
            if(ROM_uDMAChannelModeGet(I2S_CHAN0_UDMA_TX | UDMA_PRI_SELECT) ==
               UDMA_MODE_STOP)
#else // M4_I2S
            if(ROM_uDMAChannelModeGet(SSI_UDMA_TX | UDMA_PRI_SELECT) ==
                UDMA_MODE_STOP)
#endif // M4_I2S
            {
                //
                // Save a temp pointer so that the current pointer can be set to
                // zero before calling the callback.
                //
                pulTemp = g_sOutBuffers[0].pulData;

                //
                // If at the mid point then refill the first half of the buffer.
                //
                if((g_sOutBuffers[0].pfnBufferCallback) &&
                   (g_sOutBuffers[0].pulData != 0))
                {
                    g_sOutBuffers[0].pulData = 0;
                    g_sOutBuffers[0].pfnBufferCallback(pulTemp, BUFFER_EVENT_FREE);
                }
            }

#ifdef M4_I2S
            //
            // If the TX DMA is done, then call the callback if present.
            //
            if(ROM_uDMAChannelModeGet(I2S_CHAN0_UDMA_TX | UDMA_ALT_SELECT) ==
               UDMA_MODE_STOP)
#else // M4_I2S
            if(ROM_uDMAChannelModeGet(SSI_UDMA_TX | UDMA_ALT_SELECT) ==
                UDMA_MODE_STOP)
#endif // M4_I2S
            {
                //
                // Save a temporary pointer so that the current pointer can be set
                // to zero before calling the callback.
                //
                pulTemp = g_sOutBuffers[1].pulData;

                //
                // If at the mid point then refill the first half of the buffer.
                //
                if((g_sOutBuffers[1].pfnBufferCallback) &&
                   (g_sOutBuffers[1].pulData != 0))
                {
                    g_sOutBuffers[1].pulData = 0;
                    g_sOutBuffers[1].pfnBufferCallback(pulTemp, BUFFER_EVENT_FREE);
                }
            }

            //
            // If no more buffers are pending then clear the flag.
            //
            if((g_sOutBuffers[0].pulData == 0) && (g_sOutBuffers[1].pulData == 0))
            {
                HWREGBITW(&g_ulDMAFlags, FLAG_TX_PENDING) = 0;
            }
        }
    }
}

//*****************************************************************************
//
// Configures the SSI peripheral to play audio in a given format.
//
// ulSampleRate is the sample rate of the audio to be played in
// samples per second.
// usBitsPerSample is the number of bits in each audio sample.
// usChannels is the number of audio channels, 1 for mono, 2 for
// stereo. 
//
// This function configures the SSI peripheral in preparation for playing
// or recording audio data in a particular format.
//
//*****************************************************************************
void
SoundSetFormat(unsigned long ulSampleRate, unsigned short usBitsPerSample,
               unsigned short usChannels)
{
    unsigned long ulDMASetting;

    //
    // Save these values for use when configuring SSI.
    //
    g_ulSampleRate = ulSampleRate;
    g_usChannels = usChannels;
    g_usBitsPerSample = usBitsPerSample;

    //
    // Configure for Mono or Stereo, 16 bit formats.
    // Determine the MCLK rate & DAC PLL for each format.
    //

    //
    // Mono
    //
    //if(g_usChannels == 1)
    if (false)
    {
        //
        // 16-bit format
        //
        if( g_usBitsPerSample == 16 )
        {
#ifdef M4_I2S
            //
            // Set the TX FIFO Size to 16 bits.
            //
            SSIDisable(I2S_CHAN0_BASE);
            SSIConfigSetExpClk(I2S_CHAN0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_2,
                               SSI_MODE_SLAVE, 1000000, 16);
            SSIEnable(I2S_CHAN0_BASE);
#else // M4_I2S
			SSIDisable(SSI_AIC23_BASE);
			SSIConfigSetExpClk(SSI_AIC23_BASE, SysCtlClockGet(), SSI_AIC23_MODE,
				SSI_MODE_SLAVE, SSI1_RATE, 16);
			SSIEnable(SSI_AIC23_BASE);
#endif // M4_I2S
        }
        else
        {
        	//
            // Error - Bits per sample not supported.
            //
        }
    }
    //
    // Stereo
    //
    else
    {
        //
        // 16-bit format
        //
        //if( g_usBitsPerSample == 16 )
        {
#ifdef M4_I2S
            //
            // Set the TX FIFO Size to 16 bits for both SSI channels.
            //
            SSIDisable(I2S_CHAN0_BASE);
            SSIDisable(I2S_CHAN1_BASE);
            SSIConfigSetExpClk(I2S_CHAN0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_2,
                               SSI_MODE_SLAVE, 1000000, 16);
            SSIConfigSetExpClk(I2S_CHAN1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_2,
                               SSI_MODE_SLAVE, 1000000, 16);  
            SSIEnable(I2S_CHAN0_BASE);
            SSIEnable(I2S_CHAN1_BASE);
#else // M4_I2S
			SSIDisable(SSI_AIC23_BASE);
			SSIConfigSetExpClk(SSI_AIC23_BASE, SysCtlClockGet(), SSI_AIC23_MODE,
				SSI_MODE_SLAVE, SSI1_RATE, 16);
			SSIEnable(SSI_AIC23_BASE);
#endif // M4_I2S
        }
        //else
        //{
        	//
            // Error - Bits per sample not supported.
            //
        //}
    }

    //
    // Configure the DAC to sample the audio (set SSI clk) at the frequency
    // at which the file was sampled. This data was found in the .wav header.
    // Check based on most common sampling frequencies:
    // 8k, 11.025k, 16k, 32k, 48k, 64k
    //

    //
    // 8kHz
    //
    if ((ulSampleRate/1000) == 8)
    {
#ifdef M4_AIC3107
        //
        // Change Fsref of 44.1kHz.
        // K = 7.2253, J=7, D=2253
        //

        // 
        // PLL Programming Register B
        // PLL J Value = 7
        //
        DACWriteRegister(4, 0x1C);

        //
        // PLL Programming Register C
        // Both Register, 5&6, must be written to when D changes.
        // Eight most significant bits of the 14-bit integer D.
        // PLL D Value = 2253
        //
        DACWriteRegister(5, 0x23);

        //
        // PLL Programming Register D
        // Both Register, 5&6, must be written to when D changes.        
        // Six least significant bits of the 14-bit integer D.
        // PLL D Value = 2253
        //
        DACWriteRegister(6, 0x34);

        //
        // Codec Datapath Steup Register
        // Disable DAC dual rate mode.
        //
        DACWriteRegister(7, 0x8A);

        //
        // Codec Sample Rate Select Register
        // DAC Fs = Fsref/5.5
        //
        DACWriteRegister(2, 0x09);
#else // M4_AIC3107
        if (g_ulFlags == AUDIO_RECORD)  // recording:
        {
            TLV320AIC23BWriteRegister(TI_SRC, TI_SRC_CLKIN_DIV2 | TI_SRC_SR_8K_8K);
        }
        else
        {
            //TLV320AIC23BWriteRegister(TI_SRC, TI_SRC_SR_48000);
            TLV320AIC23BWriteRegister(TI_SRC, TI_SRC_SR_8K_8K);
        }
#endif // M4_AIC3107

        //
        // Set the audio interrupt rate.
        //
        g_ulAudioUpdateRate = 400;
    }
    //
    // 11kHz
    //
    else if ( ulSampleRate == 11025 )
    {
#ifdef M4_AIC3107
        //
        // Change Fsref of 44.1kHz. 
        // K = 7.2253, J=7, D=2253
        //

        //
        // PLL Programming Register B
        // PLL J Value = 7
        //        
        DACWriteRegister(4, 0x1C);

        //
        // PLL Programming Register C
        // Both Register, 5&6, must be written to when D changes.
        // Eight most significant bits of the 14-bit integer D.
        // PLL D Value = 2253
        //
        DACWriteRegister(5, 0x23);

        //
        // PLL Programming Register D
        // Both Register, 5&6, must be written to when D changes.        
        // Six least significant bits of the 14-bit integer D.
        // PLL D Value = 2253
        //
        DACWriteRegister(6, 0x34);

        //
        // Codec Datapath Steup Register
        // Disable DAC dual rate mode.
        //
        DACWriteRegister(7, 0x8A);

        //
        // Codec Sample Rate Select Register
        // DAC Fs = Fsref/4
        //
        DACWriteRegister(2, 0x06);
#else // M4_AIC3107
		TLV320AIC23BWriteRegister(TI_SRC, TI_SRC_SR_48000);
#endif // M4_AIC3107

        //
        // Set the audio interrupt rate.
        //
        g_ulAudioUpdateRate = 35;
    }
    //
    // 16kHz
    //
    else if( ulSampleRate == 16000 )
    {
#ifdef M4_AIC3107
        //
        // Change Fsref of 48kHz. 
        // K = 7.8643, J=7, D=8643
        //

        // 
        // PLL Programming Register B
        // PLL J Value = 7
        //
        DACWriteRegister(4, 0x1C);

        //
        // PLL Programming Register C
        // Both Register, 5&6, must be written to when D changes.
        // Eight most significant bits of the 14-bit integer D.
        // PLL D Value = 8643
        //
        DACWriteRegister(5,0x87);

        //
        // PLL Programming Register D
        // Both Register, 5&6, must be written to when D changes.        
        // Six least significant bits of the 14-bit integer D.
        // PLL D Value = 8643
        //
        DACWriteRegister(6,0x0C);

        //
        // Codec Datapath Steup Register
        // Enable DAC dual rate mode.
        //
        DACWriteRegister(7, 0xAA);

        //
        // Codec Sample Rate Select Register
        // DAC Fs = Fsref/6.
        //
        DACWriteRegister(2, 0x0A);
#else // M4_AIC3107
		TLV320AIC23BWriteRegister(TI_SRC, TI_SRC_SR_48000);
#endif // M4_AIC3107

        //
        // Set the audio interrupt rate.
        //
        g_ulAudioUpdateRate = 50;
    }
    //
    // 32kHz
    //
    else if( ulSampleRate == 32000 )
    {
#ifdef M4_AIC3107
        //
        // Change Fsref of 48kHz. 
        // K = 7.8643, J=7, D=8643
        //

        // 
        // PLL Programming Register B
        // PLL J Value = 7
        //
        DACWriteRegister(4, 0x1C);

        //
        // PLL Programming Register C
        // Both Register, 5&6, must be written to when D changes.
        // Eight most significant bits of the 14-bit integer D.
        // PLL D Value = 8643
        //
        DACWriteRegister(5,0x87);

        //
        // PLL Programming Register D
        // Both Register, 5&6, must be written to when D changes.
        // Six least significant bits of the 14-bit integer D.
        // PLL D Value = 8643
        //
        DACWriteRegister(6,0x0C);

        //
        // Codec Datapath Steup Register
        // Enable DAC dual rate mode.
        //
        DACWriteRegister(7, 0xAA);

        //
        // Codec Sample Rate Select Register
        // DAC Fs = Fsref/3
        //
        DACWriteRegister(2, 0x04);
#else // M4_AIC3107
		TLV320AIC23BWriteRegister(TI_SRC, TI_SRC_SR_48000);
#endif // M4_AIC3107

        //
        // Set the audio interrupt rate.
        //
        g_ulAudioUpdateRate = 90;
    }
    //
    // 48kHz
    //
    else if( ulSampleRate == 48000 )
    {
#ifdef M4_AIC3107
        //
        // Change Fsref of 48kHz. 
        // K = 7.8643, J=7, D=8643
        //

        // 
        // PLL Programming Register B
        // PLL J Value = 7
        //
        DACWriteRegister(4, 0x1C);

        //
        // PLL Programming Register C
        // Both Register, 5&6, must be written to when D changes.
        // Eight most significant bits of the 14-bit integer D.
        // PLL D Value = 8643
        //
        DACWriteRegister(5,0x87);

        //
        // PLL Programming Register D
        // Both Register, 5&6, must be written to when D changes.
        // Six least significant bits of the 14-bit integer D.
        // PLL D Value = 8643
        //
        DACWriteRegister(6,0x0C);

        //
        // Codec Datapath Steup Register
        // Enable DAC dual rate mode.
        //
        DACWriteRegister(7, 0xAA);

        //
        // Codec Sample Rate Select Register
        // DAC Fs = Fsref/2
        //
        DACWriteRegister(2, 0x02);
#else // M4_AIC3107
		TLV320AIC23BWriteRegister(TI_SRC, TI_SRC_SR_48000);

		//TLV320AIC23BWriteRegister(TI_SRC, TI_SRC_SR_48K_8K);
#endif // M4_AIC3107

        //
        // Set the audio interrupt rate.
        //
        g_ulAudioUpdateRate = (48000*16*2/8)/(AUDIO_BUFFER_SIZE/12);
    }
    //
    // 64kHz
    //
    else if( ulSampleRate == 64000 )
    {
#ifdef M4_AIC3107
        //
        // Change Fsref of 48kHz. 
        // K = 7.8643, J=7, D=8643
        //

        // 
        // PLL Programming Register B
        // PLL J Value = 7
        //
        DACWriteRegister(4, 0x1C);

        //
        // PLL Programming Register C
        // Both Register, 5&6, must be written to when D changes.
        // Eight most significant bits of the 14-bit integer D.
        // PLL D Value = 8643
        //
        DACWriteRegister(5,0x87);

        //
        // PLL Programming Register D
        // Both Register, 5&6, must be written to when D changes.
        // Six least significant bits of the 14-bit integer D.
        // PLL D Value = 8643
        //
        DACWriteRegister(6,0x0C);

        //
        // Codec Datapath Steup Register
        // Enable DAC dual rate mode.
        //
        DACWriteRegister(7, 0xAA);

        //
        // Codec Sample Rate Select Register
        // DAC Fs = Fsref/1.5
        //
        DACWriteRegister(2, 0x01);
#else // M4_AIC3107
		TLV320AIC23BWriteRegister(TI_SRC, TI_SRC_SR_48000);
#endif // M4_AIC3107

        //
        // Set the audio interrupt rate.
        //
        g_ulAudioUpdateRate = 400;
    }
    else
    {
        //
        // Error - Unsupported sampling rate.
        //
    }

#ifdef M4_I2S
    //
    // Configure SSI channel 0 TX DMA channel to use high priority 
    // burst transfer.
    //
    ROM_uDMAChannelAttributeEnable(I2S_CHAN0_UDMA_TX,
                                   (UDMA_ATTR_USEBURST |
                                    UDMA_ATTR_HIGH_PRIORITY));

    //
    // Setup DMA for second SSI if using stereo mode.
    //
    if(g_usChannels == 2)
    {
        //
        // Configure SSI channel 1 TX DMA channel to use high priority 
        // burst transfer.
        //
        uDMAChannelAssign(I2S_CHAN1_UDMA_TX);
        ROM_uDMAChannelAttributeEnable(I2S_CHAN1_UDMA_TX,
                                       (UDMA_ATTR_USEBURST |
                                        UDMA_ATTR_HIGH_PRIORITY));
    }
#else // M4_I2S
    uDMAChannelAssign(SSI_UDMA_TX);
    ROM_uDMAChannelAttributeEnable(SSI_UDMA_TX,
                                   (UDMA_ATTR_USEBURST |
                                    UDMA_ATTR_HIGH_PRIORITY));
#endif // M4_I2S

    //
    // Set the DMA channel configuration.
    //

    //
    // Handle Mono formats.
    //     
    //if(g_usChannels == 1)
    if (false)
    {
        if(g_usBitsPerSample == 16)
        {
            //
            // The transfer size is 16 bits from the TX buffer to the TX FIFO.
            //
            ulDMASetting = UDMA_SIZE_16 | UDMA_SRC_INC_16 |
                           UDMA_DST_INC_NONE | UDMA_ARB_2;
        }
        else
        {
        	//
        	// Error - Bits per sample not supported.
        	//
        }
    }
    //
    // Handle Stereo formats.
    //
    else
    {
        //if(g_usBitsPerSample == 16)
        {
            //
            // The transfer size is 16bits from a source of 32 bits 
            // (stereo 16-bit) to the dual TX FIFOs.
            //
            ulDMASetting = UDMA_SIZE_16 | UDMA_SRC_INC_16 |
                           UDMA_DST_INC_NONE | UDMA_ARB_2;
        }
        //else
        //{
        	//
        	// Error - Bits per sample not supported.
        	//
        //}
    }


#ifdef M4_I2S
    //
    // Configure the DMA settings for these channels.
    //
    ROM_uDMAChannelControlSet(I2S_CHAN0_UDMA_TX | UDMA_PRI_SELECT,
                              ulDMASetting);
    ROM_uDMAChannelControlSet(I2S_CHAN0_UDMA_TX | UDMA_ALT_SELECT,
                              ulDMASetting);

    //
    // Configure the DMA settings for the second SSI if stereo.
    //
    if(g_usChannels == 2)
    {
        ROM_uDMAChannelControlSet(I2S_CHAN1_UDMA_TX | UDMA_PRI_SELECT,
                                  ulDMASetting);
        ROM_uDMAChannelControlSet(I2S_CHAN1_UDMA_TX | UDMA_ALT_SELECT,
                                  ulDMASetting);
    }
#else // M4_I2S
    ROM_uDMAChannelControlSet(SSI_UDMA_TX | UDMA_PRI_SELECT,
                              ulDMASetting);
    ROM_uDMAChannelControlSet(SSI_UDMA_TX | UDMA_ALT_SELECT,
                              ulDMASetting);
#endif // M4_I2S
}


//*****************************************************************************
//
//! Returns the current audio sample rate setting.
//!
//! This function returns the sample rate that is currently set.  The value
//! returned reflects the actual rate set which may be slightly different from
//! the value passed on the last call to SoundSetFormat() if the requested
//! rate could not be matched exactly.
//!
//! \return Returns the sample rate in samples per second.
//
//*****************************************************************************
unsigned long
SoundSampleRateGet(void)
{
    return(g_ulSampleRate);
}

//*****************************************************************************
//
// Starts playback of a block of PCM audio samples.
//
// pvData is a pointer to the audio data to play.
// ulLength is the length of the data in bytes.
// pfnCallback is a function to call when this buffer has been played.
//
// This function starts the playback of a block of PCM audio samples.  If
// playback of another buffer is currently ongoing, its playback is canceled
// and the buffer starts playing immediately.
//
//*****************************************************************************
void
SoundBufferPlay(const void *pvData, unsigned long ulLength,
                void (*pfnCallback)(void *pvBuffer, unsigned long ulEvent))
{
    unsigned long ulChannel;
#ifdef M4_I2S
    unsigned long ulChannel2;

    //
    // Must disable SSI interrupts during this time to prevent state problems.
    //
    ROM_IntDisable(I2S_CHAN0_INT);
    ROM_IntDisable(I2S_CHAN1_INT);
#else // M4_I2S
	ROM_IntDisable(SSI_AIC23_INT);
#endif // M4_I2S

    //
    // Save the buffer information.
    //
    g_sOutBuffers[g_ulPlaying].pulData = (unsigned long *)pvData;
    g_sOutBuffers[g_ulPlaying].ulSize = ulLength;
    g_sOutBuffers[g_ulPlaying].pfnBufferCallback = pfnCallback;

    //
    // Handle which half of the ping-pong DMA is in use.
    //
    if(g_ulPlaying)
    {
#ifdef M4_I2S
        ulChannel = I2S_CHAN0_UDMA_TX | UDMA_ALT_SELECT;
        ulChannel2 = I2S_CHAN1_UDMA_TX | UDMA_ALT_SELECT;
#else // M4_I2S
		ulChannel = SSI_UDMA_TX | UDMA_ALT_SELECT;
#endif // M4_I2S
    }
    else
    {
#ifdef M4_I2S
        ulChannel = I2S_CHAN0_UDMA_TX | UDMA_PRI_SELECT;
        ulChannel2 = I2S_CHAN1_UDMA_TX | UDMA_PRI_SELECT;
#else // M4_I2S
		ulChannel = SSI_UDMA_TX | UDMA_PRI_SELECT;
#endif // M4_I2S
    }

    //
    // Set the DMA channel configuration.
    //
    //if(g_usChannels == 1)
    if (false)
    {
        //
        // Handle Mono formats.
        //
        if(g_usBitsPerSample == 16)
        {
            //
            // The transfer size is 16 bits from the TX buffer to the TX FIFO.
            // Modify the DMA transfer size at it is units not bytes.
            //
            g_sOutBuffers[g_ulPlaying].ulSize >>= 1;
        }
    }
    else
    {
        //
        // Handle Stereo formats.
        //
        //if(g_usBitsPerSample == 16)
        {
            //
            // The transfer size is 16 bits from the TX buffer to the TX FIFO.
            // Modify the DMA transfer size at it is units not bytes.
            //
            g_sOutBuffers[g_ulPlaying].ulSize >>= 1;
        }
    }

#ifdef M4_I2S
    //
    // Set up the uDMA transfer addresses, using ping-pong mode to SSI channel
    // 0. Select the first 16 bits of the 32 bit data.
    //
    ROM_uDMAChannelTransferSet(ulChannel,
                          UDMA_MODE_PINGPONG,
                          (unsigned short *)g_sOutBuffers[g_ulPlaying].pulData,
                          (void *)(I2S_CHAN0_BASE | SSI_O_DR),
                          g_sOutBuffers[g_ulPlaying].ulSize);
    //
    // If using stereo:
    // Set up the uDMA transfer addresses, using ping-pong mode to SSI channel
    // 1. Select the last 16 bits of the 32 bit data.
    //
    if(g_usChannels == 2)
    {
        ROM_uDMAChannelTransferSet(ulChannel2,
                        UDMA_MODE_PINGPONG,
                        (unsigned short *)g_sOutBuffers[g_ulPlaying].pulData+1,
                        (void *)(I2S_CHAN1_BASE | SSI_O_DR),
                        g_sOutBuffers[g_ulPlaying].ulSize );
                        ROM_uDMAChannelEnable(I2S_CHAN0_UDMA_TX);
    }

    //
    // Enable the TX channel.  At this point the uDMA controller will
    // start servicing the request from the SSI, and the transmit side
    // should start running.
    //
    if(g_usChannels == 1)
    {
        ROM_uDMAChannelEnable(I2S_CHAN0_UDMA_TX);
    }
    else
    {
        ROM_uDMAChannelEnable(I2S_CHAN1_UDMA_TX);
    }
#else // M4_I2S
    ROM_uDMAChannelTransferSet(ulChannel,
                          UDMA_MODE_PINGPONG,
                          (unsigned short *)g_sOutBuffers[g_ulPlaying].pulData,
                          (void *)(SSI_AIC23_BASE | SSI_O_DR),
                          g_sOutBuffers[g_ulPlaying].ulSize);
	ROM_uDMAChannelEnable(SSI_UDMA_TX);
#endif // M4_I2S

    //
    // Indicate that there is still a pending transfer.
    //
    HWREGBITW(&g_ulDMAFlags, FLAG_TX_PENDING) = 1;

    //
    // Toggle which ping-pong DMA setting is in use.
    //
    g_ulPlaying ^= 1; 

    //
    // Do not re-enable interrupts. The caller, SoundPlayContinue, will
    // handle this.
    // 
}

//*****************************************************************************
//
//! Sets the audio volume to a given level.
//!
//! \param ulPercent is the volume level to set.  Valid values are between
//! 0 and 100 inclusive.
//!
//! This function sets the audio output volume.  The supplied level is
//! expressed as a percentage between 0% (silence) and 100% (full volume)
//! inclusive, of 10% step sizes.
//!
//! \return None.
//
//*****************************************************************************
void
SoundVolumeSet(unsigned long ulPercent)
{
    //
    // Make sure we were passed a valid volume setting.
    //
    ASSERT(ulPercent <= 100);
	
	g_ucVolume = ulPercent;

#if 0
    //
    // Set the volume to the desired level.
    //
    DACVolumeSet(ulPercent);
#else
    TLV320AIC23BHeadPhoneVolumeSet(ulPercent);
#endif
}

//*****************************************************************************
//
//! Returns the current sound volume.
//!
//! This function returns the current volume, specified as a percentage between
//! 0% (silence) and 100% (full volume), inclusive.
//!
//! \return Returns the current volume setting expressed as a percentage.
//
//*****************************************************************************
unsigned char
SoundVolumeGet(void)
{
    return(g_ucVolume);
}

//*****************************************************************************
//
//! Adjusts the audio volume downwards by a given amount.
//!
//! \param ulPercent is the amount to decrease the volume, specified as a
//! percentage relative to the full volume.
//!
//! This function adjusts the audio output downwards by the specified
//! percentage.  The adjusted volume will not go below 0% (silence).
//!
//! \return None.
//
//*****************************************************************************
void
SoundVolumeDown(unsigned long ulPercent)
{
    //
    // Do not let the volume go below 0%.
    //
    if(g_ucVolume < ulPercent)
    {
        //
        // Set the volume to the minimum.
        //
        g_ucVolume = 0;
    }
    else
    {
        //
        // Decrease the volume by the specified amount.
        //
        g_ucVolume -= ulPercent;
    }
    
    SoundVolumeSet(g_ucVolume);
}

//*****************************************************************************
//
//! Adjusts the audio volume upwards by a given amount.
//!
//! \param ulPercent is the amount to increase the volume, specified as a
//! percentage relative to the full volume.
//!
//! This function adjusts the audio output upwards by the specified percentage.
//! The adjusted volume will not go above 100% (full volume).
//!
//! \return None.
//
//*****************************************************************************
void
SoundVolumeUp(unsigned long ulPercent)
{
    //
    // Increase the volume by the specified amount.
    //
    g_ucVolume += ulPercent;

    //
    // Do not let the volume go above 100%.
    //
    if(g_ucVolume > 100)
    {
        //
        // Set the volume to the maximum.
        //
        g_ucVolume = 100;
    }

    SoundVolumeSet(g_ucVolume);
}


//*****************************************************************************
//
// Enables the class D amplifier in the DAC.
//
// This function enables the class D amplifier in the DAC.  It is merely a
// wrapper around the DAC driver's DACClassDEn() function. SoundClassDDis()
// should be called to disable the Amp.
//
//*****************************************************************************
void
SoundClassDEn(void)
{
	//
    // NOTE: Current data path bypasses the Class-D Speaker Amplifier. To
    // enable this, DAC Register 41 will need to be routed to the
    // LEFT/RIGHT_LOP via DAC_L3 and DAC_R3. SW_L0, etc will need to be
    // adjusted to allow for a path to the Amp.
    // 
    //DACClassDEn();
}

//*****************************************************************************
//
// Disables the class D amplifier in the DAC.
//
// This function disables the class D amplifier in the DAC.  It is merely a
// wrapper around the DAC driver's DACClassDDis() function.
//
//*****************************************************************************
void
SoundClassDDis(void)
{
    //DACClassDDis();
}

//*****************************************************************************
//
//! Begins playback of the opened .wav audio file.
//!
//! This function alters flags indicating playback, and sets up a timer
//! interrupt to ensure the audio continues playing.
//!
//! \return None.
//
//*****************************************************************************
void
SoundPlay()
{
    //
    // Mark both buffers as empty.
    //
    g_ulFlags = BUFFER_BOTTOM_EMPTY | BUFFER_TOP_EMPTY;

    //
    // Indicate that the application is about to start playing.
    //
    g_ulFlags |= BUFFER_PLAYING;

    //
    // Set the required interrupt rate based on the
    // audio sampling frequency. This update rate
    // was set within SoundSetFormat(). The smaller
    // the update rate, less CPU usage.
    //
    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()/g_ulAudioUpdateRate);
     
    //
    // Enable the Timer to continue playback.
    // For higher sampling frequencies, a more periodic interrupt is required.
    //
    IntEnable(INT_TIMER2A);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER2_BASE, TIMER_A);

    IntMasterEnable();
}

//*****************************************************************************
//
//! Change the playback state to stop the playback.
//!
//! This function changes the state of playback to ``stopped'' so that the
//! audio clip will stop playing.  This clears and disables all playback
//! interrupts, sets appropriate flags, and updates the playback information.
//!
//! \return None.
//
//*****************************************************************************
void
SoundStop()
{
    unsigned long ulStatus;

#ifdef M4_I2S
    //
    // Disable the SSI interrupts, and clear their status.
    //
    ROM_IntDisable(I2S_CHAN0_INT);
    ROM_IntDisable(I2S_CHAN1_INT);
    ulStatus = SSIIntStatus(I2S_CHAN0_BASE, 1);
    SSIIntClear(I2S_CHAN0_BASE, ulStatus);
    ulStatus = SSIIntStatus(I2S_CHAN1_BASE, 1);
    SSIIntClear(I2S_CHAN1_BASE, ulStatus);
#else // M4_I2S
	ROM_IntDisable(SSI_AIC23_INT);
	ulStatus = SSIIntStatus(SSI_AIC23_BASE, 1);
	SSIIntClear(SSI_AIC23_BASE, ulStatus);
#endif // M4_I2S

    //
    // Clear the flag that indicates audio is playing.
    //
    g_ulFlags &= ~BUFFER_PLAYING;

    //
    // Disable the Timer.
    //
    IntDisable(INT_TIMER2A);

    //
    // Update playback information.
    //
    g_usSeconds = 0;
    g_usMinutes = 0;
    g_ulBytesPlayed = 0;
    g_Paused = false;
}

//*****************************************************************************
//
//! Change the playback state to pause the playback.
//!
//! This function changes the state of playback to ``paused'' so that the
//! audio clip will halt playback. This function keeps the current state
//! so that another call to this function will un-pause the audio to continue
//! playback.
//!
//! \return None.
//
//*****************************************************************************
void
SoundPause()
{
    IntMasterDisable();
    //
    // Audio is paused. Re-enable playback.
    //
    if(g_Paused)
    {
		// ? bad for gpio pin
        //IntMasterEnable();
        IntEnable(INT_TIMER2A);
        IntEnable(SSI_AIC23_INT);
#ifdef M4_I2S
        ROM_uDMAChannelEnable(I2S_CHAN1_UDMA_TX);
#else // M4_I2S
		ROM_uDMAChannelEnable(SSI_UDMA_TX);
#endif // M4_I2S
        g_Paused = false;
    }
    //
    // Audio is playing. Halt playback.
    //
    else
    {
        // IntMasterDisable
        IntDisable(INT_TIMER2A);
        IntDisable(SSI_AIC23_INT);
#ifdef M4_I2S
        ROM_uDMAChannelDisable(I2S_CHAN1_UDMA_TX);
#else // M4_I2S
		ROM_uDMAChannelDisable(SSI_UDMA_TX);
#endif // M4_I2S
        g_Paused = true;
    }
    IntMasterEnable();
}

//*****************************************************************************
//
// This function is called by the sound driver when a buffer has been
// played.  It marks the buffer (top half or bottom half) as free.
//
//*****************************************************************************
static void
BufferCallback(void *pvBuffer, unsigned long ulEvent)
{
    //
    // Handle buffer-free event
    //
    if(ulEvent & BUFFER_EVENT_FREE)
    {
        //
        // If pointing at the start of the buffer, then this is the first
        // half, so mark it as free.
        //
        if(pvBuffer == g_pucBuffer)
        {
            g_ulFlags |= BUFFER_BOTTOM_EMPTY;
        }

        //
        // Otherwise it must be the second half of the buffer that is free.
        //
        else
        {
            g_ulFlags |= BUFFER_TOP_EMPTY;
        }

        if ((g_usChannels == 1) && (g_usBitsPerSample == 8))
        {
            g_ulBytesPlayed += AUDIO_BUFFER_SIZE >> 3;
        }
        else
        {
            //
            // Update the byte count.
            //
            g_ulBytesPlayed += AUDIO_BUFFER_SIZE >> 1;
        }
    }
}

//*****************************************************************************
//
// Converts unsigned 8 bit data to signed data in place.
//
// pucBuffer is a pointer to the input data buffer.
// ulSize is the size of the input data buffer.
//
// This function converts the contents of  an 8 bit unsigned buffer to 8 bit
// signed data in place so that the buffer can be passed into the I2S
// controller for playback.
//
//*****************************************************************************
static void
WaveConvert8Bit(unsigned char *pucBuffer, unsigned long ulSize)
{
    unsigned long ulIdx;
    unsigned short wave_sample; 

#if 0
    //
    // Loop through the entire array, and convert the 8-bit unsigned data
    // to 8-bit signed.
    //
    for(ulIdx = 0; ulIdx < ulSize; ulIdx++)
    {
        *pucBuffer = ((short)(*pucBuffer)) - 128;
        pucBuffer++;
    }
#else
    for(ulIdx = ulSize; ulIdx > 0; ulIdx--)
    {
        wave_sample = (*(pucBuffer + ulIdx - 1)) - 128;
        
        // wasty:
        *(pucBuffer + ulIdx*4 - 1) = wave_sample;
        *(pucBuffer + ulIdx*4 - 2) = 0;
        *(pucBuffer + ulIdx*4 - 3) = wave_sample;
        *(pucBuffer + ulIdx*4 - 4) = 0;
    }
#endif
}

//*****************************************************************************
//
//! Opens a wav file and parses its header information.
//!
//! \param pcFileName is a pointer to the start of the wav format data in
//! memory.
//! \param pWaveHeader is a pointer to a caller-supplied tWaveHeader structure
//! that will be populated by the function.
//!
//! This function is used to parse a wav header and populate a caller-supplied
//! header structure in preparation for playback.  It can also be used to test
//! if a clip is in wav format or not.
//!
//! \return \b TRUE if the file was parsed successfully,
//! \b FALSE 
//!  if RIFF information is not supported,
//!  if the chunk size is not supported,
//!  if the format is not supported.
//
//*****************************************************************************
bool
SoundOpen(const char *pcFileName, tWaveHeader *pWaveHeader)
{
    unsigned long *pulBuffer;
    unsigned short *pusBuffer;
    unsigned long ulChunkSize;
    unsigned long ulBytesPerSample;
    unsigned int usCount;
    FRESULT Result;

    g_WaveHeader = pWaveHeader;

    //
    // Get a pointer to the RIFF tag.
    //
    pulBuffer = (unsigned long *)g_pucBuffer;
    pusBuffer = (unsigned short *)g_pucBuffer;

    //
    // Open and get file object from SD card
    //
    Result = f_open(&g_sFileObject, pcFileName, FA_READ);
    if(Result != FR_OK)
    {
        return(false);
    }

    //
    // Read the first 12 bytes.
    //
    Result = f_read(&g_sFileObject, g_pucBuffer, 12, &usCount);
    if(Result != FR_OK)
    {
        f_close(&g_sFileObject);
        return(false); 
    }

    //
    // Check for valid RIFF.
    //
    if((pulBuffer[0] != RIFF_CHUNK_ID_RIFF) || (pulBuffer[2] != RIFF_TAG_WAVE))
    {
        f_close(&g_sFileObject);
        return(false);
    }

    //
    // Read the next chunk header.
    //
    Result = f_read(&g_sFileObject, g_pucBuffer, 8, &usCount);
    if(Result != FR_OK)
    {
        f_close(&g_sFileObject);
        return(false); 
    }

    //
    // Check for valid chunk.
    //
    if(pulBuffer[0] != RIFF_CHUNK_ID_FMT)
    {
        f_close(&g_sFileObject);
        return(false); 
    }

    //
    // Read the format chunk size.
    //
    ulChunkSize = pulBuffer[1];

    if(ulChunkSize > 40)
    {
        f_close(&g_sFileObject);
        return(false); 
    }

    //
    // Read the next chunk header.
    //
    Result = f_read(&g_sFileObject, g_pucBuffer, ulChunkSize, &usCount);
    if(Result != FR_OK)
    {
        f_close(&g_sFileObject);
        return(false); 
    }
    
    //
    // Populate the caller-supplied header structure with the wav format
    // information.
    //
    pWaveHeader->usFormat = pusBuffer[0];
    pWaveHeader->usNumChannels =  pusBuffer[1];
    pWaveHeader->ulSampleRate = pulBuffer[1];
    pWaveHeader->ulAvgByteRate = pulBuffer[2];
    pWaveHeader->usBitsPerSample = pusBuffer[7];
    
    //
    // Reset the byte count.
    //
    g_ulBytesPlayed = 0;

    //
    // Calculate the Maximum buffer size based on format.  There can only be
    // 1024 samples per ping-pong buffer due to uDMA.
    //
    ulBytesPerSample = (pWaveHeader->usBitsPerSample *
                        pWaveHeader->usNumChannels) >> 3;

    //
    // Cap the maximum buffer size used to be the smaller of 1024 samples,
    // which is the limit of the uDMA controller, or the size of the
    // audio buffer.
    //
    if(((AUDIO_BUFFER_SIZE >> 1) / ulBytesPerSample) > 1024)
    {
        g_ulMaxBufferSize = 1024 * ulBytesPerSample;
    }
    else
    {
        g_ulMaxBufferSize = AUDIO_BUFFER_SIZE >> 1;
    }

    //
    // Only mono and stereo supported.
    //
    if(pWaveHeader->usNumChannels > 2)
    {
        f_close(&g_sFileObject);
        return(false);
    }
    
    //
    // Read the next chunk header.
    //
    Result = f_read(&g_sFileObject, g_pucBuffer, 8, &usCount);
    if(Result != FR_OK)
    {
        f_close(&g_sFileObject);
        return(false); 
    }

    if(pulBuffer[0] != RIFF_CHUNK_ID_DATA)
    {
        f_close(&g_sFileObject);
        return(false); 
    }

    //
    // Save the size of the data.
    //
    pWaveHeader->ulDataSize = pulBuffer[1];

    //
    // Calculate the duration of the clip
    //
    g_usSeconds = pWaveHeader->ulDataSize / pWaveHeader->ulAvgByteRate;
    g_usMinutes = g_usSeconds / 60;
    g_usSeconds -= g_usMinutes * 60;


    //
    // Set the data pointer to the start of the data
    //
    g_pucDataPtr = (unsigned char *)&pulBuffer[2];

    //
    // Set the number of data bytes in the file.
    //
    g_ulBytesRemaining = pWaveHeader->ulDataSize;

    //
    // Adjust the average bit rate for 8 bit mono files.
    //
    if ((pWaveHeader->usNumChannels == 1) && 
        (pWaveHeader->usBitsPerSample == 8))
    {
        pWaveHeader->ulAvgByteRate <<=1;
    }
    
    // mute adc & line in to reduce noise:
    TLV320AIC23BWriteRegister(TI_POWER_DOWN, TI_POWER_DOWN_ADC | TI_POWER_DOWN_LINE | TI_POWER_DOWN_MIC);
    TLV320AIC23BWriteRegister(TI_ANALOG_AP, TI_ANALOG_AP_DAC | TI_ANALOG_AP_INSEL | TI_ANALOG_AP_MICM);
    TLV320AIC23BWriteRegister(TI_DIGITAL_AP, TI_DIGITAL_AP_DEEMP_48K);

    //
    // Set the format of the playback in the sound driver.
    //
    SoundSetFormat(pWaveHeader->ulSampleRate, pWaveHeader->usBitsPerSample,
                   pWaveHeader->usNumChannels);
    
    //
    // Made it to here ... return success.
    //
    return(true);
}

//*****************************************************************************
//
// Read next block of data from audio clip.
//
// psFileObject is a point to the wave file in memory.
// pWaveHeader is a pointer to wave header data for this audio clip.
// pucBuffer is a pointer to a buffer where the next block of data
// will be copied.
//
// This function reads the next block of data from the playing audio clip
// and stores it in the caller-supplied buffer.  If required the data will
// be converted in-place from 8-bit unsigned to 8-bit signed.
//
// Return the number of bytes read from the audio clip and stored in the
// buffer.
//
//*****************************************************************************
static unsigned long
WaveRead(FIL *psFileObject, tWaveHeader *pWaveHeader, unsigned char *pucBuffer)
{
    unsigned long ulBytesToRead;
    unsigned int usCount;

    //
    // Either read a half buffer or just the bytes remaining if we are at the
    // end of the file.
    //
    if(g_ulBytesRemaining < g_ulMaxBufferSize)
    {
        ulBytesToRead = g_ulBytesRemaining;
    }
    else
    {
        ulBytesToRead = g_ulMaxBufferSize;
    }

    if(pWaveHeader->usBitsPerSample == 8)
    {
        // we need convert 8bit mono -> 16bit stereo, so only read quarter of the length
        ulBytesToRead /= 4;
    }
    //
    // Read in another buffer from the sd card.
    //
    if(f_read(&g_sFileObject, pucBuffer, ulBytesToRead, &usCount) != FR_OK)
    {
        return(0);
    }

    //
    // Decrement the number of data bytes remaining to be read.
    //
    g_ulBytesRemaining -= ulBytesToRead;

    //
    // Need to convert the audio from unsigned to signed if 8 bit
    // audio is used.
    //
    if(pWaveHeader->usBitsPerSample == 8)
    {
        WaveConvert8Bit(pucBuffer, ulBytesToRead);
        ulBytesToRead *= 4;
    }

    //
    // Update the global data pointer keeping track of the location in flash.
    //
    g_pucDataPtr += ulBytesToRead;

    //
    // Return the number of bytes that were read from the audio clip into
    // the buffer.
    //
    return(ulBytesToRead);
}

//*****************************************************************************
//
// Continues playback of a wave file previously passed to WavePlayStart().
//
// pWaveHeader points to the structure containing information on the
// wave file being played.
//
// This function must be called periodically (at least every 40mS) after
// WavePlayStart() to continue playback of an audio wav file.  It does any
// necessary housekeeping to keep the DAC supplied with audio data and returns
// a value indicating when the playback has completed.
//
// Returns true when playback is complete or false if more audio
// data still remains to be played.
//
//*****************************************************************************
bool
SoundPlayContinue(tWaveHeader *pWaveHeader)
{
    bool bRetcode;
    unsigned long ulCount;
    unsigned long ulStatus;

    //
    // Assume we're not finished until we learn otherwise.
    //
    bRetcode = false;

    //
    // Set a value that we can use to tell whether or not we processed any
    // new data.
    //
    ulCount = 0xFFFFFFFF;
#ifdef M4_I2S
    //
    // Must disable SSI interrupts during this time to prevent state problems.
    //
    ROM_IntDisable(I2S_CHAN0_INT);
    ROM_IntDisable(I2S_CHAN1_INT);
#else
	ROM_IntDisable(SSI_AIC23_INT);
#endif // M4_I2S

    //
    // If the refill flag gets cleared then fill the requested side of the
    // buffer.
    //
    if(g_ulFlags & BUFFER_BOTTOM_EMPTY)
    {
        //
        // Read out the next buffer worth of data.
        //
        ulCount = WaveRead(&g_sFileObject, pWaveHeader, g_pucBuffer);

        //
        // Start the playback for a new buffer.
        //
        SoundBufferPlay(&g_pucBuffer, ulCount, BufferCallback);

        //
        // Bottom half of the buffer is now not empty.
        //
        g_ulFlags &= ~BUFFER_BOTTOM_EMPTY;
    }

    if(g_ulFlags & BUFFER_TOP_EMPTY)
    {
        //
        // Read out the next buffer worth of data.
        //
        ulCount = WaveRead(&g_sFileObject, pWaveHeader, 
                           &g_pucBuffer[AUDIO_BUFFER_SIZE >> 1]);

        //
        // Start the playback for a new buffer.
        //
        SoundBufferPlay(&g_pucBuffer[AUDIO_BUFFER_SIZE >> 1],
                        ulCount, BufferCallback);

        //
        // Top half of the buffer is now not empty.
        //
        g_ulFlags &= ~BUFFER_TOP_EMPTY;
    }

    //
    // Audio playback is done once the count is below a full buffer.
    //
    if((ulCount < g_ulMaxBufferSize) || (g_ulBytesRemaining == 0))
    {
        //
        // No longer playing audio.
        //
        g_ulFlags &= ~BUFFER_PLAYING;

        f_close(&g_sFileObject);

#if 0
        //
        // Disable the Class D amp to save power.
        //
        SoundClassDDis();
#endif

#ifdef M4_I2S
        //
        // Disable and clear all interrupts.
        //
        ROM_IntDisable(I2S_CHAN0_INT);
        ROM_IntDisable(I2S_CHAN1_INT);
        ulStatus = SSIIntStatus(I2S_CHAN0_BASE, 1);
        SSIIntClear(I2S_CHAN0_BASE, ulStatus);
        ulStatus = SSIIntStatus(I2S_CHAN1_BASE, 1);
        SSIIntClear(I2S_CHAN1_BASE, ulStatus);
#else // M4_I2S
		ROM_IntDisable(SSI_AIC23_INT);
		ulStatus = SSIIntStatus(SSI_AIC23_BASE, 1);
		SSIIntClear(SSI_AIC23_BASE, ulStatus);
#endif // M4_I2S

        return(true);
    }

#ifdef M4_I2S
    //
    // Reenable the SSI interrupt now that we're finished mucking with
    // state and flags.
    //
    ROM_IntEnable(I2S_CHAN0_INT);
#else // M4_I2S
	ROM_IntEnable(SSI_AIC23_INT);
#endif // M4_I2S
	
#ifdef M4_I2S
    // Reenable the second SSI interrupt if using stereo mode.
    if(g_usChannels == 2)
    {
        ROM_IntEnable(I2S_CHAN1_INT);
		ROM_IntEnable(SSI_INT);
    }
#endif // M4_I2S

    return(bRetcode);
}

//*****************************************************************************
//
//! Formats a text string showing elapsed and total playing time.
//!
//! \param pWaveHeader is a pointer to the current wave file's header
//!  information.  This structure will have been populated by a previous call
//!  to WaveOpen().
//! \param pcTime points to storage for the returned string.
//! \param ulSize is the size of the buffer pointed to by \e pcTime.
//!
//! This function may be called periodically by an application during the time
//! that a wave file is playing.  It formats a text string containing the
//! current playback time and the total length of the audio clip.  The
//! formatted string may be up to 12 bytes in length containing the
//! terminating NULL so, to prevent truncation, \e ulSize must be 12 or larger.
//!
//! \return Returns an integer value of the number of seconds of playback.
//
//*****************************************************************************
unsigned long
SoundGetTime(tWaveHeader *pWaveHeader, char *pcTime, unsigned long ulSize)
{
    unsigned long ulSeconds;
    unsigned long ulMinutes;

    //
    // Calculate the integer number of minutes and seconds.
    //
    ulSeconds = g_ulBytesPlayed / pWaveHeader->ulAvgByteRate;
    ulMinutes = ulSeconds / 60;
    ulSeconds -= ulMinutes * 60;

    // m4_dbgprt("%d,%d\r\n", g_ulBytesPlayed, pWaveHeader->ulAvgByteRate);
    
    if (g_ulFlags == AUDIO_RECORD)
    {
        usnprintf((char *)pcTime, ulSize, "%d:%02d", ulMinutes, ulSeconds);
    }
    else
    {
        //
        // Print the time string in the format mm.ss/mm.ss
        //
        usnprintf((char *)pcTime, ulSize, "%d:%02d/%d:%02d", ulMinutes, ulSeconds, 
                  g_usMinutes, g_usSeconds);
    }
    
    return(ulSeconds + 60 * ulMinutes);
}

//*****************************************************************************
//
//! Returns the current playback status of the wave file.
//!
//! This function may be called to determine whether a wave file is currently
//! playing.
//!
//! \return Returns \b true if a wave file us currently playing or \b false if
//! no file is playing.
//
//*****************************************************************************
bool
SoundPlaybackStatus(void)
{
    return((g_ulFlags & BUFFER_PLAYING) ? true : false);
}

//*****************************************************************************
//
//! Returns the total length of the wave file in seconds.
//!
//! This function may be called to recall the total length of the current wave
//! file.
//!
//! \return Returns an integer in units of seconds.
//
//*****************************************************************************
unsigned long
SoundGetLength()
{
    return(g_usMinutes * 60 + g_usSeconds);
}

void phone_data_arrive()
{
    char pkt_len = CC_NOTIFY_WINDOW;
    
    if (phone_mode != PHONE_RECORDING)
    {
        // store data directly into play buffer:
        cc1101_receive(play_data_ptr, &pkt_len);
        
        play_data_ptr += CC_NOTIFY_WINDOW;
        
        // rewind:
        if (play_data_ptr == (phone_play_buffer + PHONE_BUFFER_SIZE))
        {
            play_data_ptr = phone_play_buffer;
        }
        
        phone_mode = PHONE_PLAYING;
    }
    // else discard this gpio event
}

void phone_init()
{
    g_ulFlags = AUDIO_PHONE;
    g_pucDataPtr = g_pucBuffer;
    
    mic_data_ptr    = phone_mic_buffer;
    play_data_ptr   = phone_play_buffer;
    
    // configure the mic record path:
    TLV320AIC23BWriteRegister(TI_POWER_DOWN, TI_POWER_DOWN_LINE); // power on adc
    
    TLV320AIC23BWriteRegister(TI_ANALOG_AP, TI_ANALOG_AP_DAC | TI_ANALOG_AP_INSEL | TI_ANALOG_AP_MICB);

    SoundSetFormat(8192, 8, 1);
    
    cc1101_int_register(&phone_data_arrive);
    
    // configure pend system service interrupt:
    *((uint32_t volatile *)0xE000ED22) = 0xFF;  // lowest priority
    IntEnable(FAULT_PENDSV);
    
    // configure ssi rx interrupt:
    SSIIntClear(SSI_AIC23_BASE, SSI_RXFF);
    SSIIntEnable(SSI_AIC23_BASE, SSI_RXFF);
    IntEnable(SSI_AIC23_INT);
}

uint8_t phone_status_get()
{
    return phone_mode;
}

// pendsv isr:
void mic_phone_data_handler()
{
    UINT btw, bw;
    FRESULT fr;
    
    static unsigned char *mic_cursor_ptr = phone_mic_buffer + CC_NOTIFY_WINDOW;
    
    mic_store_ing = 1;
    
    *((uint32_t volatile *)0xE000ED04) = 0x00000000;    // clear PendSV
    
    if (g_ulFlags == AUDIO_RECORD)
    {
        btw = AUDIO_BUFFER_SIZE/2;
        
        // no overwhelm
        fr = f_write(&g_sFileObject, mic_data_ptr, btw, &bw);
        
        g_ulBytesPlayed += btw;
        
        m4_dbgprt("tw=%d,w=%d, fr=%d\r\n", btw, bw, fr);
        
        mic_store_ing = 0;
    }
    else if (g_ulFlags == AUDIO_PHONE)
    {
        btw = CC_NOTIFY_WINDOW;

        // do not overrun mic_dat_ptr:
        while (!(((mic_cursor_ptr - CC_NOTIFY_WINDOW) <= mic_data_ptr) && (mic_data_ptr <= mic_cursor_ptr)))
        {
            if (mic_cursor_ptr == (phone_mic_buffer + PHONE_BUFFER_SIZE))
            {
                mic_cursor_ptr = phone_mic_buffer + CC_NOTIFY_WINDOW;
                continue;
            }

            cc1101_send(mic_cursor_ptr, btw);
            
            mic_cursor_ptr += CC_NOTIFY_WINDOW;
        }
    }
}

void mic_sidetone()
{
    TLV320AIC23BWriteRegister(TI_ANALOG_AP, TI_ANALOG_AP_STA | TI_ANALOG_AP_STE);
}

void mic_record_start(const char *file_name, tWaveHeader *pWaveHeader)
{
    BYTE *mic_buffer_ptr;
    FRESULT fr;    

    g_ulFlags = AUDIO_RECORD;
    g_pucDataPtr = g_pucBuffer;
    
    fr = f_open(&g_sFileObject, file_name, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
    
    // skip the wave file header:
    fr = f_lseek(&g_sFileObject, sizeof(wav_Hdr));

    // TODO: merge to init
    // configure the mic record path:
    TLV320AIC23BWriteRegister(TI_POWER_DOWN, TI_POWER_DOWN_LINE); // power on adc
    
    TLV320AIC23BWriteRegister(TI_ANALOG_AP, TI_ANALOG_AP_DAC | TI_ANALOG_AP_INSEL | TI_ANALOG_AP_MICB);

    SoundSetFormat(8192, 8, 1);
    
    g_ulBytesPlayed = 0;
    pWaveHeader->ulAvgByteRate = AUDIO_BUFFER_SIZE*2; // bad crystal clock

    lcd_printf(0, 1, "record start    ");
    
    // configure pend system service interrupt:
    *((uint32_t volatile *)0xE000ED22) = 0xFF;  // lowest priority
    IntEnable(FAULT_PENDSV);
    
    // configure ssi rx interrupt:
    SSIIntClear(SSI_AIC23_BASE, SSI_RXFF);
    SSIIntEnable(SSI_AIC23_BASE, SSI_RXFF);
    IntEnable(SSI_AIC23_INT);
}

void mic_record_stop()
{
    UINT btw, bw;
    volatile FRESULT fr;
    
    wav_Hdr wave_header;

    g_ulFlags = AUDIO_IDLE;
    
    SSIIntClear(SSI_AIC23_BASE, SSI_RXFF);
    SSIIntDisable(SSI_AIC23_BASE, SSI_RXFF);
    IntDisable(SSI_AIC23_INT);
    
    memcpy(wave_header.ChunkID      ,   "RIFF"  , 4);
    memcpy(wave_header.Format       ,   "WAVE"  , 4);
    memcpy(wave_header.Subchunk1ID  ,   "fmt "  , 4);

    // Subchunk1:
    wave_header.Subchunk1Size   = 16;
    wave_header.AudioFormat     = 1;
    wave_header.NumChannels     = 1;
    wave_header.SampleRate      = 8192;
    wave_header.BitsPerSample   = 8;

    // Subchunk2:
    memcpy(wave_header.Subchunk2ID  ,   "data"  , 4);
    
    wave_header.ByteRate        = wave_header.SampleRate*wave_header.NumChannels*wave_header.BitsPerSample/8;
    wave_header.BlockAlign      = wave_header.NumChannels*wave_header.BitsPerSample/8;

    wave_header.Subchunk2Size   = g_ulBytesPlayed; // NumSamples*NumChannels*BitsPerSample/8;
    
    wave_header.ChunkSize       = 4 + (8 + wave_header.Subchunk1Size) + (8 + wave_header.Subchunk2Size);

    // write the wave file header
    f_lseek(&g_sFileObject, 0); 
    f_write(&g_sFileObject, &wave_header, sizeof(wav_Hdr), &bw);
    fr = f_close(&g_sFileObject);

    lcd_printf(0, 1, "record stop     ");
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
