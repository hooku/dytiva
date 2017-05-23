#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#include "sound.h"
#include "m4_common.h"
#include "m4_app_res.h"

typedef enum __Player_State_
{
    S_FIRST,
    S_STOP = S_FIRST,
    S_PLAY_ING,
    S_PLAY_PAUSE,
    S_RECORD_ING,
    S_LAST,
} Player_State;

typedef enum __Player_Button_
{
    BT_FIRST,
    BT_NONE = BT_FIRST,
    BT_SELECT,
    BT_CANCEL,
    BT_LEFT,
    BT_RIGHT,
    BT_UP,
    BT_DOWN,
    BT_LAST
} Player_Button;

typedef enum __LCD_STATUS_REFRESH_
{
    REF_FIRST,
    REF_NONE = BT_FIRST,
    REF_STOP,
    REF_PAUSE,
    REF_VOLUME,
    REF_LAST,
} LCD_Status_Refresh;

Player_State g_sSoundState = S_STOP;
Player_Button g_button_press = BT_NONE;
//Player_Button g_button_press = BT_SELECT;

long g_DACVolume = 20;
long g_ulWaveIndex = 0;

static unsigned long g_ulWavCount;

tWaveHeader g_sWaveHeader;

uint8_t lcd_status_refresh = 0;
LCD_Status_Refresh lcd_status_refresh_item = REF_NONE;

#ifdef M4_USBMSC
void usb_msc_loop()
{
    lcd_clear();
    lcd_printf(0xFF, 0, "USB Card Reader");
        
    while (1)
    {
        switch (usb_msc_stat())
        {
            case 0: // MSC_DEV_DISCONNECTED
                lcd_printf(0, 1, "RESET");
                break;
            case 1: // MSC_DEV_CONNECTED
                // since vbus is not connected, this would never happen
                break;
            case 2: // MSC_DEV_IDLE
                lcd_printf(0, 1, "IDLE ");
                //gpio_led_all(0);
                break;
            case 3: // MSC_DEV_READ
                lcd_printf(0, 1, "READ ");
                //gpio_led_all(1);
                break;
            case 4: // MSC_DEV_WRITE
                lcd_printf(0, 1, "WRITE");
                //gpio_led_all(1);
                break;
        }
    }
}
#endif // M4_USBMSC

void gpio_button_down_player(Button button)
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
            // PLAY / PAUSE
            g_button_press = BT_SELECT;
            break;
        case Key_2:
            // RECORD
            g_button_press = BT_CANCEL;
            //mic_sidetone();
            break;
        case Key_3:
            // PREVIOUS
            g_button_press = BT_LEFT;
            break;
        case Key_4:
            // NEXT
            g_button_press = BT_RIGHT;
            break;
    }
}

//*****************************************************************************
//
// This function is called to read the contents of the root directory on
// a given FAT logical drive and fill the listbox containing the names of all
// audio WAV files found.
//
//*****************************************************************************
static int
FindWaveFilesOnDrive(const char *pcDrive, int iStartIndex)
{
    FRESULT fresult;
    int iCount;

    //
    // Open the current directory for access.
    //
    fresult = f_opendir(&g_sDirObject, pcDrive);

    //
    // Check for error and return if there is a problem.
    //
    if(fresult != FR_OK)
    {
        //
        // Ensure that the error is reported.
        //
        return(0);
    }

    //
    // Start by inserting at the next entry in the list box.
    //
    iCount = iStartIndex;

    //
    // Enter loop to enumerate through all directory entries.
    //
    while(1)
    {
        //
        // Read an entry from the directory.
        //
        fresult = f_readdir(&g_sDirObject, &g_sFileInfo);

        //
        // Check for error and return if there is a problem.
        //
        if(fresult != FR_OK)
        {
            lcd_printf(0, 2, "!!SD CARD ERR!! ");
            while (1)
                ;
            
            return(0);
        }

        //
        // If the file name is blank, then this is the end of the
        // listing.
        //
        if(!g_sFileInfo.fname[0])
        {
            break;
        }

        //
        // Add the information as a line in the listbox widget if there is
        // space left and the filename contains ".wav".
        //
        if((g_ulWavCount < NUM_LIST_STRINGS) &&
           ((ustrstr(g_sFileInfo.fname, ".wav")) ||
            (ustrstr(g_sFileInfo.fname, ".WAV"))))
        {
            //
            // Ignore directories.
            //
            if((g_sFileInfo.fattrib & AM_DIR) == 0)
            {
                usnprintf(g_pcFilenames[iCount], MAX_FILENAME_STRING_LEN,
                          "%s", g_sFileInfo.fname);
                // If folder name is needed.
                //usnprintf(g_pcFilenames[iCount], MAX_FILENAME_STRING_LEN,
                //          "%s%s", pcDrive, g_sFileInfo.fname);

                //
                // Move on to the next entry in the list box.
                //
                iCount++;
            }
        }
    }

    //
    // Made it to here, return the number of files we found.
    //
    return(iCount);
}

//*****************************************************************************
//
// This function is called to read the contents of the current directory on
// the SD card and fill the listbox containing the names of all files.
//
//*****************************************************************************
static int
PopulateFileList()
{
    //
    // How many files can we find on the SD card (if present)?
    //
    g_ulWavCount = (unsigned long)FindWaveFilesOnDrive("0:/", 0);

    //
    // Did we find any files at all?
    //
    return(g_ulWavCount ? 0 : FR_NO_FILE);
}

// LCD Display:
//  ________________
// |MP3 Player    CD|
// |48_16_stereo.wav|
// |02:25/04:23     |
// |                |
// |________________|
//

void update_song_title()
{
    unsigned char *pic_quality;
    
    // draw sample rate icon:
    switch (SoundSampleRateGet()/1000)
    {
        case 48:
            pic_quality = pic_48k;
            break;
        case 8:
            pic_quality = pic_8k;
            break;
        default:
            pic_quality = pic_8k;
            break;
    }
    lcd_draw(12, 0, ICON_WIDTH, ICON_WIDTH, pic_quality);
    
    // print song title:
    lcd_printf(0, 1, "%s                ", g_pcFilenames[g_ulWaveIndex]);
}

void update_play_status()
{
    // note: this function plays in the timer isr!!
    
    static unsigned long last_play_second;
    unsigned long play_second;
    char play_time[16];
    
    static uint8_t pic_cd_index = 0;
    
    // draw cd animation
    if (g_sSoundState == S_PLAY_ING)
    {
        lcd_draw(14, 0, ICON_WIDTH, ICON_WIDTH, pic_cd[pic_cd_index]);
        pic_cd_index ++;
        
        if (pic_cd_index == 4)
        {
            pic_cd_index = 0;
        }
    }
    
    if (lcd_status_refresh > 0)
    {
        switch (lcd_status_refresh_item)
        {
            case REF_STOP:
                lcd_printf(0, 2, "Stopped         ");
                break;
            case REF_PAUSE:
                lcd_printf(0, 2, "Playing Paused  ");
                break;
            case REF_VOLUME:
                lcd_printf(0, 2, "Volume=%d       ", g_DACVolume);
                break;
            default:
                break;
        }
        lcd_status_refresh --;
    }
    else if ((g_sSoundState == S_PLAY_ING) || (g_sSoundState == S_RECORD_ING))
    {
        // draw play time or volume
        play_second = SoundGetTime(&g_sWaveHeader, play_time, sizeof(play_time)); 
        if (play_second != last_play_second)
        {
            lcd_printf(0, 2, "%s     ", play_time);
        }
    }
}

void Timer1AIntHandler(void)
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    
    update_play_status();
}

void player_sm()
{
    switch(g_sSoundState)
    {
        //
        // Not playing audio. Loop, waiting for button presses, until
        // the user selects an audio file to play.
        //
        case S_STOP:
        {
            //
            // PLAY - Select button was pressed.
            //
            switch (g_button_press)
            {
                case BT_SELECT:
                {
                    //
                    // Try to open the selected filename. If successful,
                    // change state to playing, update the screen title to
                    // "PLAYING", and begin playing the audio file.
                    //
                    if(SoundOpen(g_pcFilenames[g_ulWaveIndex], 
                                  &g_sWaveHeader))
                    {
                        g_sSoundState = S_PLAY_ING;
                        
                        SoundPlay(); 
                        
                        update_song_title();
                        update_play_status();
                    }
                    //
                    // If opening the file failed, clear the screen and
                    // display error message.
                    //
                    else
                    {
                        lcd_printf(0, 0xFF, "!!!FILE ERR!!!");
                    }
                    break;
                }
                //
                // NEXT SONG - Left button was pressed.
                //
                case BT_LEFT:
                {
                    //
                    // Decrement the wave clip index.
                    //
                    g_ulWaveIndex = g_ulWaveIndex ?
                                   (g_ulWaveIndex - 1) : (g_ulWavCount - 1);

                    //
                    // Draw the next filename.
                    //
                    update_song_title();
                    
                    break;
                }
                //
                // NEXT SONG - Right button was pressed.
                //
                case BT_RIGHT:
                {
                    //
                    // Increment the wave clip index.
                    //
                    g_ulWaveIndex++;

                    if(g_ulWaveIndex >= g_ulWavCount)
                    {
                        g_ulWaveIndex = 0;
                    }

                    //
                    // Draw the next filename.
                    //
                    update_song_title();
                    
                    break;
                }
                case BT_CANCEL:
                {
                    g_sSoundState = S_RECORD_ING;

#if 1
                    f_unlink("0:/record.wav");
#endif
                    mic_record_start("0:/record.wav", &g_sWaveHeader);
                }
                default:
                {
                    break;
                }
            }

            break;
        } // S_STOP

        //
        // Song is being played. Just loop, waiting for the song to
        // finish, or the user to press a button.
        //
        case S_PLAY_ING:
        {
            //
            // Check to see if audio was finished playing.
            //
            if(!SoundPlaybackStatus())
            {
                SoundStop();
                g_sSoundState = S_STOP;
            }

            switch (g_button_press)
            {
                //
                // PAUSE - Select button was pressed.
                //
                case BT_SELECT:
                {
                    SoundPause();
                    g_sSoundState = S_PLAY_PAUSE;
                    
                    lcd_status_refresh = 12;
                    lcd_status_refresh_item = REF_PAUSE;
                    
                    break;
                }
                case BT_LEFT:
                case BT_RIGHT:
                case BT_CANCEL:
                {
                    SoundStop();
                    g_sSoundState = S_STOP;
                    
                    lcd_status_refresh = 4;
                    lcd_status_refresh_item = REF_STOP;
                    
                    break;
                }
                default:
                {
                    break;
                }
            }
            break;
        } // S_PLAY_ING
        
        case S_PLAY_PAUSE:
        {
            switch (g_button_press)
            {
                case BT_SELECT:
                {
                    // active playing
                    SoundPause();
                    g_sSoundState = S_PLAY_ING;
                    
                    break;
                }
                case BT_LEFT:
                case BT_RIGHT:
                case BT_CANCEL:
                {
                    SoundStop();
                    g_sSoundState = S_STOP;
                    
                    lcd_status_refresh = 4;
                    lcd_status_refresh_item = REF_STOP;
                    
                    break;
                }
                default:
                {
                    break;
                }
            }
            break;
        } // S_PLAY_PAUSE
        
        case S_RECORD_ING:
        {
            if (g_button_press == BT_CANCEL)
            {
                mic_record_stop();
                g_sSoundState = S_STOP;
                
                // record creates a new file, so we'd regenerate the file list
                PopulateFileList();
            }
            break;
        }
        default:
        {
            break;
        } // S_RECORD_ING
    }// end switch

#if 0
    //
    // NOTE: Volume functionality during playback may
    // require a stronger I2C pull-up. This is verified
    // for operation using an effective pull-up R of
    // 750 ohms.

    //
    // VOLUME UP - Up button was pressed.
    //
    if(g_button_press == BT_UP)
    {
        //
        // Increase the volume by 10% and update the display.
        //
        SoundVolumeUp( 10 );
        g_DACVolume = SoundVolumeGet();
        //updateVolumeSlider();
    }

    //
    // VOLUME DOWN - Down button was pressed.
    //
    else if(g_button_press == BT_DOWN)
    {
        //
        // Decrease the volume by 10% and update the display.
        //
        SoundVolumeDown( 10 );
        g_DACVolume = SoundVolumeGet();
        //updateVolumeSlider();
    }
#endif
    
    g_button_press = BT_NONE;
}

void set_volume()
{
#define ADC_RESOLUTION          0x1000 // 12bit
#define VOLUME_DELTA_THRESHOLD  0x2
    
#define ALPHA_VOLUME            0.1
            
    int32_t volume_value;
    
    g_DACVolume = SoundVolumeGet();
    
    volume_value = adc_read(ADC0_BASE)*(100*(1+ALPHA_VOLUME))/ADC_RESOLUTION;
    
#if 0
    if (((g_DACVolume - volume_value) > VOLUME_DELTA_THRESHOLD) ||
        ((volume_value - g_DACVolume) > VOLUME_DELTA_THRESHOLD))
    {
        SoundVolumeSet(volume_value);
        g_DACVolume = SoundVolumeGet();
    }
#else
    g_DACVolume = SoundVolumeGet();
    
    // y[i] = y[i-1] + a*(x[i] - y[i-1])
    volume_value = g_DACVolume + ALPHA_VOLUME*(volume_value - g_DACVolume);
    
    if (g_DACVolume != volume_value)
    {
        SoundVolumeSet(volume_value);
        
        lcd_status_refresh = 8;
        lcd_status_refresh_item = REF_VOLUME;
    }
#endif
}

void app_main_player(void)
{
    uint32_t idle_loops = 0;
    char file_name[_MAX_LFN];
    
    uint8_t file_count;
    
    // mp3 mode:
    lcd_clear();
    lcd_printf(0xFF, 0, "Player");
    
    adc_init(ADC0_BASE, ADC_CTL_CH2);
    
    // initialize sound driver:
    SoundInit();
    
    // register key call back:
    gpio_int_register(&gpio_button_down_player);
    
    // generate playlist:
    file_count = PopulateFileList();
    
    update_song_title();
    
    //
    // Configure the peripheral timer 0A to cause an interrupt
    // every second. This is used to update on-screen information.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet()/4);
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); 
    TimerEnable(TIMER1_BASE, TIMER_A);
    
    while (1)
    {
        // TODO: replace with adc int
        set_volume();
        player_sm();
        
        __wfi();
        
        idle_loops ++;
        
        if ((idle_loops & 0x7FFF) == 0x7FFF) // 32767
        {
            m4_dbgprt("idle_loop=%d\r\n", idle_loops);
        }
    }
}
