#include "m4_driverlib.h"
#include "m4mid_common.h"

/* interface for adafruit */
inline void spiWrite(uint8_t data)
{
    spi_put(M4M_SPI_BASE_LCD, data);
}

inline void digitalWrite()
{
    
}

inline void digitalRead()
{
    
}

inline void delay()
{
    
}
