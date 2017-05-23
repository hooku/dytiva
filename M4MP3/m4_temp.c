#include <stdio.h>
#include <stdarg.h>

#include "m4_config.h"
#include "m4_common.h"

#include "tmp100.h"

void temp_init()
{
    TMP100Init();
}

float temp_read()
{
    float temp_degree;
    uint16_t temp_value;
    
    temp_value = TMP100DataRead();
    temp_degree = ((temp_value >> 8) & 0xFF) + ((float)((temp_value >> 4) & 0xF))*0.0625;
    
    m4_dbgprt("raw=%d\r\n", ((temp_value >> 4) & 0xF));

    return temp_degree;
}

#if 1
// this function helps determine dypb3 / dypb4
uint8_t temp_addr_test()
{
    if (TMP100AddrTest(0x48) == 1)
    {
        return 4;
    }
    return 3;
}
#endif
