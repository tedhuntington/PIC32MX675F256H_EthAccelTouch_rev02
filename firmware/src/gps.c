//gps.c - GPS functions

#include <proc/p32mz0512efe064.h>

#include "gps.h"

//send string to UART3
uint8_t SendSerial(uint8_t *serstr,uint32_t num)
{
    int i;
    
    i=0;
    while(i<num && !(U3STAbits.UTXBF)) {
        U3TXREG=serstr[i];
        i++;
    } //while    
    
    return(1);
} //uint8_t SendSerial(uint8_t *serstr,uint32_t num)

