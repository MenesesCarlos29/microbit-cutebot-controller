#include "nrf52833.h"

int main(void) {
   
    //=== set ROW3 high
    
    // ROW3 == P0.15

    // configure as output
    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ...A A: DIR:   1=Output
    // .... .... .... .... .... .... .... ..B. B: INPUT: 1=Disconnect
    // .... .... .... .... .... .... .... CC.. C: PULL:  0=Disabled
    // .... .... .... .... .... .DDD .... .... D: DRIVE: 0=S0S1
    // .... .... .... ..EE .... .... .... .... E: SENSE: 0=Disabled
    // xxxx xxxx xxxx xx00 xxxx xxxx xxxx 0011 
    //    0    0    0    0    0    0    0    3 0x00000003
    NRF_P0->PIN_CNF[15]           = 0x00000003;

    // set high
    NRF_P0->OUTSET                = (0x00000001 << 15);

    //=== pull COL3 low

    // COL3 == P0.31

    NRF_P0->PIN_CNF[31]           = 0x00000003;

    // set high
    NRF_P0->OUTCLR                = (0x00000001 << 31);

    // main loop
    while(1);
}
