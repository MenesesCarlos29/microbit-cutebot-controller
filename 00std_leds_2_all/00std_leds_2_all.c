#include "nrf52833.h"

int main(void) {
   
    //=== set ROW1-ROW5 high
    
    // ROW1 == P0.21
    NRF_P0->PIN_CNF[21]           = 0x00000003;
    NRF_P0->OUTSET                = (0x00000001 << 21);
    // ROW2 == P0.22
    NRF_P0->PIN_CNF[22]           = 0x00000003;
    NRF_P0->OUTSET                = (0x00000001 << 22);
    // ROW3 == P0.15
    NRF_P0->PIN_CNF[15]           = 0x00000003;
    NRF_P0->OUTSET                = (0x00000001 << 15);
    // ROW4 == P0.24
    NRF_P0->PIN_CNF[24]           = 0x00000003;
    NRF_P0->OUTSET                = (0x00000001 << 24);
    // ROW5 == P0.19
    NRF_P0->PIN_CNF[19]           = 0x00000003;
    NRF_P0->OUTSET                = (0x00000001 << 19);

    //=== pull COL3 low

    // COL1 == P0.28
    NRF_P0->PIN_CNF[28]           = 0x00000003;
    NRF_P0->OUTCLR                = (0x00000001 << 28);
    // COL2 == P0.11
    NRF_P0->PIN_CNF[11]           = 0x00000003;
    NRF_P0->OUTCLR                = (0x00000001 << 11);
    // COL3 == P0.31
    NRF_P0->PIN_CNF[31]           = 0x00000003;
    NRF_P0->OUTCLR                = (0x00000001 << 31);
    // COL4 == P1.05
    NRF_P1->PIN_CNF[5]            = 0x00000003;
    NRF_P1->OUTCLR                = (0x00000001 << 5);
    // COL5 == P0.30
    NRF_P0->PIN_CNF[30]           = 0x00000003;
    NRF_P0->OUTCLR                = (0x00000001 << 30);

    // main loop
    while(1);
}
