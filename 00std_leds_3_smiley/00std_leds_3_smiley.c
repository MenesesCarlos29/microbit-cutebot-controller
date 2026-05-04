#include "nrf52833.h"

#define ROW1_DISCONNECT()    (NRF_P0->PIN_CNF[21]     = 0x00000002 )          // P0.21
#define ROW2_DISCONNECT()    (NRF_P0->PIN_CNF[22]     = 0x00000002 )          // P0.22
#define ROW3_DISCONNECT()    (NRF_P0->PIN_CNF[15]     = 0x00000002 )          // P0.15
#define ROW4_DISCONNECT()    (NRF_P0->PIN_CNF[24]     = 0x00000002 )          // P0.24
#define ROW5_DISCONNECT()    (NRF_P0->PIN_CNF[19]     = 0x00000002 )          // P0.19

#define ROW1_OUTPUT()        (NRF_P0->PIN_CNF[21]     = 0x00000003 )          // P0.21
#define ROW2_OUTPUT()        (NRF_P0->PIN_CNF[22]     = 0x00000003 )          // P0.22
#define ROW3_OUTPUT()        (NRF_P0->PIN_CNF[15]     = 0x00000003 )          // P0.15
#define ROW4_OUTPUT()        (NRF_P0->PIN_CNF[24]     = 0x00000003 )          // P0.24
#define ROW5_OUTPUT()        (NRF_P0->PIN_CNF[19]     = 0x00000003 )          // P0.19

#define ROW1_HIGH()          (NRF_P0->OUTSET          = (0x00000001 << 21) )   // P0.21
#define ROW2_HIGH()          (NRF_P0->OUTSET          = (0x00000001 << 22) )   // P0.22
#define ROW3_HIGH()          (NRF_P0->OUTSET          = (0x00000001 << 15) )   // P0.15
#define ROW4_HIGH()          (NRF_P0->OUTSET          = (0x00000001 << 24) )   // P0.24
#define ROW5_HIGH()          (NRF_P0->OUTSET          = (0x00000001 << 19) )   // P0.19
                        
#define COL1_DISCONNECT()    (NRF_P0->PIN_CNF[28]     = 0x00000002 )           // P0.28
#define COL2_DISCONNECT()    (NRF_P0->PIN_CNF[11]     = 0x00000002 )           // P0.11
#define COL3_DISCONNECT()    (NRF_P0->PIN_CNF[31]     = 0x00000002 )           // P0.31
#define COL4_DISCONNECT()    (NRF_P1->PIN_CNF[05]     = 0x00000002 )           // P1.05
#define COL5_DISCONNECT()    (NRF_P0->PIN_CNF[30]     = 0x00000002 )           // P0.30

#define COL1_OUTPUT()        (NRF_P0->PIN_CNF[28]     = 0x00000003 )           // P0.28
#define COL2_OUTPUT()        (NRF_P0->PIN_CNF[11]     = 0x00000003 )           // P0.11
#define COL3_OUTPUT()        (NRF_P0->PIN_CNF[31]     = 0x00000003 )           // P0.31
#define COL4_OUTPUT()        (NRF_P1->PIN_CNF[05]     = 0x00000003 )           // P1.05
#define COL5_OUTPUT()        (NRF_P0->PIN_CNF[30]     = 0x00000003 )           // P0.30
                             
#define COL1_LOW()           (NRF_P0->OUTCLR          = (0x00000001 << 28) )   // P0.28
#define COL2_LOW()           (NRF_P0->OUTCLR          = (0x00000001 << 11) )   // P0.11
#define COL3_LOW()           (NRF_P0->OUTCLR          = (0x00000001 << 31) )   // P0.31
#define COL4_LOW()           (NRF_P1->OUTCLR          = (0x00000001 << 05) )   // P1.05
#define COL5_LOW()           (NRF_P0->OUTCLR          = (0x00000001 << 30) )   // P0.30

typedef enum {
    LED11,LED12,LED13,LED14,LED15,
    LED21,LED22,LED23,LED24,LED25,
    LED31,LED32,LED33,LED34,LED35,
    LED41,LED42,LED43,LED44,LED45,
    LED51,LED52,LED53,LED54,LED55
} led_id_t;

/**
     12345
   1 .....
   2 .#.#.
   3 .....
   4 #...#
   5 .###.
*/

const led_id_t NUM_5[] = { LED11, LED12, LED13, LED21, LED31, LED32, LED33, LED43, LED51, LED52, LED53 };
const led_id_t NUM_4[] = { LED11, LED21, LED31, LED32, LED33, LED13, LED23, LED43, LED53 };
const led_id_t NUM_3[] = { LED11, LED12, LED13, LED23, LED31, LED32, LED33, LED43, LED51, LED52, LED53 };
const led_id_t NUM_2[] = { LED11, LED12, LED13, LED23, LED31, LED32, LED33, LED41, LED51, LED52, LED53 };
const led_id_t NUM_1[] = { LED13, LED23, LED33, LED43, LED53 };

void leds_all_off(void) {
   
    ROW1_DISCONNECT();
    ROW2_DISCONNECT();
    ROW3_DISCONNECT();
    ROW4_DISCONNECT();
    ROW5_DISCONNECT();

    COL1_DISCONNECT();
    COL2_DISCONNECT();
    COL3_DISCONNECT();
    COL4_DISCONNECT();
    COL5_DISCONNECT();
}

void leds_on(led_id_t led_id) {
    
    switch(led_id) {
        
        case LED11: ROW1_OUTPUT(); ROW1_HIGH(); COL1_OUTPUT(); COL1_LOW(); break;
        case LED12: ROW1_OUTPUT(); ROW1_HIGH(); COL2_OUTPUT(); COL2_LOW(); break;
        case LED13: ROW1_OUTPUT(); ROW1_HIGH(); COL3_OUTPUT(); COL3_LOW(); break;
        case LED14: ROW1_OUTPUT(); ROW1_HIGH(); COL4_OUTPUT(); COL4_LOW(); break;
        case LED15: ROW1_OUTPUT(); ROW1_HIGH(); COL5_OUTPUT(); COL5_LOW(); break;

        case LED21: ROW2_OUTPUT(); ROW2_HIGH(); COL1_OUTPUT(); COL1_LOW(); break;
        case LED22: ROW2_OUTPUT(); ROW2_HIGH(); COL2_OUTPUT(); COL2_LOW(); break;
        case LED23: ROW2_OUTPUT(); ROW2_HIGH(); COL3_OUTPUT(); COL3_LOW(); break;
        case LED24: ROW2_OUTPUT(); ROW2_HIGH(); COL4_OUTPUT(); COL4_LOW(); break;
        case LED25: ROW2_OUTPUT(); ROW2_HIGH(); COL5_OUTPUT(); COL5_LOW(); break;

        case LED31: ROW3_OUTPUT(); ROW3_HIGH(); COL1_OUTPUT(); COL1_LOW(); break;
        case LED32: ROW3_OUTPUT(); ROW3_HIGH(); COL2_OUTPUT(); COL2_LOW(); break;
        case LED33: ROW3_OUTPUT(); ROW3_HIGH(); COL3_OUTPUT(); COL3_LOW(); break;
        case LED34: ROW3_OUTPUT(); ROW3_HIGH(); COL4_OUTPUT(); COL4_LOW(); break;
        case LED35: ROW3_OUTPUT(); ROW3_HIGH(); COL5_OUTPUT(); COL5_LOW(); break;

        case LED41: ROW4_OUTPUT(); ROW4_HIGH(); COL1_OUTPUT(); COL1_LOW(); break;
        case LED42: ROW4_OUTPUT(); ROW4_HIGH(); COL2_OUTPUT(); COL2_LOW(); break;
        case LED43: ROW4_OUTPUT(); ROW4_HIGH(); COL3_OUTPUT(); COL3_LOW(); break;
        case LED44: ROW4_OUTPUT(); ROW4_HIGH(); COL4_OUTPUT(); COL4_LOW(); break;
        case LED45: ROW4_OUTPUT(); ROW4_HIGH(); COL5_OUTPUT(); COL5_LOW(); break;

        case LED51: ROW5_OUTPUT(); ROW5_HIGH(); COL1_OUTPUT(); COL1_LOW(); break;
        case LED52: ROW5_OUTPUT(); ROW5_HIGH(); COL2_OUTPUT(); COL2_LOW(); break;
        case LED53: ROW5_OUTPUT(); ROW5_HIGH(); COL3_OUTPUT(); COL3_LOW(); break;
        case LED54: ROW5_OUTPUT(); ROW5_HIGH(); COL4_OUTPUT(); COL4_LOW(); break;
        case LED55: ROW5_OUTPUT(); ROW5_HIGH(); COL5_OUTPUT(); COL5_LOW(); break;
    }
}

int main(void) {
    //uint8_t i;
    uint32_t timer;

    //leds_on(LED11);

    // i=0
    while(1) {
     

        leds_all_off();
        leds_on(LED11);
        for(timer = 0; timer < 20000; timer++); 

        leds_all_off();
        leds_on(LED55); 

        for(timer = 0; timer < 1000; timer++); 

    
        //i=0;
        //for(timer = 0; timer < 500000; timer++) {
        //    leds_all_off();
        //    if(i != 3 && timer%2 == 0 ){
        //        leds_on(NUM_5[i]);
        //    }              
            
        //    i++;
        //    if(i == (sizeof(NUM_5)))
        //     { i = 0; }
        //}
        //i=0;
        // for(timer = 0; timer < 500000; timer++) {
        //    leds_all_off();
        //    leds_on(NUM_4[i]);
        //    i++;
        //    if(i == (sizeof(NUM_4)))
        //     { i = 0; }
        //}
        //i=0;
        //for(timer = 0; timer < 500000; timer++) {
        //    leds_all_off();
        //    leds_on(NUM_3[i]);
        //    i++;
        //    if(i == (sizeof(NUM_3)))
        //     { i = 0; }
        //}
        //i=0;
        //for(timer = 0; timer < 500000; timer++) {
        //    leds_all_off();
        //    leds_on(NUM_2[i]);
        //    i++;
        //    if(i == (sizeof(NUM_2)))
        //     { i = 0; }
        //}
        //i=0;
        //for(timer = 0; timer < 500000; timer++) {
        //    leds_all_off();
        //    leds_on(NUM_1[i]);
        //    i++;
        //    if(i == (sizeof(NUM_1)))
        //     { i = 0; }
        //}
        //for(timer = 0; timer < 500000; timer++) {
        //    leds_all_off();
        //}
    }
}
