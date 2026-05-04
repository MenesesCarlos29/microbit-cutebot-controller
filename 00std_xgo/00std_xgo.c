#include <string.h>
#include "nrf52833.h"

/*
sources:
    https://wiki.elecfreaks.com/en/microbit/robot/xgo-robot-kit/
    https://github.com/elecfreaks/pxt-xgo/blob/master/main.ts
    https://makecode.microbit.org/device/pins
    https://wiki.elecfreaks.com/en/pico/cm4-xgo-robot-kit/advanced-development/serial-protocol/
    
    UART
        tx==P2 onthe micro:bit silkscreen==P0.04 on the nRF (RING2 line on schematic)
        rx==P1 onthe micro:bit silkscreen==P0.03 on the nRF (RING1 line on schematic)
        baudrate: 115200
*/

#define ACTION_GET_DOWN                 1
#define ACTION_STAND_UP                 2
#define ACTION_CREEP_FORWARD            3
#define ACTION_CIRCLE_AROUND            4
#define ACTION_SQUAT_UP                 6
#define ACTION_TURN_ROLL                7
#define ACTION_TURN_PITCH               8
#define ACTION_TURN_YAW                 9
#define ACTION_THREE_AXIS_ROTATION      10
#define ACTION_PEE                      11
#define ACTION_SIT_DOWN                 12
#define ACTION_WAVE1                    13
#define ACTION_STRETCH                  14
#define ACTION_WAVE2                    15
#define ACTION_SWING_LEFT_AND_RIGHT     16
#define ACTION_BEGGING_FOR_FOOD         17
#define ACTION_LOOKING_FOR_FOOD         18
#define ACTION_SHAKE_HANDS              19
#define ACTION_CHICKEN_HEAD             20
#define ACTION_PUSH_UPS                 21
#define ACTION_LOOK_AROUND              22
#define ACTION_DANCE                    23
#define ACTION_NAUGHTY                  24
#define ACTION_CATCH_UP                 128
#define ACTION_CAUGHT                   129
#define ACTION_CATCH                    130
#define ACTION_RESTORE DEFAULT POSTURE  255

uint8_t BUF_ACTION[] = {
    0x55,            // header 1
    0x00,            // header 2
    0x09,            // length
    0x00,            // command_type
    0x3e,            // address (0x3e==action command)
    ACTION_SIT_DOWN, // data    (action)
    0x00,            // crc (will be computed below)
    0x00,            // end 1
    0xAA,            // end 2
};

uint8_t uartvars_rxBuf[9];

uint8_t _compute_crc(void) {
    uint16_t tmp;
    tmp += BUF_ACTION[2]; // length
    tmp += BUF_ACTION[3]; // command_type
    tmp += BUF_ACTION[4]; // address
    tmp += BUF_ACTION[5]; // datatmpz
    return 0xff - (uint8_t)(tmp & 0x00ff);
}

int main(void) {
    
    // start hfclock
    NRF_CLOCK->EVENTS_HFCLKSTARTED     = 0;
    NRF_CLOCK->TASKS_HFCLKSTART        = 0x00000001;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    // configure P0.04 (UART TX) as output
    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ...A A: DIR:     1=Output
    // .... .... .... .... .... .... .... ..B. B: INPUT:   1=Disconnect
    // .... .... .... .... .... .... .... CC.. C: PULL:    0=Disabled
    // .... .... .... .... .... .DDD .... .... D: DRIVE:   0=S0S1
    // .... .... .... ..EE .... .... .... .... E: SENSE:   0=Disabled
    // xxxx xxxx xxxx xx00 xxxx xxxx xxxx 0011 
    //    0    0    0    0    0    0    0    3 0x00000003
    NRF_P0->PIN_CNF[4]                 = 0x00000003;
    NRF_P0->OUTSET                     = (0x00000001 << 4);

    // configure P0.03 (UART RX) as input
    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ...A A: DIR:     0=Input
    // .... .... .... .... .... .... .... ..B. B: INPUT:   0=Connect
    // .... .... .... .... .... .... .... CC.. C: PULL:    3=Pullup
    // .... .... .... .... .... .DDD .... .... D: DRIVE:   0=S0S1
    // .... .... .... ..EE .... .... .... .... E: SENSE:   0=Disabled
    // xxxx xxxx xxxx xx00 xxxx xxxx xxxx 1100 
    //    0    0    0    0    0    0    0    c 0x0000000c
    NRF_P0->PIN_CNF[3]                 = 0x0000000c;

    // configure
    NRF_UARTE0->TXD.PTR                = (uint32_t)BUF_ACTION;
    NRF_UARTE0->TXD.MAXCNT             = sizeof(BUF_ACTION);
    NRF_UARTE0->RXD.PTR                = (uint32_t)uartvars_rxBuf;
    NRF_UARTE0->RXD.MAXCNT             = sizeof(uartvars_rxBuf);
    NRF_UARTE0->PSEL.TXD               = 0x00000004; // 0x00000004==P0.04
    NRF_UARTE0->PSEL.RXD               = 0x00000003; // 0x00000003==P0.03
    NRF_UARTE0->CONFIG                 = 0x00000000; // 0x00000000==no flow control, no parity bits, 1 stop bit
    NRF_UARTE0->BAUDRATE               = 0x01D60000; // 0x01D60000==Baud115200
    
    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ...A A: CTS
    // .... .... .... .... .... .... .... ..B. B: NCTS
    // .... .... .... .... .... .... .... .C.. C: RXDRDY
    // .... .... .... .... .... .... ...D .... D: ENDRX
    // .... .... .... .... .... .... E... .... E: TXDRDY
    // .... .... .... .... .... ...F .... .... F: ENDTX
    // .... .... .... .... .... ..G. .... .... G: ERROR
    // .... .... .... ..H. .... .... .... .... H: RXTO
    // .... .... .... I... .... .... .... .... I: RXSTARTED
    // .... .... ...J .... .... .... .... .... J: TXSTARTED
    // .... .... .L.. .... .... .... .... .... L: TXSTOPPED
    // xxxx xxxx x0x0 0x0x xxxx xx01 0xx0 x000 
    //    0    0    0    0    0    1    0    0 0x00000100
    //NRF_UARTE0->INTENSET               = 0x00000100;
    NRF_UARTE0->ENABLE                 = 0x00000008; // 0x00000008==enable
    
    BUF_ACTION[6] = _compute_crc();

    NRF_UARTE0->EVENTS_ENDTX           = 0;
    NRF_UARTE0->TASKS_STARTTX          = 0x00000001;
    while(NRF_UARTE0->EVENTS_ENDTX== 0);

    while(1);
}
