#include <string.h>
#include "nrf52833.h"

/*
sources:
    https://wiki.elecfreaks.com/en/microbit/microbit-smart-car/microbit-ringbit-car-v2/
    https://github.com/elecfreaks/circuitpython_ringbit/blob/main/ringbit.py
    https://github.com/elecfreaks/pxt-ringbitcar-old/blob/master/ringbitcar.ts
    https://learn.adafruit.com/adafruit-neopixel-uberguide/advanced-coding#writing-your-own-library-2894697
    
    servos
        left ==P1 onthe micro:bit silkscreen==P0.03 on the nRF (RING1 line on schematic)
        right==P2 onthe micro:bit silkscreen==P0.04 on the nRF (RING2 line on schematic)
    per https://shop.elecfreaks.com/products/elecfreaks-360-digital-servo-with-wheel-and-tire-ef90d
        looks like
            max backwards rotation speed: pulse width of  500us
                       no rotation speed: pulse width of 1500us
            max forwards  rotation speed: pulse width of 2500us

    LEDs
        NeoPixel connected to P0 onthe micro:bit silkscreen==P0.02 on the nRF (RING0 line on schematic)
*/

#define SPEED_FORWARD   12000
#define SPEED_NONE      10000
#define SPEED_BACKWARD   8000

#define PIN_MOTOR_LEFT  3
#define PIN_MOTOR_RIGHT 3

uint16_t left_motor_pwm_comp[1];
uint16_t right_motor_pwm_comp[1];

void motor_left_init(void) {
    // configure P0.03 as output
    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ...A A: DIR:     1=Output
    // .... .... .... .... .... .... .... ..B. B: INPUT:   1=Disconnect
    // .... .... .... .... .... .... .... CC.. C: PULL:    0=Disabled
    // .... .... .... .... .... .DDD .... .... D: DRIVE:   0=S0S1
    // .... .... .... ..EE .... .... .... .... E: SENSE:   0=Disabled
    // xxxx xxxx xxxx xx00 xxxx xxxx xxxx 0011 
    //    0    0    0    0    0    0    0    3 0x00000003
    NRF_P0->PIN_CNF[3]            = 0x00000003;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... ...A AAAA A: PIN:      00 (P0.00)
    // .... .... .... .... .... .... ..B. .... B: PORT:     0  (P0.00)
    // C... .... .... .... .... .... .... .... C: CONNECT:  0=Connected
    // 0xxx xxxx xxxx xxxx xxxx xxxx xx00 0011 
    //    0    0    0    0    0    0    0    3 0x00000003
    NRF_PWM0->PSEL.OUT[0]         = 0x00000003;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ...A A: ENABLE:   1=Enabled
    // 0xxx xxxx xxxx xxxx xxxx xxxx xxxx xxx1 
    //    0    0    0    0    0    0    0    1 0x00000001
    NRF_PWM0->ENABLE              = 0x00000001;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ...A A: UPDOWN:   0=Up
    // 0xxx xxxx xxxx xxxx xxxx xxxx xxxx xxx0 
    //    0    0    0    0    0    0    0    0 0x00000000
    NRF_PWM0->MODE                = 0x00000000;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... .AAA A: PRESCALER: 2=DIV_4
    // 0xxx xxxx xxxx xxxx xxxx xxxx xxxx x000 
    //    0    0    0    0    0    0    0    0 0x00000000
    NRF_PWM0->PRESCALER           = 2;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... AAAA AAAA AAAA AAAA A: CNT:      0=Disabled
    // 0xxx xxxx xxxx xxxx 0000 0000 0000 0000 
    //    0    0    0    0    0    0    0    0 0x00000000
    NRF_PWM0->LOOP                = 0x00000000;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ..AA A: LOAD:     0=Common
    // .... .... .... .... .... ...B .... .... B: MODE:     0=RefreshCount
    // 0xxx xxxx xxxx xxxx xxxx xxx0 xxxx xx00 
    //    0    0    0    0    0    0    0    0 0x00000000
    NRF_PWM0->DECODER             = 0x00000000;
    NRF_PWM0->SEQ[0].PTR          = (uint32_t)(left_motor_pwm_comp);
    NRF_PWM0->SEQ[0].CNT          = (sizeof(left_motor_pwm_comp) / sizeof(uint16_t));

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... AAAA AAAA AAAA AAAA AAAA AAAA A: CNT:      0=Continuous
    // 0xxx xxxx 0000 0000 0000 0000 0000 0000 
    //    0    0    0    0    0    0    0    0 0x00000000
    NRF_PWM0->SEQ[0].REFRESH      = 0;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... AAAA AAAA AAAA AAAA AAAA AAAA A: CNT
    // 0xxx xxxx 0000 0000 0000 0000 0000 0000 
    //    0    0    0    0    0    0    0    0 0x00000000
    NRF_PWM0->SEQ[0].ENDDELAY     = 0;

    NRF_PWM0->COUNTERTOP          = 16000; // 16000@4MHz==4000us
    //left_motor_pwm_comp[0]        =  2000; //  2000@4MHz== 500us
    left_motor_pwm_comp[0]        = 10000; //  10000@4MHz==2500us
    //left_motor_pwm_comp[0]        = 10000; // 10000@4MHz==2500us

    NRF_PWM0->EVENTS_SEQSTARTED[0]=0;
    NRF_PWM0->TASKS_SEQSTART[0]   = 0x00000001;
    while(NRF_PWM0->EVENTS_SEQSTARTED[0]==0);
}

void motor_left_speed(uint16_t speed) {
    
    if(NRF_PWM0->EVENTS_SEQSTARTED[0]==1) {
        NRF_PWM0->EVENTS_STOPPED  = 0;
        NRF_PWM0->TASKS_STOP      = 0x00000001;
        while(NRF_PWM0->EVENTS_STOPPED==0);
    }

    left_motor_pwm_comp[0]        = speed;

    NRF_PWM0->EVENTS_SEQSTARTED[0]=0;
    NRF_PWM0->TASKS_SEQSTART[0]   = 0x00000001;
    while(NRF_PWM0->EVENTS_SEQSTARTED[0]==0);
}

void motor_right_init(void) {
    
    NRF_P0->PIN_CNF[4]            = 0x00000003;
    NRF_PWM1->PSEL.OUT[0]         = 0x00000004;
    NRF_PWM1->ENABLE              = 0x00000001;
    NRF_PWM1->MODE                = 0x00000000;
    NRF_PWM1->PRESCALER           = 2;
    NRF_PWM1->LOOP                = 0x00000000;
    NRF_PWM1->DECODER             = 0x00000000;
    NRF_PWM1->SEQ[0].PTR          = (uint32_t)(right_motor_pwm_comp);
    NRF_PWM1->SEQ[0].CNT          = (sizeof(right_motor_pwm_comp) / sizeof(uint16_t));
    NRF_PWM1->SEQ[0].REFRESH      = 0;
    NRF_PWM1->SEQ[0].ENDDELAY     = 0;
    NRF_PWM1->COUNTERTOP          = 16000;
}

void motor_right_speed(uint16_t speed) {

    if(NRF_PWM1->EVENTS_SEQSTARTED[0]==1) {
        NRF_PWM1->EVENTS_STOPPED  = 0;
        NRF_PWM1->TASKS_STOP      = 0x00000001;
        while(NRF_PWM1->EVENTS_STOPPED==0);
    }

    right_motor_pwm_comp[0]       = speed;

    NRF_PWM1->EVENTS_SEQSTARTED[0]=0;
    NRF_PWM1->TASKS_SEQSTART[0]   = 0x00000001;
    while(NRF_PWM1->EVENTS_SEQSTARTED[0]==0);
}


int main(void) {

    motor_left_init();
    motor_right_init();

    motor_left_speed(SPEED_FORWARD);
    motor_left_speed(SPEED_BACKWARD);
    motor_left_speed(SPEED_NONE);

    motor_right_speed(SPEED_FORWARD);
    motor_right_speed(SPEED_BACKWARD);
    motor_right_speed(SPEED_NONE);

    while(1);
}
