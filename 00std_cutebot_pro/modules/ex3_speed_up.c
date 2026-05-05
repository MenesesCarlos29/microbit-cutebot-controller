#include "ex3_speed_up.h"
#include <nrf52833.h>

#define PIN_BUTTON_A 14 // Button A - Pin P0.14
#define PIN_BUTTON_B 23 // Button B - Pin P0.23

// #define MOTOR_SPEED 100 // Full speed (0...100)

// I2C Commands: {Header, Command, Motor(0x03=Both), Direction, Speed, Padding, Footer}
uint8_t I2CBUF_MOTORS_FWD[]  = {0x99, 0x01, 0x03, 0x01, 0, 0x00, 0x88};
uint8_t I2CBUF_MOTORS_BACK[] = {0x99, 0x01, 0x03, 0x00, 0, 0x00, 0x88};
uint8_t I2CBUF_MOTORS_STOP[] = {0x99, 0x09, 0x03, 0x00,        0x00, 0x00, 0x88};


// ==============================================================================
// HARDWARE FUNCTIONS (I2C and Buttons)
// ==============================================================================

void i2c_init(void) {
    // SCL (P0.26) and SDA (P1.00) pins configuration
    NRF_P0->PIN_CNF[26] = 0x00000602; 
    NRF_P1->PIN_CNF[0]  = 0x00000602; 

    NRF_TWI0->ENABLE     = 0x00000005; // Enable TWI0
    NRF_TWI0->PSEL.SCL   = 0x0000001a; // Assign Pin 26 to SCL
    NRF_TWI0->PSEL.SDA   = 0x00000020; // Assign Pin 0 to SDA
    NRF_TWI0->FREQUENCY  = 0x01980000; // 100 kbps
    NRF_TWI0->ADDRESS    = 0x10;       // Cutebot I2C Address
}

void i2c_send(uint8_t* buf, uint8_t buflen) {
    uint8_t i = 0;
    NRF_TWI0->TXD            = buf[i];
    NRF_TWI0->EVENTS_TXDSENT = 0;
    NRF_TWI0->TASKS_STARTTX  = 1;
    i++;
    while(i < buflen) {
        while(NRF_TWI0->EVENTS_TXDSENT == 0);
        NRF_TWI0->EVENTS_TXDSENT  = 0;
        NRF_TWI0->TXD             = buf[i];
        i++;
    }
    while(NRF_TWI0->EVENTS_TXDSENT == 0);
    NRF_TWI0->TASKS_STOP = 1;
}



void run_gradual_speed() {
    
    // 1. Initialize Hardware
    i2c_init();
    int speed = 0;
    // Stop the motors for safety at startup
    i2c_send(I2CBUF_MOTORS_STOP, sizeof(I2CBUF_MOTORS_STOP));

    // Variable to remember the current state and avoid sending the same I2C command 1000 times per second
    // 0 = Stop, 1 = Fwd, 2 = Back
    int current_state = 0;
    

    while(1) {
        
        for (volatile int i = 0; i < 1500000; i++);
        if(current_state == 0) {
            speed += 10; // Increase speed by 10
            if (speed >= 100) { // Cap speed at 100
                current_state = 1;
                speed = 100; 
            } 
            I2CBUF_MOTORS_FWD[4] = speed; // Update speed in I2C command
            i2c_send(I2CBUF_MOTORS_FWD, sizeof(I2CBUF_MOTORS_FWD));
        }
        else if(current_state == 1) {
            speed -= 10; // Decrease speed by 10
            if (speed <= 0) { // Cap speed at 0
                current_state = 2;
                speed = 0; 
            } 
            I2CBUF_MOTORS_FWD[4] = speed; // Update speed in I2C command
            i2c_send(I2CBUF_MOTORS_FWD, sizeof(I2CBUF_MOTORS_FWD));
        }else if(current_state == 2) {
            speed += 10; // Increase speed by 10
            if (speed >= 100) { // Cap speed at 100
                current_state = 3;
                speed = 100; 
            } 
            I2CBUF_MOTORS_BACK[4] = speed; // Update speed in I2C command
            i2c_send(I2CBUF_MOTORS_BACK, sizeof(I2CBUF_MOTORS_BACK));
        }else if(current_state == 3) {
            speed -= 10; // Decrease speed by 10
            if (speed <= 0) { // Cap speed at 0
                current_state = 0;
                speed = 0; 
            } 
            I2CBUF_MOTORS_BACK[4] = speed; // Update speed in I2C command
            i2c_send(I2CBUF_MOTORS_BACK, sizeof(I2CBUF_MOTORS_BACK));
        }
              
        // Delay to avoid saturating the I2C bus by reading the buttons too fast
        for (volatile int i = 0; i < 50000; i++) {}
    }


}
