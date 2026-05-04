#include "ex2_full_speed.h"
#include <nrf52833.h>

#define PIN_BUTTON_A 14 // Button A - Pin P0.14
#define PIN_BUTTON_B 23 // Button B - Pin P0.23

#define MOTOR_SPEED 100 // Full speed (0...100)

// I2C Commands: {Header, Command, Motor(0x03=Both), Direction, Speed, Padding, Footer}
uint8_t I2CBUF_MOTORS_FWD[]  = {0x99, 0x01, 0x03, 0x01, MOTOR_SPEED, 0x00, 0x88};
uint8_t I2CBUF_MOTORS_BACK[] = {0x99, 0x01, 0x03, 0x00, MOTOR_SPEED, 0x00, 0x88};
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

void init_buttons(void) {
    
    // Pin 14 (Button A) is configured as an INPUT with a PULL-UP resistor.
    // Bit 0 = 0 (Input), Bits 2-3 = 3 (Pull-up)
    NRF_P0->PIN_CNF[PIN_BUTTON_A] = (0 << 0) | (3 << 2); 
    
    // Pin 23 (Button B) is configured.
    NRF_P0->PIN_CNF[PIN_BUTTON_B] = (0 << 0) | (3 << 2);
}

// Returns 1 if Button A is pressed, 0 otherwise.

// (The logic is reversed because we're using a pull-up resistor: when pressed, it reads 0)
int is_button_A_pressed(void) {
    if ((NRF_P0->IN & (1 << PIN_BUTTON_A)) == 0) {
        return 1; 
    }
    return 0; 
}

int is_button_B_pressed(void) {
    if ((NRF_P0->IN & (1 << PIN_BUTTON_B)) == 0) {
        return 1;
    }
    return 0;
}


void run_full_speed() {
    
    // 1. Initialize Hardware
    i2c_init();
    init_buttons();
    
    // Stop the motors for safety at startup
    i2c_send(I2CBUF_MOTORS_STOP, sizeof(I2CBUF_MOTORS_STOP));

    // Variable to remember the current state and avoid sending the same I2C command 1000 times per second
    // 0 = Stop, 1 = Fwd, 2 = Back
    int current_state = 0;
    

    while(1) {
        
        if (is_button_A_pressed()) {
            if (current_state != 1) { // Only send the command if it wasn't already moving forward
                i2c_send(I2CBUF_MOTORS_FWD, sizeof(I2CBUF_MOTORS_FWD));
                current_state = 1;
            }
        } 
        else if (is_button_B_pressed()) {
            if (current_state != 2) {
                i2c_send(I2CBUF_MOTORS_BACK, sizeof(I2CBUF_MOTORS_BACK));
                current_state = 2;
            }
        } 
        else {
            if (current_state != 0) {
                i2c_send(I2CBUF_MOTORS_STOP, sizeof(I2CBUF_MOTORS_STOP));
                current_state = 0;
            }
        }
        
        // Delay to avoid saturating the I2C bus by reading the buttons too fast
        for (volatile int i = 0; i < 50000; i++) {}
    }


}
