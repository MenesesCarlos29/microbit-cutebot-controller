#include "i2c.h"

void i2c_init(void) {
    // Configure SCL pin: Input, Connect, Pullup, Drive S0D1
    // DIR=0 (Input), INPUT=0 (Connect), PULL=3 (Pullup), DRIVE=6 (S0D1)
    NRF_P0->PIN_CNF[I2C_SCL_PIN] = 0x0000060C;  // 0000 0000 0000 0000 0000 0110 0000 1100
    
    // Configure SDA pin: Input, Connect, Pullup, Drive S0D1
    NRF_P0->PIN_CNF[I2C_SDA_PIN] = 0x0000060C;
    
    // Configure TWI0 pins
    NRF_TWIM0->PSEL.SCL = I2C_SCL_PIN;
    NRF_TWIM0->PSEL.SDA = I2C_SDA_PIN;
    
    // Set I2C frequency to 100kHz
    NRF_TWIM0->FREQUENCY = 0x01980000;
    
    // Enable TWI (ENABLE = 6 for enabled)
    NRF_TWIM0->ENABLE = 6;
}

void i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
    uint8_t tx_buffer[2];
    tx_buffer[0] = reg_addr;
    tx_buffer[1] = data;
    
    NRF_TWIM0->ADDRESS = device_addr;
    NRF_TWIM0->TXD.PTR = (uint32_t)tx_buffer;
    NRF_TWIM0->TXD.MAXCNT = 2;
    
    // Clear events
    NRF_TWIM0->EVENTS_STOPPED = 0;
    NRF_TWIM0->EVENTS_ERROR = 0;
    
    // Start transmission
    NRF_TWIM0->TASKS_STARTTX = 1;
    
    // Wait for transmission to complete
    while (NRF_TWIM0->EVENTS_STOPPED == 0 && NRF_TWIM0->EVENTS_ERROR == 0);
    
    // Stop
    NRF_TWIM0->TASKS_STOP = 1;
    while (NRF_TWIM0->EVENTS_STOPPED == 0);
    
    // Clear stop event
    NRF_TWIM0->EVENTS_STOPPED = 0;
}

uint8_t i2c_read_byte(uint8_t device_addr, uint8_t reg_addr) {
    uint8_t rx_buffer;
    
    // First write the register address
    NRF_TWIM0->ADDRESS = device_addr;
    NRF_TWIM0->TXD.PTR = (uint32_t)&reg_addr;
    NRF_TWIM0->TXD.MAXCNT = 1;
    
    NRF_TWIM0->EVENTS_STOPPED = 0;
    NRF_TWIM0->EVENTS_ERROR = 0;
    
    NRF_TWIM0->TASKS_STARTTX = 1;
    while (NRF_TWIM0->EVENTS_STOPPED == 0 && NRF_TWIM0->EVENTS_ERROR == 0);
    NRF_TWIM0->EVENTS_STOPPED = 0;
    
    // Now read the data
    NRF_TWIM0->RXD.PTR = (uint32_t)&rx_buffer;
    NRF_TWIM0->RXD.MAXCNT = 1;
    
    NRF_TWIM0->EVENTS_STOPPED = 0;
    NRF_TWIM0->EVENTS_ERROR = 0;
    
    NRF_TWIM0->TASKS_STARTRX = 1;
    while (NRF_TWIM0->EVENTS_STOPPED == 0 && NRF_TWIM0->EVENTS_ERROR == 0);
    
    NRF_TWIM0->TASKS_STOP = 1;
    while (NRF_TWIM0->EVENTS_STOPPED == 0);
    NRF_TWIM0->EVENTS_STOPPED = 0;
    
    return rx_buffer;
}