#include "ex8_transmitter.h"
#include <nrf.h>
#include "nrf52833.h"

#define PIN_SCL_INT 8
#define PIN_SDA_INT 16

static uint8_t pdu[] = { 0x00, 2, 0x00, 0x00 };

void accel_i2c_init(void) {
    NRF_P0->PIN_CNF[PIN_SCL_INT] = 0x00000602;
    NRF_P0->PIN_CNF[PIN_SDA_INT] = 0x00000602;

    NRF_TWI1->PSEL.SCL  = PIN_SCL_INT;
    NRF_TWI1->PSEL.SDA  = PIN_SDA_INT;
    NRF_TWI1->FREQUENCY = 0x01980000;      
    NRF_TWI1->ADDRESS   = 0x19;            
    NRF_TWI1->ENABLE    = 0x00000005;
}

void accel_write_reg(uint8_t reg, uint8_t value) {
    NRF_TWI1->EVENTS_TXDSENT = 0; NRF_TWI1->EVENTS_STOPPED = 0;
    NRF_TWI1->TASKS_STARTTX  = 1;
    NRF_TWI1->TXD = reg;   while (NRF_TWI1->EVENTS_TXDSENT == 0); NRF_TWI1->EVENTS_TXDSENT = 0;
    NRF_TWI1->TXD = value; while (NRF_TWI1->EVENTS_TXDSENT == 0); NRF_TWI1->EVENTS_TXDSENT = 0;
    NRF_TWI1->TASKS_STOP = 1; while (NRF_TWI1->EVENTS_STOPPED == 0); NRF_TWI1->EVENTS_STOPPED = 0;
}

uint8_t accel_read_reg(uint8_t reg) {
    uint8_t value;
    NRF_TWI1->EVENTS_TXDSENT = 0; NRF_TWI1->EVENTS_STOPPED = 0;
    NRF_TWI1->TASKS_STARTTX  = 1;
    NRF_TWI1->TXD = reg; while (NRF_TWI1->EVENTS_TXDSENT == 0); NRF_TWI1->EVENTS_TXDSENT = 0;

    NRF_TWI1->SHORTS = 0x00000002;     
    NRF_TWI1->EVENTS_RXDREADY = 0;
    NRF_TWI1->TASKS_STARTRX   = 1;
    while (NRF_TWI1->EVENTS_RXDREADY == 0);
    value = NRF_TWI1->RXD;

    while (NRF_TWI1->EVENTS_STOPPED == 0);
    NRF_TWI1->EVENTS_STOPPED  = 0;
    NRF_TWI1->EVENTS_RXDREADY = 0; NRF_TWI1->SHORTS = 0;
    return value;
}

int8_t scale_accel(int16_t raw) {
    int v = raw / 112;
    if (v < -100) return -100;
    if (v > 100) return 100;
    return (int8_t)v;
}

// --- RADIO ---
void radio_tx_init(void) {
    NRF_CLOCK->TASKS_HFCLKSTART = 1; while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
    NRF_RADIO->MODE        = (RADIO_MODE_MODE_Ble_LR125Kbit << RADIO_MODE_MODE_Pos);
    NRF_RADIO->TXPOWER     = (RADIO_TXPOWER_TXPOWER_Pos8dBm << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->PCNF0       = (8 << RADIO_PCNF0_LFLEN_Pos) | (1 << RADIO_PCNF0_S0LEN_Pos) | (0 << RADIO_PCNF0_S1LEN_Pos) | (2 << RADIO_PCNF0_CILEN_Pos) | (RADIO_PCNF0_PLEN_LongRange << RADIO_PCNF0_PLEN_Pos) | (3 << RADIO_PCNF0_TERMLEN_Pos);
    NRF_RADIO->PCNF1       = (sizeof(pdu) << RADIO_PCNF1_MAXLEN_Pos) | (0 << RADIO_PCNF1_STATLEN_Pos) | (3 << RADIO_PCNF1_BALEN_Pos) | (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) | (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos);
    NRF_RADIO->BASE0       = 0xAAAAAAAAUL; NRF_RADIO->TXADDRESS = 0; NRF_RADIO->RXADDRESSES = (RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos);
    NRF_RADIO->TIFS        = 1000;
    NRF_RADIO->CRCCNF      = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) | (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
    NRF_RADIO->CRCINIT     = 0xFFFFUL; NRF_RADIO->CRCPOLY = 0x00065b; NRF_RADIO->FREQUENCY = 14;
    NRF_RADIO->PACKETPTR   = (uint32_t)pdu;
    NRF_RADIO->SHORTS      = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) | (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);
}

void run_ex8_transmitter(void) {
    accel_i2c_init();
    accel_write_reg(0x20, 0x57); 
    radio_tx_init();

    while (1) {
        uint8_t xl = accel_read_reg(0x28); uint8_t xh = accel_read_reg(0x29);
        uint8_t yl = accel_read_reg(0x2A); uint8_t yh = accel_read_reg(0x2B);

        int16_t accel_x = (int16_t)((xh << 8) | xl);
        int16_t accel_y = (int16_t)((yh << 8) | yl);

        pdu[2] = (uint8_t)scale_accel(accel_x);
        pdu[3] = (uint8_t)scale_accel(accel_y);

        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_TXEN = 1;
        while (NRF_RADIO->EVENTS_DISABLED == 0);

        for (volatile int i = 0; i < 50000; i++); 
    }
}