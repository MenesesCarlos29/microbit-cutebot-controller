#include "ex8_receiver.h"
#include <nrf.h>
#include "nrf52833.h"

void i2c_init(void) {
    NRF_P0->PIN_CNF[26] = 0x00000602; 
    NRF_P1->PIN_CNF[0]  = 0x00000602; 
    NRF_TWI0->ENABLE     = 0x00000005; 
    NRF_TWI0->PSEL.SCL   = 0x0000001a; 
    NRF_TWI0->PSEL.SDA   = 0x00000020; 
    NRF_TWI0->FREQUENCY  = 0x01980000; 
    NRF_TWI0->ADDRESS    = 0x10;       
}

void i2c_send(uint8_t* buf, uint8_t buflen) {
    uint8_t i = 0; NRF_TWI0->TXD = buf[i++];
    NRF_TWI0->EVENTS_TXDSENT = 0; NRF_TWI0->TASKS_STARTTX  = 1;
    while(i < buflen) {
        while(NRF_TWI0->EVENTS_TXDSENT == 0);
        NRF_TWI0->EVENTS_TXDSENT = 0; NRF_TWI0->TXD = buf[i++];
    }
    while(NRF_TWI0->EVENTS_TXDSENT == 0); NRF_TWI0->TASKS_STOP = 1;
}

void set_motors(int left_speed, int right_speed) {
    uint8_t dir_l = (left_speed >= 0) ? 1 : 0;
    uint8_t dir_r = (right_speed >= 0) ? 1 : 0;
    
    if (left_speed < 0) left_speed = -left_speed;
    if (right_speed < 0) right_speed = -right_speed;
    
    if (left_speed > 100) left_speed = 100;
    if (right_speed > 100) right_speed = 100;

    uint8_t buf_left[]  = {0x99, 0x01, 0x01, dir_l, (uint8_t)left_speed, 0x00, 0x88};
    uint8_t buf_right[] = {0x99, 0x01, 0x02, dir_r, (uint8_t)right_speed, 0x00, 0x88};

    i2c_send(buf_left, sizeof(buf_left));
    i2c_send(buf_right, sizeof(buf_right));
}

// --- RADIO RECEIVER ---
static uint8_t pdu[8+1] = { 0 };
volatile int8_t rx_tilt_x = 0;
volatile int8_t rx_tilt_y = 0;

void RADIO_IRQHandler(void) {
    if (NRF_RADIO->EVENTS_DISABLED) {
        NRF_RADIO->EVENTS_DISABLED = 0;
        if (NRF_RADIO->CRCSTATUS == RADIO_CRCSTATUS_CRCSTATUS_CRCOk) {
            rx_tilt_x = (int8_t)pdu[2];
            rx_tilt_y = (int8_t)pdu[3];
        }
    }
}

void run_ex8_receiver(void) {
    i2c_init();
    set_motors(0, 0); 

    NRF_CLOCK->TASKS_HFCLKSTART = 1; while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    NRF_RADIO->MODE        = (RADIO_MODE_MODE_Ble_LR125Kbit << RADIO_MODE_MODE_Pos);
    NRF_RADIO->TXPOWER     = (RADIO_TXPOWER_TXPOWER_Pos8dBm << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->PCNF0       = (8 << RADIO_PCNF0_LFLEN_Pos) | (1 << RADIO_PCNF0_S0LEN_Pos) | (0 << RADIO_PCNF0_S1LEN_Pos) | (2 << RADIO_PCNF0_CILEN_Pos) | (RADIO_PCNF0_PLEN_LongRange << RADIO_PCNF0_PLEN_Pos) | (3 << RADIO_PCNF0_TERMLEN_Pos);
    NRF_RADIO->PCNF1       = (sizeof(pdu) << RADIO_PCNF1_MAXLEN_Pos) | (0 << RADIO_PCNF1_STATLEN_Pos) | (3 << RADIO_PCNF1_BALEN_Pos) | (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) | (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos);
    NRF_RADIO->BASE0       = 0xAAAAAAAAUL; NRF_RADIO->TXADDRESS = 0; NRF_RADIO->RXADDRESSES = (RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos);
    NRF_RADIO->TIFS        = 0;
    NRF_RADIO->CRCCNF      = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) | (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
    NRF_RADIO->CRCINIT     = 0xFFFFUL; NRF_RADIO->CRCPOLY = 0x00065b; NRF_RADIO->FREQUENCY = 14;
    NRF_RADIO->PACKETPTR   = (uint32_t)pdu;

    NRF_RADIO->SHORTS      = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) | (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos) | (RADIO_SHORTS_DISABLED_RXEN_Enabled << RADIO_SHORTS_DISABLED_RXEN_Pos);
    NRF_RADIO->TASKS_RXEN  = 1;

    NRF_RADIO->INTENCLR = 0xffffffff;
    NVIC_EnableIRQ(RADIO_IRQn);
    NRF_RADIO->INTENSET = (RADIO_INTENSET_DISABLED_Enabled << RADIO_INTENSET_DISABLED_Pos);

    while(1) {
        int left  = rx_tilt_y + rx_tilt_x;
        int right = rx_tilt_y - rx_tilt_x;

        set_motors(left, right);
        
        for (volatile int i = 0; i < 50000; i++); // Pequeño delay de estabilización
    }
}