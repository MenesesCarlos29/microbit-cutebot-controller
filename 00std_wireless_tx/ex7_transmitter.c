#include <nrf.h>
#include "nrf52833.h"

#define PIN_BUTTON_A 14
#define PIN_BUTTON_B 23


// --- RADIO PACKET (PDU) ---
// Format: {Header, Length (1 byte), Command}
// Command: 0 = Stop, 1 = Forward, 2 = Backward
static uint8_t pdu[] = { 0x00, 1, 0x00 };

void buttons_init(void) {
    NRF_P0->PIN_CNF[PIN_BUTTON_A] = (0 << 0) | (3 << 2);
    NRF_P0->PIN_CNF[PIN_BUTTON_B] = (0 << 0) | (3 << 2);
}

int is_btn_A(void) { return ((NRF_P0->IN & (1 << PIN_BUTTON_A)) == 0); }

int is_btn_B(void) { return ((NRF_P0->IN & (1 << PIN_BUTTON_B)) == 0); }

void run_ex7_transmitter(void) {

    buttons_init();

    // Set Clock
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}

    // Set radio configuration
    NRF_RADIO->MODE          = (RADIO_MODE_MODE_Ble_LR125Kbit << RADIO_MODE_MODE_Pos);
    NRF_RADIO->TXPOWER       = (RADIO_TXPOWER_TXPOWER_Pos8dBm << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->PCNF0         = (8 << RADIO_PCNF0_LFLEN_Pos) | (1 << RADIO_PCNF0_S0LEN_Pos) | (0 << RADIO_PCNF0_S1LEN_Pos) | (2 << RADIO_PCNF0_CILEN_Pos) | (RADIO_PCNF0_PLEN_LongRange << RADIO_PCNF0_PLEN_Pos) | (3 << RADIO_PCNF0_TERMLEN_Pos);
    NRF_RADIO->PCNF1         = (sizeof(pdu) << RADIO_PCNF1_MAXLEN_Pos) | (0 << RADIO_PCNF1_STATLEN_Pos) | (3 << RADIO_PCNF1_BALEN_Pos) | (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) | (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos);
    NRF_RADIO->BASE0         = 0xAAAAAAAAUL;
    NRF_RADIO->TXADDRESS     = 0UL;
    NRF_RADIO->RXADDRESSES   = (RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos);
    NRF_RADIO->TIFS          = 1000U;
    NRF_RADIO->CRCCNF        = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) | (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
    NRF_RADIO->CRCINIT       = 0xFFFFUL;
    NRF_RADIO->CRCPOLY       = 0x00065b; 
    NRF_RADIO->FREQUENCY     = 14;
    NRF_RADIO->PACKETPTR     = (uint32_t)pdu;

    NRF_RADIO->INTENCLR = 0xffffffff;
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) | (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);
    NRF_RADIO->INTENSET = (RADIO_INTENSET_DISABLED_Enabled << RADIO_INTENSET_DISABLED_Pos);
    NVIC_EnableIRQ(RADIO_IRQn);

    
    while(1) {
        if (is_btn_A()) {
            pdu[2] = 1;
        } else if (is_btn_B()) {
            pdu[2] = 2;
        } else {
            pdu[2] = 0;
        }

        // send radio packet
        NRF_RADIO->TASKS_TXEN = (RADIO_TASKS_TXEN_TASKS_TXEN_Trigger << RADIO_TASKS_TXEN_TASKS_TXEN_Pos);
        while (NRF_RADIO->EVENTS_DISABLED != 0) {}
        
        // Simple delay to avoid sending too many packets when a button is held down
        uint32_t wait = 0x000fffff;
        while (wait--);
    }
}