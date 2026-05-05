// ex8_tilt_rx.c
//
// Step 2 receiver: listens on the radio, parses (acc_x, acc_y) from the
// 4-byte packet, and prints the values via RTT. No motor control yet,
// that arrives in step 3.
//
// Expected packet layout (must match ex8_tilt_tx.c):
//   pdu[0] = 0       header
//   pdu[1] = 2       payload length
//   pdu[2] = acc_x   signed (-100..+100)
//   pdu[3] = acc_y   signed (-100..+100)

#include "ex8_tilt_rx.h"
#include <nrf.h>
#include <nrf52833.h>
#include <stdint.h>
#include <stdio.h>

// Buffer big enough for the packet (header + length + 2 payload bytes).
// Sized a bit larger to leave room without re-thinking PCNF1.
static uint8_t pdu[8] = { 0 };

// Latest received values (filled in the IRQ, printed in main).
static volatile int8_t   rx_acc_x;
static volatile int8_t   rx_acc_y;
static volatile uint32_t rx_count;
static volatile uint32_t rx_bad_crc;


// ----- Radio RX setup ------------------------------------------------------
// Same configuration as 00std_wireless_rx.c so the link is compatible.

static void radio_rx_init(void) {
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    NRF_RADIO->MODE        = (RADIO_MODE_MODE_Ble_LR125Kbit << RADIO_MODE_MODE_Pos);
    NRF_RADIO->TXPOWER     = (RADIO_TXPOWER_TXPOWER_Pos8dBm << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->PCNF0       = (8 << RADIO_PCNF0_LFLEN_Pos)   |
                             (1 << RADIO_PCNF0_S0LEN_Pos)   |
                             (0 << RADIO_PCNF0_S1LEN_Pos)   |
                             (2 << RADIO_PCNF0_CILEN_Pos)   |
                             (RADIO_PCNF0_PLEN_LongRange << RADIO_PCNF0_PLEN_Pos) |
                             (3 << RADIO_PCNF0_TERMLEN_Pos);
    NRF_RADIO->PCNF1       = (sizeof(pdu) << RADIO_PCNF1_MAXLEN_Pos)  |
                             (0           << RADIO_PCNF1_STATLEN_Pos) |
                             (3           << RADIO_PCNF1_BALEN_Pos)   |
                             (RADIO_PCNF1_ENDIAN_Little    << RADIO_PCNF1_ENDIAN_Pos) |
                             (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos);
    NRF_RADIO->BASE0       = 0xAAAAAAAAUL;
    NRF_RADIO->TXADDRESS   = 0;
    NRF_RADIO->RXADDRESSES = (RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos);
    NRF_RADIO->TIFS        = 0;
    NRF_RADIO->CRCCNF      = (RADIO_CRCCNF_LEN_Three     << RADIO_CRCCNF_LEN_Pos) |
                             (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
    NRF_RADIO->CRCINIT     = 0xFFFFUL;
    NRF_RADIO->CRCPOLY     = 0x00065b;
    NRF_RADIO->FREQUENCY   = 14;
    NRF_RADIO->PACKETPTR   = (uint32_t)pdu;

    // After END (packet received) -> auto-DISABLE -> auto-RXEN, so the
    // radio listens continuously without intervention from main.
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
                        (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos) |
                        (RADIO_SHORTS_DISABLED_RXEN_Enabled << RADIO_SHORTS_DISABLED_RXEN_Pos);

    NRF_RADIO->INTENCLR = 0xffffffff;
    NRF_RADIO->INTENSET = (RADIO_INTENSET_DISABLED_Enabled << RADIO_INTENSET_DISABLED_Pos);
    NVIC_EnableIRQ(RADIO_IRQn);

    NRF_RADIO->TASKS_RXEN = 1;
}

void RADIO_IRQHandler(void) {
    if (NRF_RADIO->EVENTS_DISABLED) {
        NRF_RADIO->EVENTS_DISABLED = 0;

        if (NRF_RADIO->CRCSTATUS != RADIO_CRCSTATUS_CRCSTATUS_CRCOk) {
            rx_bad_crc++;
        } else if (pdu[1] == 2) {
            rx_acc_x = (int8_t)pdu[2];
            rx_acc_y = (int8_t)pdu[3];
            rx_count++;
        }
    }
}


// ----- application ---------------------------------------------------------

void run_tilt_rx(void) {
    radio_rx_init();

    uint32_t last_count = 0;
    uint32_t last_bad   = 0;

    while (1) {
        if (rx_count != last_count) {
            last_count = rx_count;
            printf("rx: acc_x=%4d  acc_y=%4d\n", (int)rx_acc_x, (int)rx_acc_y);
        }
        if (rx_bad_crc != last_bad) {
            last_bad = rx_bad_crc;
            printf("rx: bad CRC (count=%u)\n", (unsigned)last_bad);
        }
        __WFE();
    }
}
