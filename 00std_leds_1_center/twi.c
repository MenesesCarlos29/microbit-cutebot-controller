#include "twi.h"
#include "nrf_drv_twi.h"

#define TWI_INSTANCE_ID 0
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

void twi_init(void)
{
    const nrf_drv_twi_config_t config = {
        .scl = 27,
        .sda = 26,
        .frequency = NRF_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init = false
    };

    nrf_drv_twi_init(&m_twi, &config, NULL, NULL);
    nrf_drv_twi_enable(&m_twi);
}

void twi_write(uint8_t addr, uint8_t *data, uint8_t len)
{
    nrf_drv_twi_tx(&m_twi, addr, data, len, false);
}

void twi_read(uint8_t addr, uint8_t *data, uint8_t len)
{
    nrf_drv_twi_rx(&m_twi, addr, data, len);
}