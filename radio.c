
#include <string.h>

#include "nrf_esb.h"
#include "nrf_esb_error_codes.h"
#include "app_util_platform.h"

#include "nrf_drv_WS2812.h"

nrf_esb_payload_t   tx_payload;

void hfclk_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            (void) nrf_esb_flush_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            break;
    }
}

void radio_init(void)
{  
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE6, 0xE6, 0xE6, 0xE6};
    uint8_t base_addr_1[4] = {0xB2, 0xB2, 0xB2, 0xB2};
    uint8_t addr_prefix[8] = {0xE6, 0xA8, 0xB7, 0xC6, 0xD5, 0xE4, 0xF3, 0x92 };
    
    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.tx_output_power          = NRF_ESB_TX_POWER_4DBM;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.retransmit_count         = 4;
    nrf_esb_config.retransmit_delay         = 250;  //In microseconds. The minimum delay time of 135 lead to hangup
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_1MBPS_BLE;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.selective_auto_ack       = false;
    nrf_esb_config.payload_length           = sizeof(nrf_drv_WS2812_pixel_t);

	hfclk_start();
	
    err_code = nrf_esb_init(&nrf_esb_config);

    APP_ERROR_CHECK(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_esb_set_rf_channel(81);
    APP_ERROR_CHECK(err_code);
    
    tx_payload.length = sizeof(nrf_drv_WS2812_pixel_t);
    tx_payload.pipe = 0;
    tx_payload.noack = true;
}

void radio_send_data(nrf_drv_WS2812_pixel_t *p_data)
{
	memcpy(tx_payload.data, p_data, sizeof(nrf_drv_WS2812_pixel_t));
	nrf_esb_write_payload(&tx_payload);
}
