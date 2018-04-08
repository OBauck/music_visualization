
#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>
#include "nrf_drv_WS2812.h"

void radio_init(void);
void radio_send_data(nrf_drv_WS2812_pixel_t *p_data);

#endif //RADIO_H
