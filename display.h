
#ifndef DISPLAY_H
#define DISPLAY_H

#include "arm_math.h"

void display_init(void);
void display_printf(const char *fmt, ...);
void display_fft(float32_t *fft_data, float32_t max_filtered, uint8_t scaling);

#endif //DISPLAY_H
