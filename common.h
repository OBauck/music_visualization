
#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include "SSD1306.h"

void update_fft_input_i32(int32_t *p_data);
void update_fft_input_i16(int16_t *p_data);

#ifdef SSD1306_SPI
	#define SSD1306_SPI_D0_PIN		27
	#define SSD1306_SPI_D1_PIN		26
	#define SSD1306_SPI_CS_PIN		25
	#define SSD1306_SPI_DC_PIN		24
	#define SSD1306_SPI_RES_PIN		2
#else
	#define	SSD1306_TWI_SCL_PIN		2
	#define SSD1306_TWI_SCK_PIN		26
#endif

#define I2S_SEL_PIN		22
#define I2S_SDIN_PIN	23
#define I2S_LRCK_PIN	11
#define I2S_BCLK_PIN 	3
#define PWM_BCLK_PIN	4
#define PWM_LRCK_PIN	12
//using PWM to create the i2s clock because incompatability with 24 bit i2s mic, see here: https://devzone.nordicsemi.com/f/nordic-q-a/15713/i2s-32-bit-word-size/59992#59992

#define ADC_IN_PIN		NRF_SAADC_INPUT_AIN5	//P0.29
#define ADC_NC_PIN		NRF_SAADC_INPUT_AIN6	//P0.30

#define WS2812_PIN		31

#define FFT_TEST_COMP_SAMPLES_LEN        1024
#define FFT_TEST_OUT_SAMPLES_LEN         (FFT_TEST_COMP_SAMPLES_LEN / 2) 

#define FFT_SAMPLES_USED				

#endif //COMMON_H
