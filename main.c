/**
 * Copyright (c) 2009 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
* @brief Example template project.
* @defgroup nrf_templates_example Example Template
*
*/

#include <stdbool.h>
#include <string.h>
#include <stdio.h>


#include "nrf.h"
#include "nordic_common.h"
#include "app_util_platform.h"
#include "boards.h"
#include "nrf_drv_common.h"
#include "nrf_delay.h"
#include "nrf_drv_i2s.h"
#include "nrf_gpio.h"
#include "app_timer.h"
#include "app_button.h"

#include "arm_const_structs.h"

#include "common.h"
#include "nrf_drv_WS2812.h"
#include "audio_in.h"
#include "radio.h"
#include "display.h"

#define FPU_EXCEPTION_MASK               0x0000009F                      //!< FPU exception mask used to clear exceptions in FPSCR register.
#define FPU_FPSCR_REG_STACK_OFF          0x40                            //!< Offset of FPSCR register stacked during interrupt handling in FPU part stack.

static float32_t m_fft_input_f32[FFT_TEST_COMP_SAMPLES_LEN];             //!< FFT input array. Time domain.
static float32_t m_fft_output_f32[FFT_TEST_OUT_SAMPLES_LEN];             //!< FFT output data. Frequency domain.

static uint32_t  m_ifft_flag             = 0;                            //!< Flag that selects forward (0) or inverse (1) transform.
static uint32_t  m_do_bit_reverse        = 1;                            //!< Flag that enables (1) or disables (0) bit reversal of output.

static bool fft_data_ready = false;

#define MAX_FILTER_ALPHA_UP		0.05f	//higher value -> less filtering (trust in current measurement more)
#define MAX_FILTER_ALPHA_DOWN	0.2f

#define MAX_VS_AVG_WEIGHT		0.5f	//0 - 1. Higher value will give more weight on max value vs average value. Only for LED display

float32_t max_value;
float32_t max_filtered = 1.0f;
uint32_t  max_val_index;

uint8_t scaling = 2;

void update_fft_input_i32(int32_t *p_data)
{
	for(uint16_t i = 0; i < FFT_TEST_COMP_SAMPLES_LEN/2; i++)
	{
		m_fft_input_f32[i*2] = (float)p_data[i];
		m_fft_input_f32[i*2+1] = 0;
	}
	fft_data_ready = true;
}

void update_fft_input_i16(int16_t *p_data)
{
	for(uint16_t i = 0; i < FFT_TEST_COMP_SAMPLES_LEN/2; i++)
	{
		m_fft_input_f32[i*2] = (float)p_data[i];
		m_fft_input_f32[i*2+1] = 0;
	}
	fft_data_ready = true;
}

void update_led_strip(nrf_drv_WS2812_pixel_t *new_pixel)
{
    static nrf_drv_WS2812_pixel_t strip[(NR_OF_PIXELS+ 1)/2];
    //shift all pixels

    if((NR_OF_PIXELS % 2) == 0)
    {
        for(uint8_t i = (NR_OF_PIXELS/2 - 1); i > 0; i--)
        {
            memcpy(&strip[i], &strip[i-1], sizeof(nrf_drv_WS2812_pixel_t));
            nrf_drv_WS2812_set_pixel(NR_OF_PIXELS/2 + i, &strip[i]);
            nrf_drv_WS2812_set_pixel(NR_OF_PIXELS/2 - 1 - i, &strip[i]);
        }

        memcpy(&strip[0], new_pixel, sizeof(nrf_drv_WS2812_pixel_t));
        nrf_drv_WS2812_set_pixel(NR_OF_PIXELS/2, &strip[0]);
        nrf_drv_WS2812_set_pixel(NR_OF_PIXELS/2 - 1, &strip[0]);
    }
    else
    {
        for(uint8_t i = ((NR_OF_PIXELS + 1)/2 - 1); i > 0; i--)
        {
            memcpy(&strip[i], &strip[i-1], sizeof(nrf_drv_WS2812_pixel_t));
            nrf_drv_WS2812_set_pixel(NR_OF_PIXELS/2 + i, &strip[i]);
            nrf_drv_WS2812_set_pixel(NR_OF_PIXELS/2 - i, &strip[i]);
        }

        memcpy(&strip[0], new_pixel, sizeof(nrf_drv_WS2812_pixel_t));
        nrf_drv_WS2812_set_pixel(NR_OF_PIXELS/2, &strip[0]);
    }
    
    nrf_drv_WS2812_show();
}

static void fft_process(float32_t *                   p_input,
                        const arm_cfft_instance_f32 * p_input_struct,
                        float32_t *                   p_output,
                        uint16_t                      output_size)
{
    // Use CFFT module to process the data.
    arm_cfft_f32(p_input_struct, p_input, m_ifft_flag, m_do_bit_reverse);
    // Calculate the magnitude at each bin using Complex Magnitude Module function.
    arm_cmplx_mag_f32(p_input, p_output, output_size);
}

void FPU_IRQHandler(void)
{
    // Prepare pointer to stack address with pushed FPSCR register.
    uint32_t * fpscr = (uint32_t * )(FPU->FPCAR + FPU_FPSCR_REG_STACK_OFF);
    // Execute FPU instruction to activate lazy stacking.
    (void)__get_FPSCR();
    // Clear flags in stacked FPSCR register.
    *fpscr = *fpscr & ~(FPU_EXCEPTION_MASK);
}

nrf_drv_WS2812_pixel_t hsv_to_rgb(float hue, float saturation, float value)
{
	nrf_drv_WS2812_pixel_t out;
	float hh = hue * 6.0f;
	uint32_t i = (uint32_t)hh;
	
	float ff = hh - i;
	
	float p = value * (1.0f - saturation);
    float q = value * (1.0f - (saturation * ff));
    float t = value * (1.0f - (saturation * (1.0f - ff)));
	
    switch(i) {
    case 0:
        out.red = (uint8_t)(value * 255);
        out.green = (uint8_t)(t * 255);
        out.blue = (uint8_t)(p * 255);
        break;
    case 1:
        out.red = (uint8_t)(q * 255);
        out.green = (uint8_t)(value * 255);
        out.blue = (uint8_t)(p * 255);
        break;
    case 2:
        out.red = (uint8_t)(p * 255);
        out.green = (uint8_t)(value * 255);
        out.blue = (uint8_t)(t * 255);
        break;

    case 3:
        out.red = (uint8_t)(p * 255);
        out.green = (uint8_t)(q * 255);
        out.blue = (uint8_t)(value * 255);
        break;
    case 4:
        out.red = (uint8_t)(t * 255); 
        out.green = (uint8_t)(p * 255);
        out.blue = (uint8_t)(value * 255);
        break;
    case 5:
    default:
        out.red = (uint8_t)(value * 255);
        out.green = (uint8_t)(p * 255);
        out.blue = (uint8_t)(q * 255);
        break;
    }
    return out;
}

void lfclk_start(void)
{
	NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }
}

static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    if(button_action == APP_BUTTON_PUSH)
	{
		switch(pin_no)
		{
			case BUTTON_1:
				
				break;
			case BUTTON_2:
				scaling++;
				if(scaling > 3)
				{
				scaling = 1;
				}
				break;
			case BUTTON_3:
				
				break;
			case BUTTON_4:
				
				break;
		}
	}
}

static const app_button_cfg_t app_buttons[4] = 
{
    {BUTTON_1, false, BUTTON_PULL, button_event_handler},
    {BUTTON_2, false, BUTTON_PULL, button_event_handler},
	{BUTTON_3, false, BUTTON_PULL, button_event_handler},
    {BUTTON_4, false, BUTTON_PULL, button_event_handler},
};


void buttons_init(void)
{
	ret_code_t err_code;
	
	lfclk_start();
	
	//app_button uses app_timer, if this is not initialize, then initialize it here
    app_timer_init();

    //init app_button module, 50ms detection delay (button debouncing)
    err_code = app_button_init((app_button_cfg_t *)app_buttons,
                                   4,
                                   APP_TIMER_TICKS(10));
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}

void leds_init(void)
{
	nrf_gpio_cfg_output(17);
	nrf_gpio_cfg_output(18);
	nrf_gpio_cfg_output(19);
	nrf_gpio_cfg_output(20);
	
	nrf_gpio_pin_set(17);
	nrf_gpio_pin_set(18);
	nrf_gpio_pin_set(19);
	nrf_gpio_pin_set(20);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{   	
	leds_init();
	
	radio_init();
	audio_in_init();
	
	display_init();
	
	nrf_drv_WS2812_init(WS2812_PIN);
	
	buttons_init();
	
    // Enable FPU interrupt
    NVIC_SetPriority(FPU_IRQn, APP_IRQ_PRIORITY_LOWEST);
    NVIC_ClearPendingIRQ(FPU_IRQn);
    NVIC_EnableIRQ(FPU_IRQn);
	
	display_printf("FFT microphone\nvisualizer");
    
	nrf_delay_ms(2000);
	
    while (true)
    {
		if(fft_data_ready)
		{
			//for measuring time used
			nrf_gpio_pin_set(17);
			
			//1024 numbers in input array (512 complex pairs of samples) -> 512 output bins power data -> &arm_cfft_sR_f32_len64.
			fft_process(m_fft_input_f32, &arm_cfft_sR_f32_len512, m_fft_output_f32, FFT_TEST_OUT_SAMPLES_LEN);
			
			//arm_max_f32(&m_fft_output_f32[1], FFT_TEST_OUT_SAMPLES_LEN/2 - 1, &max_value, &max_val_index);
			arm_max_f32(&m_fft_output_f32[3], SSD1306_LCDWIDTH/scaling, &max_value, &max_val_index);
			
			if(max_value > max_filtered)
			{
				max_filtered = max_filtered * (1.0f - MAX_FILTER_ALPHA_UP) + max_value * MAX_FILTER_ALPHA_UP;
			}
			else
			{
				max_filtered = max_filtered * (1.0f - MAX_FILTER_ALPHA_DOWN) + max_value * MAX_FILTER_ALPHA_DOWN;
			}
				
			display_fft(m_fft_output_f32, max_filtered, scaling);
			
			static nrf_drv_WS2812_pixel_t pixel;
			static uint8_t counter;
			static float max_red, max_green, max_blue;

			//divide the frequencies equally in three bulks [R, G, B]
			//arm_max_f32(&m_fft_output_f32[3], SSD1306_LCDWIDTH/scaling/3, &max_red, &max_val_index);
			//arm_max_f32(&m_fft_output_f32[3 + SSD1306_LCDWIDTH*2/scaling/3], SSD1306_LCDWIDTH/scaling/3, &max_green, &max_val_index);
			//arm_max_f32(&m_fft_output_f32[3 + SSD1306_LCDWIDTH/scaling], SSD1306_LCDWIDTH/scaling/3, &max_blue, &max_val_index);
			
			//divide the frequencies skewed like [R, G, G, B, B, B], R = 1/6, G = 2/6, B = 3/6
			//arm_max_f32(&m_fft_output_f32[3], SSD1306_LCDWIDTH/scaling/6, &max_red, &max_val_index);
			//arm_max_f32(&m_fft_output_f32[3 + SSD1306_LCDWIDTH/scaling/6], SSD1306_LCDWIDTH/scaling/3, &max_green, &max_val_index);
			//arm_max_f32(&m_fft_output_f32[3 + SSD1306_LCDWIDTH/scaling/2], SSD1306_LCDWIDTH/scaling/2, &max_blue, &max_val_index);

			//divide the frequencies skewed like [R, G, g, B, B], R = 2/9, G = 3/9, B = 4/9
			arm_max_f32(&m_fft_output_f32[3], SSD1306_LCDWIDTH*2/scaling/9, &max_red, &max_val_index);
			arm_max_f32(&m_fft_output_f32[3 + SSD1306_LCDWIDTH*2/scaling/9], SSD1306_LCDWIDTH/scaling/3, &max_green, &max_val_index);
			arm_max_f32(&m_fft_output_f32[3 + SSD1306_LCDWIDTH*5/scaling/9], SSD1306_LCDWIDTH*4/scaling/9, &max_blue, &max_val_index);
			
			counter++;
			if((counter % 1) == 0)
			{
				pixel.red = (uint8_t)(fmin(1.0f, max_red * max_red / (max_filtered * max_filtered))*255);
				pixel.green = (uint8_t)(fmin(1.0f, max_green * max_green / (max_filtered * max_filtered))*255);
				pixel.blue = (uint8_t)(fmin(1.0f, max_blue * max_blue / (max_filtered * max_filtered))*255);
				update_led_strip(&pixel);

				//send data to remote LED strip
				radio_send_data(&pixel);
				
				//update local LED strip
				//update_led_strip(&pixel);
			}

			//for measuring time used
			nrf_gpio_pin_clear(17);
			
			//ready for new data
			fft_data_ready = false;
		}
    }
}
/** @} */
