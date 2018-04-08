
#include <stdarg.h>

#include "SSD1306.h"
#include "Adafruit_GFX.h"

#include "nrf_drv_twi.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "app_util_platform.h"

#include "arm_math.h"

#include "common.h"

#ifdef SSD1306_SPI
	const nrf_drv_spi_t m_spi_master = NRF_DRV_SPI_INSTANCE(0);
#else
	const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(0);
#endif

#ifdef SSD1306_SPI
	void spi_init (void)
	{
		nrf_drv_spi_config_t spi_config = 
		{                                                            
			.sck_pin      = SSD1306_SPI_D0_PIN,                
			.mosi_pin     = SSD1306_SPI_D1_PIN,                
			.miso_pin     = NRF_DRV_SPI_PIN_NOT_USED,                
			.ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,                
			.irq_priority = APP_IRQ_PRIORITY_LOW,         
			.orc          = 0xFF,                                    
			.frequency    = NRF_DRV_SPI_FREQ_8M,                     
			.mode         = NRF_DRV_SPI_MODE_0,                      
			.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,         
		};

		APP_ERROR_CHECK(nrf_drv_spi_init(&m_spi_master, &spi_config, NULL, NULL));
		
		nrf_gpio_pin_set(SSD1306_SPI_CS_PIN);
		nrf_gpio_pin_set(SSD1306_SPI_DC_PIN);
		nrf_gpio_pin_set(SSD1306_SPI_RES_PIN);
		
		nrf_gpio_cfg_output(SSD1306_SPI_CS_PIN);
		nrf_gpio_cfg_output(SSD1306_SPI_DC_PIN);
		nrf_gpio_cfg_output(SSD1306_SPI_RES_PIN);
	}
#else
	void twi_init (void)
	{
		ret_code_t err_code;

		const nrf_drv_twi_config_t twi_sensors_config = {
		   .scl                = 2,  
		   .sda                = 26,
		   .frequency          = NRF_TWI_FREQ_400K,
		   .interrupt_priority = APP_IRQ_PRIORITY_LOW
		};

		//err_code = nrf_drv_twi_init(&m_twi_lis2dh12, &twi_lis2dh12_config, twi_handler, NULL);
		err_code = nrf_drv_twi_init(&m_twi_master, &twi_sensors_config, NULL, NULL);        // twi in blocking mode.
		APP_ERROR_CHECK(err_code);

		nrf_drv_twi_enable(&m_twi_master);
	}
#endif

void display_init(void)
{
#ifdef SSD1306_SPI
	spi_init();
#else
	twi_init();
#endif
	SSD1306_begin(SSD1306_SWITCHCAPVCC, 0x3C, false);
    Adafruit_GFX_init(SSD1306_LCDWIDTH, SSD1306_LCDHEIGHT, SSD1306_drawPixel);
	Adafruit_GFX_setTextColor(WHITE, BLACK);
	//Adafruit_GFX_setRotation(2);		//upside down
}
	
void display_fft(float32_t *fft_data, float32_t max_filtered, uint8_t scaling)
{
	SSD1306_clearDisplay();
			
	for(uint16_t i = 0; i < SSD1306_LCDWIDTH/scaling; i++)
	{
		for(uint8_t j = 0; j < scaling; j++)
		{
			if((fft_data[i+3] * fft_data[i+3]) > (max_filtered * max_filtered))
			{
				Adafruit_GFX_drawFastVLine(i*scaling+j, SSD1306_LCDHEIGHT, -(int16_t)SSD1306_LCDHEIGHT, WHITE);
			}
			else
			{
				//POWER
				Adafruit_GFX_drawFastVLine(i*scaling+j, SSD1306_LCDHEIGHT, -(int16_t)(fft_data[i+3]*fft_data[i+3]*SSD1306_LCDHEIGHT/(max_filtered*max_filtered)), WHITE);
				//LINEAR
				//Adafruit_GFX_drawFastVLine(i*scaling+j, SSD1306_LCDHEIGHT, -(int16_t)(m_fft_output_f32[i+3]*SSD1306_LCDHEIGHT/max_filtered), WHITE);
			}
		}
	}
	
	SSD1306_display();
}

void display_printf(const char *fmt, ...)
{
	SSD1306_clearDisplay();
    char buf[128], *p;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    for (p = buf; *p; ++p)
		Adafruit_GFX_write(*p);
    va_end(ap);
	SSD1306_display();
}
