
bool config_menu = false;
bool refresh_menu_display = false;
bool configuring_value = false;
bool apply_config = false;

#define CONFIG_MENU_PAGE_SIZE	4

const char *config_menu_page[CONFIG_MENU_PAGE_SIZE] = 
{
	"Amplitude:    %s",
	"FFT size:     %d",
	"Scale/range:  %d",
	"LED:          %s"
};

#define NUMBER_OF_AMPLITUDE_VALUES		2
#define AMPLITUDE_VALUES_DEFAULT_INDEX	0

const char *amplitude_values[NUMBER_OF_AMPLITUDE_VALUES] = 
{
	"Power",
	"Linear"
};

#define NUMBER_OF_FFT_SIZE_VALUES	4
#define FFT_SIZE_VALUES_DEFAULT_INDEX	0

const uint32_t FFT_size_values[NUMBER_OF_FFT_SIZE_VALUES] = 
{
	512,
	1024,
	2048,
	4096
};

#define NUMBER_OF_SCALE_VALUES	3
#define SCALE_VALUES_DEFAULT_INDEX	2

const uint32_t scale_values[NUMBER_OF_SCALE_VALUES] = 
{
	1,
	2,
	3
};

#define NUMBER_OF_LED_VALUES	3
#define LED_VALUES_DEFAULT_INDEX	2

const char *LED_values[NUMBER_OF_LED_VALUES] = 
{
	"Bar",
	"Dot",
	"All"
};

const uint8_t config_menu_values_max_indexes[CONFIG_MENU_PAGE_SIZE] =
{
	NUMBER_OF_AMPLITUDE_VALUES,
	NUMBER_OF_FFT_SIZE_VALUES,
	NUMBER_OF_SCALE_VALUES,
	NUMBER_OF_LED_VALUES
};

uint8_t config_menu_values_indexes[CONFIG_MENU_PAGE_SIZE] = 
{
	AMPLITUDE_VALUES_DEFAULT_INDEX,
	FFT_SIZE_VALUES_DEFAULT_INDEX,
	SCALE_VALUES_DEFAULT_INDEX,
	LED_VALUES_DEFAULT_INDEX,
};

uint32_t config_menu_values[4];

uint8_t cursor_line = 0;

static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    if(button_action == APP_BUTTON_PUSH)
	{
		if(config_menu == false)
		{
			config_menu = true;
			refresh_menu_display = true;
		}
		else
		{
			switch(pin_no)
			{
				case BUTTON_1:
					if(configuring_value)
					{
						if(config_menu_values_indexes[cursor_line] == (config_menu_values_max_indexes[cursor_line] - 1))
						{
							config_menu_values_indexes[cursor_line] = 0;
						}
						else
						{
							config_menu_values_indexes[cursor_line]++;
						}
					}
					else
					{
						if(cursor_line == 0)
						{
							cursor_line = CONFIG_MENU_PAGE_SIZE - 1;
						}
						else
						{
							cursor_line--;
						}
					}
					refresh_menu_display = true;
					break;
				case BUTTON_3:
					if(configuring_value)
					{
						if(config_menu_values_indexes[cursor_line] == 0)
						{
							config_menu_values_indexes[cursor_line] = config_menu_values_max_indexes[cursor_line] - 1;
						}
						else
						{
							config_menu_values_indexes[cursor_line]--;
						}
					}
					else
					{
						if(cursor_line == (CONFIG_MENU_PAGE_SIZE - 1))
						{
							cursor_line = 0;
						}
						else
						{
							cursor_line++;
						}
					}
					refresh_menu_display = true;
					break;
				case BUTTON_2:
					if(configuring_value)
					{
						configuring_value = false;
					}
					else
					{
						configuring_value = true;
					}
					break;
				case BUTTON_4:
					if(configuring_value)
					{
						configuring_value = false;
					}
					else
					{
						config_menu = false;
						apply_config = true;
					}
					break;
			}
		}
	}
}

if(refresh_menu_display)
		{	
			//refresh config menu values
			config_menu_values[0] = (uint32_t)amplitude_values[config_menu_values_indexes[0]];
			config_menu_values[1] = (uint32_t)FFT_size_values[config_menu_values_indexes[1]];
			config_menu_values[2] = (uint32_t)scale_values[config_menu_values_indexes[2]];
			config_menu_values[3] = (uint32_t)LED_values[config_menu_values_indexes[3]];
			
			SSD1306_clearDisplay();
			Adafruit_GFX_setCursor(0, 0);
			Adafruit_GFX_setTextColor(WHITE, BLACK);
			display_printf("Config menu:\n");
			
			for(uint8_t i = 0; i < CONFIG_MENU_PAGE_SIZE; i++)
			{
				if(i == cursor_line)
				{
					Adafruit_GFX_setTextColor(BLACK, WHITE);
				}
				else
				{
					Adafruit_GFX_setTextColor(WHITE, BLACK);
				}
				display_printf(config_menu_page[i], config_menu_values[i]);
				display_printf("\n");
			}
			SSD1306_display();
			refresh_menu_display = false;
		}
		if(apply_config)
		{
			nrf_drv_i2s_stop();
			
			err_code = nrf_drv_i2s_start((uint32_t *)m_buffer_rx, (uint32_t *)m_buffer_rx, FFT_size_values[config_menu_values_indexes[1]]*2, 0);
			APP_ERROR_CHECK(err_code);
		
			scaling = scale_values[config_menu_values_indexes[2]];
			
			apply_config = false;
		}