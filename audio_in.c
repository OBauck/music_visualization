
#include "nrf_gpio.h"
#include "nrf_drv_i2s.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_common.h"
#include "nrf_saadc.h"
#include "app_util_platform.h"

#include "common.h"

#define PWM_BCLK_PERIOD		6		//this is equal to 16MHz/6 = 2.67MHz, which equals a sampling frequency of 41.67 KHz

#define I2S_BUFFER_SIZE     FFT_TEST_COMP_SAMPLES_LEN
static int32_t m_buffer_rx[I2S_BUFFER_SIZE];

static uint16_t pwm_seq0 = PWM_BCLK_PERIOD/2;
static uint16_t pwm_seq1 = PWM_BCLK_PERIOD*64/2;

bool buffer_1_in_use = true;
bool saadc_end_event = false;
int16_t saadc_buffer_1[FFT_TEST_COMP_SAMPLES_LEN];
int16_t saadc_buffer_2[FFT_TEST_COMP_SAMPLES_LEN];

const nrf_drv_timer_t timer0 = NRF_DRV_TIMER_INSTANCE(0);

// This is the I2S data handler - all data exchange related to the I2S transfers
// is done here.
static void data_handler(uint32_t const * p_data_received,
                         uint32_t       * p_data_to_send,
                         uint16_t         number_of_words)
{
    // Non-NULL value in 'p_data_received' indicates that a new portion of
    // data has been received and should be processed.
    if (p_data_received != NULL)
    {
		update_fft_input_i32((int32_t *)p_data_received);
    }
}

void pwm_init(void)
{
	NRF_PWM1->PSEL.OUT[0] = (PWM_BCLK_PIN << PWM_PSEL_OUT_PIN_Pos) | 
                            (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
	
    NRF_PWM1->ENABLE      = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
    NRF_PWM1->MODE        = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    NRF_PWM1->PRESCALER   = (PWM_PRESCALER_PRESCALER_DIV_1 << PWM_PRESCALER_PRESCALER_Pos);
    NRF_PWM1->COUNTERTOP  = (6 << PWM_COUNTERTOP_COUNTERTOP_Pos);
    NRF_PWM1->DECODER   = (PWM_DECODER_LOAD_Common << PWM_DECODER_LOAD_Pos) | 
                          (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
	
	NRF_PWM1->SEQ[1].PTR  = ((uint32_t)(&pwm_seq0) << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM1->SEQ[1].CNT  = ((sizeof(pwm_seq0) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
    NRF_PWM1->SEQ[1].REFRESH  = 0;
    NRF_PWM1->SEQ[1].ENDDELAY = 0;
	
	NRF_PWM2->PSEL.OUT[0] = (PWM_LRCK_PIN << PWM_PSEL_OUT_PIN_Pos) | 
                            (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
	
	NRF_PWM2->ENABLE      = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
    NRF_PWM2->MODE        = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    NRF_PWM2->PRESCALER   = (PWM_PRESCALER_PRESCALER_DIV_1 << PWM_PRESCALER_PRESCALER_Pos);
    NRF_PWM2->COUNTERTOP  = (PWM_BCLK_PERIOD*64 << PWM_COUNTERTOP_COUNTERTOP_Pos);
    NRF_PWM2->DECODER   = (PWM_DECODER_LOAD_Common << PWM_DECODER_LOAD_Pos) | 
                          (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
	
	NRF_PWM2->SEQ[1].PTR  = ((uint32_t)(&pwm_seq1) << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM2->SEQ[1].CNT  = ((sizeof(pwm_seq1) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
    NRF_PWM2->SEQ[1].REFRESH  = 0;
    NRF_PWM2->SEQ[1].ENDDELAY = 0;
}

void pwm_start(void)
{
	NRF_PWM1->TASKS_SEQSTART[1] = 1;
	NRF_PWM2->TASKS_SEQSTART[1] = 1;
}

void i2s_init(void)
{
	ret_code_t err_code;
	
	nrf_gpio_cfg_output(I2S_SEL_PIN);
	nrf_gpio_pin_clear(I2S_SEL_PIN);
	
	nrf_drv_i2s_config_t config = NRF_DRV_I2S_DEFAULT_CONFIG;

    config.sdin_pin  	= I2S_SDIN_PIN;
    config.sdout_pin 	= NRF_DRV_I2S_PIN_NOT_USED;
	config.lrck_pin  	= I2S_LRCK_PIN;
	config.mck_pin   	= NRF_DRV_I2S_PIN_NOT_USED;
	config.sck_pin   	= I2S_BCLK_PIN;
	config.mode         = NRF_I2S_MODE_SLAVE;
	config.mck_setup    = NRF_I2S_MCK_DISABLED;
	config.sample_width	= NRF_I2S_SWIDTH_24BIT;
    config.channels  	= NRF_I2S_CHANNELS_LEFT;
	config.format       = NRF_I2S_FORMAT_I2S;
    err_code = nrf_drv_i2s_init(&config, data_handler);
    APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_i2s_start((uint32_t *)m_buffer_rx, (uint32_t *)m_buffer_rx, I2S_BUFFER_SIZE, 0);
	APP_ERROR_CHECK(err_code);
	
	pwm_init();
	pwm_start();
}

//SAADC (line in)
// Timer even handler. Not used since timer is used only for PPI.
void timer_event_handler(nrf_timer_event_t event_type, void * p_context){}

void saadc_timer_sample_ppi_init(void)
{
	static nrf_ppi_channel_t     m_ppi_channel_timer_cc_sample;
	
	nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_16MHz;
	
    ret_code_t err_code = nrf_drv_timer_init(&timer0, &timer_cfg, timer_event_handler);
    APP_ERROR_CHECK(err_code);
	
	//Compare event every 384 timer cycles (41.67kHz * 2)
	nrf_drv_timer_extended_compare(&timer0, NRF_TIMER_CC_CHANNEL0, 384, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
	
	err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel_timer_cc_sample);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel_timer_cc_sample,
                                          nrf_drv_timer_event_address_get(&timer0, NRF_TIMER_EVENT_COMPARE0),
                                          nrf_saadc_task_address_get(NRF_SAADC_TASK_SAMPLE));
    APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_ppi_channel_enable(m_ppi_channel_timer_cc_sample);
    APP_ERROR_CHECK(err_code);
}

void saadc_end_start_ppi_init(void)
{
    ret_code_t err_code;
	static nrf_ppi_channel_t     m_ppi_channel_saadc_end_start;
    
    uint32_t saadc_end_event_addr = nrf_saadc_event_address_get(NRF_SAADC_EVENT_END);
    uint32_t saadc_start_task_addr   = nrf_saadc_task_address_get(NRF_SAADC_TASK_START);

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel_saadc_end_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel_saadc_end_start,
                                          saadc_end_event_addr,
                                          saadc_start_task_addr);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_enable(m_ppi_channel_saadc_end_start);

    APP_ERROR_CHECK(err_code);
}

void saadc_init(void)
{
	uint32_t err_code;
	
	//todo: increase resolution and oversampling
	nrf_saadc_resolution_set(NRF_SAADC_RESOLUTION_12BIT);
    nrf_saadc_oversample_set(NRF_SAADC_OVERSAMPLE_DISABLED);
    
    nrf_drv_common_irq_enable(SAADC_IRQn, APP_IRQ_PRIORITY_HIGHEST);
    
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
    nrf_saadc_int_enable(NRF_SAADC_INT_END);
    
    nrf_saadc_enable();
	
	nrf_saadc_channel_config_t l_config = {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,     
        .resistor_n = NRF_SAADC_RESISTOR_VDD1_2,     
        .gain       = NRF_SAADC_GAIN4,               
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,    
        .acq_time   = NRF_SAADC_ACQTIME_15US,          
        .mode       = NRF_SAADC_MODE_DIFFERENTIAL,     
        .burst      = NRF_SAADC_BURST_DISABLED,        
        .pin_p      = (nrf_saadc_input_t)(ADC_IN_PIN),
        .pin_n      = (nrf_saadc_input_t)(ADC_NC_PIN)
    };
    
    nrf_saadc_channel_init(0, &l_config);
    nrf_saadc_channel_input_set(0, l_config.pin_p, l_config.pin_n);
    
	nrf_saadc_buffer_init(saadc_buffer_1, FFT_TEST_COMP_SAMPLES_LEN);
	
	err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);
	
	saadc_end_start_ppi_init();
	saadc_timer_sample_ppi_init();
	
    nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
	
	//start timer
	nrf_drv_timer_enable(&timer0);
}

void SAADC_IRQHandler(void)
{
    if (nrf_saadc_event_check(NRF_SAADC_EVENT_END))
    {
        nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
        //data ready

		if(buffer_1_in_use)
		{
			nrf_gpio_pin_set(18);
			nrf_saadc_buffer_init(saadc_buffer_2, FFT_TEST_COMP_SAMPLES_LEN);
			update_fft_input_i16(saadc_buffer_1);
			nrf_gpio_pin_clear(18);
		}
		else
		{
			nrf_gpio_pin_set(18);
			nrf_saadc_buffer_init(saadc_buffer_1, FFT_TEST_COMP_SAMPLES_LEN);
			update_fft_input_i16(saadc_buffer_2);
			nrf_gpio_pin_clear(18);
		}
		buffer_1_in_use = !buffer_1_in_use;
	}
}

void audio_in_init(void)
{
	i2s_init();
	//saadc_init();	//noise on aux cable too high -> bad performance on FFT
}
