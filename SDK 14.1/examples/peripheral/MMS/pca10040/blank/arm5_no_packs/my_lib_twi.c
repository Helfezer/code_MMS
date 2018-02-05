#include "my_lib_twi.h"

/* initialize the pin which are needed to drive switch output */
void init_switch_pins(void)
{
		/* Pins set as output */ 
		nrf_gpio_cfg_output(SWITCH_A);
		nrf_gpio_cfg_output(SWITCH_B);
		nrf_gpio_cfg_output(SWITCH_C);
		/* Pins set to low at the beginning */
		nrf_gpio_pin_clear(SWITCH_A);
		nrf_gpio_pin_clear(SWITCH_B);
		nrf_gpio_pin_clear(SWITCH_C);
}

void init_RTC(void)
{
		NRF_LOG_INFO("Init_RTC");
		uint8_t rtc_address = 0x6f;
		uint8_t reg[2];
		reg[0] = 0x00; // RTCSEC register address
		reg[1] = 0x00; // set RTCSEC to 0x00

		nrf_drv_twi_tx(&m_twi, rtc_address, &reg[0], 2, false); // register seconds
		nrf_delay_us(100);
}

/* Function which initialize the MPU6050 */
void init_MPU6050(nrf_drv_twi_t twi_instance)
{
			NRF_LOG_INFO ("entering init_imu\n");
uint8_t reset[2] = {REG_PWR_MGMT_1, 0x80};
uint8_t unsleep[2] = {REG_PWR_MGMT_1, 0x00};

			/* Reset the MPU */
			nrf_drv_twi_tx(&twi_instance, MPU_ADDR, reset, sizeof(reset), false);
			nrf_delay_ms(100);
			/* Unsleep the MPU */
			nrf_drv_twi_tx(&twi_instance, MPU_ADDR, unsleep, sizeof(unsleep), false);		
		

NRF_LOG_INFO ("exiting init_imu\n");

}



/* Function that initialize the HMC5883L */
void init_HMC5883(nrf_drv_twi_t twi_instance)
{
uint8_t config_a[2] = {HMC_CONFIG_A, 0x14};
uint8_t mode[2]			= {HMC_MODE, 0x00};
			
			/* Set the config A register */
			nrf_drv_twi_tx(&twi_instance, HMC_ADDR, config_a, sizeof(config_a), false);
			nrf_delay_ms(100);
			/* Set the mode register (continuous mode) */
			nrf_drv_twi_tx(&twi_instance, HMC_ADDR, mode, sizeof(mode), false);		
}

void switch_imu(int imu)
{
	switch(imu)
	{
		case 0:
			nrf_gpio_pin_clear(SWITCH_A);
			nrf_gpio_pin_clear(SWITCH_B);
			nrf_gpio_pin_clear(SWITCH_C);
			break;		
		case 1:
			nrf_gpio_pin_set(SWITCH_A);
			nrf_gpio_pin_clear(SWITCH_B);
			nrf_gpio_pin_clear(SWITCH_C);
			break;
		case 2:
			nrf_gpio_pin_clear(SWITCH_A);
			nrf_gpio_pin_set(SWITCH_B);
			nrf_gpio_pin_clear(SWITCH_C);
			break;
		case 3:
			nrf_gpio_pin_set(SWITCH_A);
			nrf_gpio_pin_set(SWITCH_B);
			nrf_gpio_pin_clear(SWITCH_C);
			break;
		case 4:
			nrf_gpio_pin_clear(SWITCH_A);
			nrf_gpio_pin_clear(SWITCH_B);
			nrf_gpio_pin_set(SWITCH_C);
			break;
		case 5:
			nrf_gpio_pin_set(SWITCH_A);
			nrf_gpio_pin_clear(SWITCH_B);
			nrf_gpio_pin_set(SWITCH_C);
			break;
		case 6:
			nrf_gpio_pin_clear(SWITCH_A);
			nrf_gpio_pin_set(SWITCH_B);
			nrf_gpio_pin_set(SWITCH_C);
			break;
		case 7:
			nrf_gpio_pin_set(SWITCH_A);
			nrf_gpio_pin_set(SWITCH_B);
			nrf_gpio_pin_set(SWITCH_C);
			break;
		default:
			nrf_gpio_pin_clear(SWITCH_A);
			nrf_gpio_pin_clear(SWITCH_B);
			nrf_gpio_pin_clear(SWITCH_C);
			break;
	}
	
}

uint8_t RTC_function(uint8_t regis, nrf_drv_twi_t twi_instance)
{
		NRF_LOG_INFO("RTC_function");
		uint8_t rtc_address = 0x6f;
		uint8_t reg[1];
		reg[0] = regis; //minutes
		uint8_t rtc[1] = {0};
		
		// read RTC registers
		nrf_drv_twi_tx(&twi_instance, rtc_address, &reg[0], 1, false); // register seconds
		nrf_delay_us(100);
		nrf_drv_twi_rx(&twi_instance, rtc_address, &rtc[0], 1);				// read seconds
		nrf_delay_us(100);
		NRF_LOG_INFO("Recu: %02x", rtc[0]);
		
		return rtc[0];
}

uint16_t ADC_function(nrf_drv_twi_t twi_instance)
{
	NRF_LOG_INFO("ADC_function");
	uint8_t reg[2] = {0};						// storage register
	uint16_t ret;
	
	nrf_drv_twi_rx(&twi_instance, ADC_ADDR, reg, 2);		// read registers
	nrf_delay_us(100);
	ret =(reg[0]<<8)|reg[1];
	NRF_LOG_INFO("ADC value =%04x", ret); 
	
	return ret;
}

uint8_t IMU_function(nrf_drv_twi_t twi_instance,uint8_t addr, uint8_t* reg)
{
		uint8_t write[1] = {0};
		
		nrf_drv_twi_tx(&twi_instance, addr, reg, 1, false);	// Point at register
		nrf_delay_us(MPU_DELAY_US);			
		nrf_drv_twi_rx(&twi_instance, addr, &write[0], 1);				// read register
		nrf_delay_us(MPU_DELAY_US);
	
		return write[0];
};



