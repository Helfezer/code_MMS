#include "my_lib_twi.h"
/* Function which initialize the MPU6050 */
void init_MPU6050(nrf_drv_twi_t twi_instance)
{
uint8_t reset[2] = {REG_PWR_MGMT_1, 0x80};
uint8_t unsleep[2] = {REG_PWR_MGMT_1, 0x00};

			/* Reset the MPU */
			nrf_drv_twi_tx(&twi_instance, MPU_ADDR, reset, sizeof(reset), false);
			nrf_delay_ms(100);
			/* Unsleep the MPU */
			nrf_drv_twi_tx(&twi_instance, MPU_ADDR, unsleep, sizeof(unsleep), false);			
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
			nrf_gpio_pin_clear(25);
			nrf_gpio_pin_clear(24);
			nrf_gpio_pin_clear(23);
			break;		
		case 1:
			nrf_gpio_pin_set(25);
			nrf_gpio_pin_clear(24);
			nrf_gpio_pin_clear(23);
			break;
		case 2:
			nrf_gpio_pin_clear(25);
			nrf_gpio_pin_set(24);
			nrf_gpio_pin_clear(23);
			break;
		case 3:
			nrf_gpio_pin_set(25);
			nrf_gpio_pin_set(24);
			nrf_gpio_pin_clear(23);
			break;
		case 4:
			nrf_gpio_pin_clear(25);
			nrf_gpio_pin_clear(24);
			nrf_gpio_pin_set(23);
			break;
		case 5:
			nrf_gpio_pin_set(25);
			nrf_gpio_pin_clear(24);
			nrf_gpio_pin_set(23);
			break;
		case 6:
			nrf_gpio_pin_clear(25);
			nrf_gpio_pin_set(24);
			nrf_gpio_pin_set(23);
			break;
		case 7:
			nrf_gpio_pin_set(25);
			nrf_gpio_pin_set(24);
			nrf_gpio_pin_set(23);
			break;
		default:
			nrf_gpio_pin_clear(25);
			nrf_gpio_pin_clear(24);
			nrf_gpio_pin_clear(23);
			break;
	}
	
}
