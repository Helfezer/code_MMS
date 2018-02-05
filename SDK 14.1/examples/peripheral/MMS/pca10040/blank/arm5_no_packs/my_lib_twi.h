/*!
* @file my_lib_twi.h
* @brief Header for TWI fonctions
* @author Pantostier Quentin / Meissburger Jordan / Rakotovao Sebastien
* @version 1.0 
*/
#include <stdbool.h>
#include <stdint.h>
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/*!
* @biref delay between TWI trames 
*/
#define MPU_DELAY						100
#define MPU_DELAY_US				100

/*!
 * @brief Pin for swtich address 
 */
#define SWITCH_A 28
#define SWITCH_B 29
#define SWITCH_C 30

/*!
 * @brief Common addresses definition for accelereomter. 
 */
#define MPU_WHO_AM_I					0x75
#define MPU_ADDR							0x68
#define HMC_ADDR							0x1E
#define ADC_ADDR							0x6E

/*!
 * @brief Switch macros 
 */
#define NB_IMU 1

/*!
 * @brief MPU registers */
#define NB_ACC_REG_MPU 6
#define NB_GYR_REG_MPU 6
#define NB_TEMP_REG_MPU 2
#define NB_REG_HMC 6

#define REG_ACC_XH	0x3B
#define REG_ACC_XL	0x3C
#define REG_ACC_YH	0x3D
#define REG_ACC_YL	0x3E
#define REG_ACC_ZH	0x3F
#define REG_ACC_ZL	0x40

#define REG_TEMP_H 0x41
#define REG_TEMP_L 0x42

#define REG_GYR_XH	0x43
#define REG_GYR_XL	0x44
#define REG_GYR_YH	0x45
#define REG_GYR_YL	0x46
#define REG_GYR_ZH	0x47
#define REG_GYR_ZL	0x48

#define REG_PWR_MGMT_1 0x6B

/*!
 * @brief HMC Registers 
 */
#define HMC_CONFIG_A	0x00
#define HMC_CONFIG_B	0x01
#define HMC_MODE			0x02
#define HMC_XH				0x03
#define HMC_XL				0x04
#define HMC_ZH				0x05
#define HMC_ZL				0x06
#define HMC_YH				0x07
#define HMC_YL				0x08

/*!
 * @typedef
 * @struct IMU
 * @brief structure use to save data send by IMU
 */
typedef struct 
{
	uint16_t acc_x, acc_y, acc_z; /*!< 2bytes data for acceleration on X, Y and Z */
	uint16_t gyr_x, gyr_y, gyr_z; /*!< 2bytes data for gyroscope on X, Y and Z */
	uint16_t temp; /*!< 2bytes data for temp */
	uint16_t mag_x, mag_y, mag_z; /*!< 2bytes data for magnitude on X, Y and Z */
	uint16_t adc;
}IMU;

/*!
 * @fn init_switch_pins (void)
 * @brief Initialize pin use by the switchs
 */
void init_switch_pins(void);

/*!
 * @fn init_MPU6050 (nrf_drv_twi_t twi_instance)
 * @brief initialise MPU6050(IMU) with the corresponding twi instance
 *
 * @param[in] twi_instance nrf_drv_twi_t that indicate the twi_instance use
 */
void init_MPU6050(nrf_drv_twi_t twi_instance);

/*!
 * @fn init_HMC5883 (nrf_drv_twi_t twi_instance)
 * @brief initialise HMC5883 with the corresponding twi instance
 *
 * @param[in] twi_instance nrf_drv_twi_t that indicate the twi_instance use
*/
void init_HMC5883(nrf_drv_twi_t twi_instance);

/*!
 * @fn switch_imu (int imu)
 * @brief getting all info from the IMUs 
 * 
 * @param[in] imu int varaible that give IMU number
 */ 
void switch_imu(int imu);

/*!
 * @fn init (void)
 * @brief init fonction use in main.c
 */
void init(void);

/*
 * Initialialization function for the RTC
 * Init RTCSEC to 0x00 to start counting
 */
void init_RTC(void);
/*
 * return the value which is the register reg
 */
uint8_t RTC_function(uint8_t regis, nrf_drv_twi_t twi_instance);

/*!
 * @fn ADC_function (void)
 * @brief function reading ADC on TWI bus
 *
 * @param[in] twi_instance instance of the TWI to use for the communication
 */
uint16_t ADC_function(nrf_drv_twi_t twi_instance);


/*!
 * @fn IMU_function
 * @brief function reading IMU data on twi bus
 *
 * @param[in] twi_instance instance of the TWI to use for the communication
 * @param[in] reg address of the register to read
 * @param[in] addr adresse of the device on twi bus
 */
 uint8_t IMU_function(nrf_drv_twi_t twi_instance, uint8_t addr, uint8_t* reg);
	 
