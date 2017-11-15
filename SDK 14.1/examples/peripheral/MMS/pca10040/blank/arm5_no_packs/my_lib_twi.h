#include <stdbool.h>
#include <stdint.h>
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* delay between TWI trames */
#define MPU_DELAY						100
#define MPU_DELAY_US				80

/* Pin for swtich address */
#define SWITCH_A 28
#define SWITCH_B 29
#define SWITCH_C 30

/*Common addresses definition for accelereomter. */
#define MPU_WHO_AM_I					0x75
#define MPU_ADDR							0x68
#define HMC_ADDR							0x1E

/* Switch macros */
#define NB_IMU 1

/* MPU registers */
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

/* HMC Registers */
#define HMC_CONFIG_A	0x00
#define HMC_CONFIG_B	0x01
#define HMC_MODE			0x02
#define HMC_XH				0x03
#define HMC_XL				0x04
#define HMC_ZH				0x05
#define HMC_ZL				0x06
#define HMC_YH				0x07
#define HMC_YL				0x08

 
typedef struct 
{
	int16_t acc_x, acc_y, acc_z; 
	int16_t gyr_x, gyr_y, gyr_z;
	int16_t temp;
	int16_t mag_x, mag_y, mag_z;
}IMU;

void init_switch_pins(void);
void init_MPU6050(nrf_drv_twi_t twi_instance);
void init_HMC5883(nrf_drv_twi_t twi_instance);
/* getting all info from the IMUs */ 
void switch_imu(int imu);
void init(void);
