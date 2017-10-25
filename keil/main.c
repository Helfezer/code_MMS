/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example_freertos
 *
 * @brief Blinky FreeRTOS Example Application main file.
 *
 * This file contains the source code for a sample application using FreeRTOS to blink LEDs.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf_gpio.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"
#include "app_uart.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "my_lib_twi.h"




#define TASK_DELAY        200    /**< Task delay. Delays a LED0 task for 200 ms */
#define TWI_DELAY					1


/* *****************************************************************
********************************************************************
********************************************************************/

/*Pins to connect shield. */
#define ARDUINO_I2C_SCL_PIN 26
#define ARDUINO_I2C_SDA_PIN 27

/*UART buffer size. */
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 1


/*Tilt specific bits*/
#define TILT_TAP_MASK (1U << 5)
#define TILT_SHAKE_MASK (1U << 7)

// [max 255, otherwise "int16_t" won't be sufficient to hold the sum
//  of accelerometer samples]
#define NUMBER_OF_SAMPLES 20

/* Define version of GCC. */
#define GCC_VERSION (__GNUC__ * 10000 \
                     + __GNUC_MINOR__ * 100 \
                     + __GNUC_PATCHLEVEL__)




/**
 * @brief Union to keep raw and converted data from accelerometer samples at one memory space.
 */
typedef union{
    uint8_t raw;
    int8_t  conv;
}elem_t;

/**
 * @brief Enum for selecting accelerometer orientation.
 */
typedef enum{
    LEFT = 1,
    RIGHT = 2,
    DOWN = 5,
    UP = 6
}accelerometer_orientation_t;

/**
 * @brief Structure for holding samples from accelerometer.
 */

typedef struct
{
    elem_t  x;
    elem_t  y;
    elem_t  z;
    uint8_t tilt;
} sample_t;

/* MPU accelerometer register tab */
uint8_t mpu_acc_reg[NB_ACC_REG_MPU] = {REG_ACC_XH, REG_ACC_XL, REG_ACC_YH, REG_ACC_YL, REG_ACC_ZH, REG_ACC_ZL};

/* MPU gyrometer register tab */
uint8_t mpu_gyr_reg[NB_GYR_REG_MPU] = {REG_GYR_XH, REG_GYR_XL, REG_GYR_YH, REG_GYR_YL, REG_GYR_ZH, REG_GYR_ZL};

/* MPU thermometer register tab */
uint8_t mpu_temp_reg[NB_TEMP_REG_MPU] = {REG_TEMP_H, REG_TEMP_L};

/* HMC register tab */ 
uint8_t hmc_reg[NB_REG_HMC] = {HMC_XH, HMC_XL, HMC_YH, HMC_YL, HMC_ZH, HMC_ZL};

IMU imu_list[NB_IMU];


#ifdef __GNUC_PATCHLEVEL__
#if GCC_VERSION < 50505
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-braces"           // Hack to GCC 4.9.3 bug. Can be deleted after switch on using GCC 5.0.0
#endif
#endif

#ifdef __GNUC_PATCHLEVEL__
#if GCC_VERSION < 50505
#pragma GCC diagnostic pop
#endif
#endif
/* Indicates if reading operation from accelerometer has ended. */
static volatile bool m_xfer_done = true;
/* Indicates if setting mode operation has ended. */
static volatile bool m_set_mode_done = false;
/* TWI instance. */
static const nrf_drv_twi_t m_twi_mma_7660 = NRF_DRV_TWI_INSTANCE(0);


/**
 * @brief UART events handler.
 */


static void uart_events_handler(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}



/**
 * @brief UART initialization.
 */

static void uart_config(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud38400
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_events_handler,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{   
    ret_code_t err_code;
    static sample_t m_sample;
    
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if ((p_event->type == NRF_DRV_TWI_EVT_DONE) &&
                (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX))
            {
                if(m_set_mode_done != true)
                {
                    m_set_mode_done  = true;
                    return;
                }
                m_xfer_done = false;
                /* Read 4 bytes from the specified address. */
                //err_code = nrf_drv_twi_rx(&m_twi_mma_7660, READ_ADDR, (uint8_t*)&m_sample, sizeof(m_sample));
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                m_xfer_done = true;
            }
            break;
        default:
            break;        
    }   
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t twi_mma_7660_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    
    err_code = nrf_drv_twi_init(&m_twi_mma_7660, &twi_mma_7660_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_twi_enable(&m_twi_mma_7660);
}

/* *****************************************************************
********************************************************************
********************************************************************/

/**@brief LED0 task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
static void vLed0Function (void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    for( ;; )
    {
        nrf_gpio_pin_toggle(BSP_LED_0);
        vTaskDelay(TASK_DELAY); // Delay a task for a given number of ticks

        // Tasks must be implemented to never return...
    }
}


static void vTwiFunction (void *pvParameter)
{
		int j = 0,init = 0,i = 0;	
	  UNUSED_PARAMETER(pvParameter);

		//buffer for accelerometer values
		uint8_t acc[6] = {0};
		//buffer for gyrometer values
		uint8_t gyr[6] = {0};
		//buffer for magnetometer values
		uint8_t mag[6] = {0};
		//buffer for temperature values
		uint8_t temp[2] = {0};
		
		for(;;)
		{
			if(init != NB_IMU)
			{
				//Init MPU6050 and HMC5883
				init_MPU6050(m_twi_mma_7660);
				nrf_delay_ms(50);
				init_HMC5883(m_twi_mma_7660);
				nrf_delay_ms(50);
				init ++;
			}
// Get values from accelerometer **********************************************
// Point at register			
			nrf_drv_twi_tx(&m_twi_mma_7660, MPU_ADDR, &mpu_acc_reg[i], 1, false);
			nrf_delay_us(80);
			// read register
		nrf_drv_twi_rx(&m_twi_mma_7660, MPU_ADDR, &acc[i], 1);
			nrf_delay_us(80);			
		
// Get values from gyrometer **********************************************
// Point at register
			nrf_drv_twi_tx(&m_twi_mma_7660, MPU_ADDR, &mpu_gyr_reg[i], 1, false);
			nrf_delay_us(80);
			// read register	
		nrf_drv_twi_rx(&m_twi_mma_7660, MPU_ADDR, &gyr[i], 1);
			nrf_delay_us(80);
			
			
// Get values from magnetomter **********************************************
// Point at register
			nrf_drv_twi_tx(&m_twi_mma_7660, HMC_ADDR, &hmc_reg[i], 1, false);
			nrf_delay_us(80);
			// read register	
		nrf_drv_twi_rx(&m_twi_mma_7660, HMC_ADDR, &mag[i], 1);
			nrf_delay_us(80);
		
		
// Get values from temperature  **********************************************
// Point at register
		if ( i < 3)
		{
			nrf_drv_twi_tx(&m_twi_mma_7660, MPU_ADDR, &mpu_temp_reg[i], 1, false);
			nrf_delay_us(80);
			// read register
			nrf_drv_twi_rx(&m_twi_mma_7660, MPU_ADDR, &temp[i], 1);
			nrf_delay_us(80);
			i++;
		}
		else if (i == 5)
			{
				// Accelerometer X
				imu_list[j].acc_x = (acc[0]<<8)|acc[1];
						// Accelerometer Y
				imu_list[j].acc_y = (acc[2]<<8)|acc[3];
				// Accelerometer Z
				imu_list[j].acc_z = (acc[4]<<8)|acc[5];
				// Gyrometer X
				imu_list[j].gyr_x = (gyr[0]<<8)|gyr[1];
				// Gyrometer Y
				imu_list[j].gyr_y = (gyr[2]<<8)|gyr[3];
				// Gyrometer Z
				imu_list[j].gyr_z = (gyr[4]<<8)|gyr[5];
				// Magnetometer X
				imu_list[j].mag_x = (mag[0]<<8)|mag[1];
				// Magnetometer Z
				imu_list[j].mag_y = (mag[2]<<8)|mag[3];
				// Magnetometer Y
				imu_list[j].mag_z = (mag[4]<<8)|mag[5];
				//Temperature
				imu_list[j].temp = (temp[0]<<8)|temp[1];
				
				printf("X=%d, X=%d X=%d X=%d X=%d X=%d X=%d X=%d X=%d X=%d", imu_list[j].acc_x, imu_list[j].acc_y, imu_list[j].acc_z, imu_list[j].gyr_x, imu_list[j].gyr_y, imu_list[j].gyr_z, imu_list[j].mag_x, imu_list[j].mag_y, imu_list[j].mag_z, imu_list[j].temp);
				i = 0;
				//increase the IMU index
				
				if(j == NB_IMU-1)
				{
					j = 0;					
					nrf_delay_ms(200);
				}
				else
				{
					nrf_delay_ms(200);
					
					j++;					
				}
				switch_imu(j);
			}
			else
			{
				i++;
			}
		}
}
int main(void)
{
		uart_config();
		twi_init();
    TaskHandle_t  xLed0Handle;       /**< Reference to LED0 toggling FreeRTOS task. */
		TaskHandle_t  xTwiHandle;       /**< Reference to LED0 toggling FreeRTOS task. */
    ret_code_t err_code;
	
		
	
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    // Configure LED-pins as outputs
    nrf_gpio_cfg_output(BSP_LED_0);
    nrf_gpio_cfg_output(BSP_LED_1);
    nrf_gpio_cfg_output(BSP_LED_2);
    nrf_gpio_cfg_output(BSP_LED_3);
		nrf_gpio_cfg_output(25);
		nrf_gpio_cfg_output(24);
		nrf_gpio_cfg_output(23);
    nrf_gpio_pin_set(BSP_LED_0);
    nrf_gpio_pin_set(BSP_LED_1);
    nrf_gpio_pin_set(BSP_LED_2);
    nrf_gpio_pin_set(BSP_LED_3);
		nrf_gpio_pin_clear(25);
		nrf_gpio_pin_clear(24);
		nrf_gpio_pin_clear(23);
    UNUSED_VARIABLE(xTaskCreate( vLed0Function, "L0", configMINIMAL_STACK_SIZE + 200, NULL, 2, &xLed0Handle ));    // LED0 task creation                       
		UNUSED_VARIABLE(xTaskCreate( vTwiFunction, "T1", configMINIMAL_STACK_SIZE + 200, NULL, 2, &xTwiHandle )); 
		
    /* Activate deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    while (true)
    {
        // FreeRTOS should not be here...
    }
}

/**
 *@}
 **/
