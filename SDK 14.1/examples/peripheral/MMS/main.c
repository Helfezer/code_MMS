/**
 * Copyright (c) 2017 - 2017, Nordic Semiconductor ASA
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
 //Création des principaux doxygen module
/*!
 * @defgroup MMS config
 */
/*! 
 * @file main.c
 * \author Pantostier Quentin / Meissburger Jordan / Rakotovao Sebastien
 * \version 1.0
 *
 * @brief Main file of MMS project.
 * This file contains the source code that define the various task of MMS project.
 */

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"

#include "nrf_drv_rtc.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_uart.h"

#include "my_lib_twi.h"
#include "SDcard.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "queue.h"
#include "task.h"

#define FILE_NAME "MMS.TXT"
uint8_t failed;

#if LEDS_NUMBER <= 2
#error "Board is not equipped with enough amount of LEDs"
#endif

/**
 * @brief RTC instance number used for blinking
 *
 */
#define BLINK_RTC 2

/**
 * @brief RTC compare channel used
 *
 */
#define BLINK_RTC_CC 0

/**
 * @brief Number of RTC ticks between interrupts
 */
#define BLINK_RTC_TICKS   (RTC_US_TO_TICKS(500000ULL, RTC_DEFAULT_CONFIG_FREQUENCY))

/**
 * @brief Reference to ADC reading task.
 */
static TaskHandle_t  m_ADC_task_handle;


/**
 * @brief Reference to LED0 toggling FreeRTOS task.
 */
//static TaskHandle_t  m_led_toggle_task_handle;

/**
 * @brief Reference to Twi IMU reading task.
 */
TaskHandle_t  xTwiHandle;

/**
 * @brief Reference to RTC task.
 */
TaskHandle_t  xRTCHandle;

/**
 * @brief Reference to Reading the Queue task.
 */
TaskHandle_t  xQHandleRead;

/**
 * @brief Reference to Queue writing task.
 */
TaskHandle_t  xQHandleWrite;

/**
 * @brief Reference to SDcard writing task.
 */
TaskHandle_t  xSDHandle;

/**
 * @brief Reference to start task.
 */
 TaskHandle_t xStartHandle;
 
 /**
 * @brief Reference to stop task.
 */
 TaskHandle_t xStopHandle;

/**
 * @brief Reference to Queue
 */
QueueHandle_t xQueue;

/**
 * @brief Semaphore set in RTC event
 */
static SemaphoreHandle_t m_led_semaphore;

/**
 * @brief Mutex protecting the TWI (I2C)
 */
static SemaphoreHandle_t m_twi_mutex;

/**
 * @brief RTC configuration
 */
static nrf_drv_rtc_config_t const m_rtc_config = NRF_DRV_RTC_DEFAULT_CONFIG;

/**
 * @brief RTC instance
 *
 * Instance of the RTC used for led blinking
 */
static nrf_drv_rtc_t const m_rtc = NRF_DRV_RTC_INSTANCE(BLINK_RTC);

static volatile bool m_xfer_done = true;
static volatile bool m_set_mode_done = false;

bool is_open = false;
bool stop = false;


/*!
 * @brief TWI instance (set to 1 as define in sdk_config.h)
 */
static nrf_drv_twi_t const m_twi = NRF_DRV_TWI_INSTANCE(1);

/**
 * @brief UART instance
 */
static nrf_drv_uart_t const m_uart = NRF_DRV_UART_INSTANCE(0);

/*!
 * @brief MPU accelerometer register tab 
 */
uint8_t mpu_acc_reg[NB_ACC_REG_MPU] = {REG_ACC_XH, REG_ACC_XL, REG_ACC_YH, REG_ACC_YL, REG_ACC_ZH, REG_ACC_ZL};

/*!
 * @brief MPU gyrometer register tab 
 */
uint8_t mpu_gyr_reg[NB_GYR_REG_MPU] = {REG_GYR_XH, REG_GYR_XL, REG_GYR_YH, REG_GYR_YL, REG_GYR_ZH, REG_GYR_ZL};

/*!
 * @brief MPU thermometer register tab 
 */
uint8_t mpu_temp_reg[NB_TEMP_REG_MPU] = {REG_TEMP_H, REG_TEMP_L};

/*!
 * @brief HMC register tab 
 * @todo precise what is HMC
 */ 
uint8_t hmc_reg[NB_REG_HMC] = {HMC_XH, HMC_XL, HMC_YH, HMC_YL, HMC_ZH, HMC_ZL};

/*!
 * @cond PROTOTYPE
 */

/*
 * Prototype of the init function for pin interruption
 */
void init_pin_interrupt (void);

/*
 * Prototype of the init function for twi
 */
void twi_init (void)
	
/*
 * Prototype of the init function
 */
void init(void);

/*
 *  Prototype of the task function that reading all IMUs
 */
void vTwiFunction (void *pvParameter);

/*
 *  Prototype of the task function that read Imus data contain by Queue
 */
void vQueueRead(void* pvParameter);

/*!
 * @endcond
 */

/*
static void blink_rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
		uint32_t time;
	
    BaseType_t yield_req = pdFALSE;
    ret_code_t err_code;
    bsp_board_led_invert(BSP_BOARD_LED_1);
    err_code = nrf_drv_rtc_cc_set(
        &m_rtc,
        BLINK_RTC_CC,
        (nrf_rtc_cc_get(m_rtc.p_reg, BLINK_RTC_CC) + BLINK_RTC_TICKS) & RTC_COUNTER_COUNTER_Msk,
        true);
		time = nrf_drv_rtc_counter_get(&m_rtc);
		//NRF_LOG_INFO("time:%d", time/16384);
    APP_ERROR_CHECK(err_code);

   *The returned value may be safely ignored, if error is returned it only means that
   *the semaphore is already given (raised). 
   UNUSED_VARIABLE(xSemaphoreGiveFromISR(m_led_semaphore, &yield_req));
   portYIELD_FROM_ISR(yield_req);
}
*/


/*
 * @brief ADC task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 *
static void ADC_task_function (void * pvParameter)
{
	NRF_LOG_INFO("test_task_function");
    UNUSED_PARAMETER(pvParameter);
		uint8_t adc_address = 0x6E; // ADC address
		char reg[10] = {0};					// storage register
    while (true)
    {
			//nrf_drv_twi_tx(&m_twi, reg[0], &reg[1], 1, false);
			//nrf_delay_us(50);
			// read register
			nrf_drv_twi_rx(&m_twi, adc_address, reg, 6);		// read register
			
			nrf_delay_ms(100);
			//NRF_LOG_INFO("Registre %d ", buf[0]);
			//if(i==31) i = 0;
			//i++;
			//nrf_delay_ms(500);
			NRF_LOG_INFO("Recu: %02x %02x %02x %02x %02x %02x", reg[0], reg[1], reg[2], reg[3], reg[4], reg[5]);
    }

     Tasks must be implemented to never return... 
}
*/


/*
 * @fn led_toggle_task_function (void* pvParameter)
 * @brief LED0 task function, make led blink.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.

static void led_toggle_task_function (void * pvParameter)
{
	NRF_LOG_INFO("led_toggle_task_function");
    ret_code_t err_code;	
    err_code = nrf_drv_rtc_init(&m_rtc, &m_rtc_config, blink_rtc_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_rtc_cc_set(&m_rtc, BLINK_RTC_CC, BLINK_RTC_TICKS, true);
    APP_ERROR_CHECK(err_code);
    nrf_drv_rtc_enable(&m_rtc);

    m_led_semaphore = xSemaphoreCreateBinary();
    ASSERT(NULL != m_led_semaphore);
		//TickType_t xDelay = 100;
    UNUSED_PARAMETER(pvParameter);
    while (true)
    {
        bsp_board_led_invert(BSP_BOARD_LED_0);
				//vTaskDelay(xDelay);
        * Wait for the event from the RTC 
        UNUSED_RETURN_VALUE(xSemaphoreTake(m_led_semaphore, portMAX_DELAY));
				
    }

     Tasks must be implemented to never return... 
}
*/
/*
static void RTC_task_function(void * pvParameter)
{
		//NRF_LOG_INFO("RTC_task_function");
    UNUSED_PARAMETER(pvParameter);
		uint8_t rtc_address = 0x6f;
		uint8_t reg[5];
		reg[0] = 0x00;
		reg[1] = 0x80; //seconds
		reg[2] = 0x00; //minutes
		reg[3] = 0x03; //RTCWKDAY
		reg[4] = 0x09; //VBATEN = 1
		uint8_t rtc[1] = {0};
		//nrf_drv_twi_tx(&m_twi, rtc_address, &reg[0], 1, false); // seconde to 00 (including CH)
		//nrf_delay_us(100);
		nrf_drv_twi_tx(&m_twi, rtc_address, &reg[0], 2, false); // register seconds
		nrf_delay_us(100);
		nrf_drv_twi_rx(&m_twi, rtc_address, &rtc[0], 1);						// read seconds
		nrf_delay_ms(100);
		
		nrf_drv_twi_tx(&m_twi, rtc_address, &reg[3], 2, false); // register seconds
		nrf_delay_us(100);
		
    while(true)
    {			
			nrf_drv_twi_tx(&m_twi, rtc_address, &reg[2], 1, false); // register seconds
			nrf_delay_us(100);
			nrf_drv_twi_rx(&m_twi, rtc_address, &rtc[0], 1);						// read seconds
			NRF_LOG_INFO("Recu: %02x", rtc[0]);
			
			nrf_delay_ms(500);
			
			NRF_LOG_INFO("---------------------------------------------------------------------------------");
			
			//NRF_LOG_INFO("Registre %d ", buf[0]);
			//if(i==31) i = 0;
			//i++;
			//nrf_delay_ms(500);
    }

    // Tasks must be implemented to never return... 
}
*/

/*!
 * @fn start_task_function (void *pvParameter)
 * @brief Start task function.
 *
 * @param[in] pvParameter   Void Pointer that can be used to past the parameter for the task.(not used here)
 *
 * @remark This task function is deleted when it finish.
 */
static void start_task_function(void * pvParameter)
{
		UNUSED_PARAMETER(pvParameter);
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY ); //wait for interrupt notify (button 0)
		xTaskNotifyGive(xSDHandle); // send signal to SDCard function to start all
		vTaskDelete(xStartHandle);
   /* Tasks must be implemented to never return... */
}	

/*!
 * @fn stop_task_function (void* pvParameter)
 * @brief Stop task function.
 *
 * @param[in] pvParameter   Void Pointer that can be used to past the parameter for the task.(not used here)
 *
 * @remark This task function is deleted when it finish.
 */
static void stop_task_function(void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY ); //wait for interrupt notify
		stop = true; //change stop flag state	
		vTaskDelete(xStopHandle);
   /* Tasks must be implemented to never return... */

}

/*!
 * @fn vTwiFunction (void* pvParameter)
 * @brief Twi IMUs reading task function.
 *
 * @param[in] pvParameter   void pointer used to past the queue handle to the task.
 */
void vTwiFunction (void *pvParameter)
{		
	NRF_LOG_INFO("twi_task_function");
	QueueHandle_t xQueue = (QueueHandle_t) pvParameter;
	
	uint8_t acc[6] = {0};					//buffer for accelerometer values	
	uint8_t gyr[6] = {0};					//buffer for gyrometer values	
	uint8_t mag[6] = {0};					//buffer for magnetometer values	
	uint8_t temp[2] = {0};				//buffer for temperature values
	uint16_t adc_buf[1] = {0};		//buffer for adc values
	
	IMU imu_list[NB_IMU];					// IMU struct creation	
	const TickType_t xDelay = 80;	// set sampling speed
	
	int j = 0; 										//imu id index
	int i = 0; 										//imu register id
	
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY ); //wait for SDCard task signal to start looping
	
	while(!stop)
	{			
		// Get values from accelerometer		
		xSemaphoreTake(m_twi_mutex, portMAX_DELAY);										//protecting the twi during transaction
		acc[i] = IMU_function(m_twi, MPU_ADDR, &mpu_acc_reg[i]);
		xSemaphoreGive(m_twi_mutex);																	// Free the twi
		
		
		// Get values from gyrometer		
		xSemaphoreTake(m_twi_mutex, portMAX_DELAY);										//protecting the twi during transaction									
		gyr[i] = IMU_function(m_twi, MPU_ADDR, &mpu_gyr_reg[i]);
		xSemaphoreGive(m_twi_mutex);																	// Free the twi
		
		
		// Get values from magnetomter	
		xSemaphoreTake(m_twi_mutex, portMAX_DELAY);										//protecting the twi during transaction
		mag[i] = IMU_function(m_twi, HMC_ADDR, &hmc_reg[i]);
		xSemaphoreGive(m_twi_mutex);																	// Free the twi
	
		// Get values from temperature
		if ( i < 2)																											// Two first register read, now reading temperature
		{				
			xSemaphoreTake(m_twi_mutex, portMAX_DELAY);										//protecting the twi during transaction
			temp[i] = IMU_function(m_twi, MPU_ADDR, &mpu_temp_reg[i]);
			xSemaphoreGive(m_twi_mutex);																	// Free the twi
			
			i++;
		}
		else if (i == 5)																								// All registers have been read
		{
			adc_buf[0] = ADC_function(m_twi);
			// Fill structure with data
			imu_list[j].acc_x = (acc[0]<<8)|acc[1]; 	// Accelerometer X				
			imu_list[j].acc_y = (acc[2]<<8)|acc[3];		// Accelerometer Y				
			imu_list[j].acc_z = (acc[4]<<8)|acc[5];		// Accelerometer Z
			imu_list[j].gyr_x = (gyr[0]<<8)|gyr[1];		// Gyrometer X
			imu_list[j].gyr_y = (gyr[2]<<8)|gyr[3];		// Gyrometer Y	
			imu_list[j].gyr_z = (gyr[4]<<8)|gyr[5];		// Gyrometer Z
			imu_list[j].mag_x = (mag[0]<<8)|mag[1];		// Magnetometer X	
			imu_list[j].mag_y = (mag[2]<<8)|mag[3];		// Magnetometer Z		
			imu_list[j].mag_z = (mag[4]<<8)|mag[5];		// Magnetometer Y		
			imu_list[j].temp = (temp[0]<<8)|temp[1];	// Temperature
			imu_list[j].adc = adc_buf[0];							// ADC value
			
			xQueueSendToBack(xQueue, (void*) &imu_list[j], ( TickType_t ) 10);	// write the data struct to the queue
			xTaskNotifyGive(xSDHandle);																					// data available for reading, notify SDCard task
			ulTaskNotifyTake( pdTRUE, portMAX_DELAY );													// Wait for SDCard notify to continue
			
			i = 0;																															// Reset register index		
			
			if(j == NB_IMU-1)																										// All IMU read, reset IMU index
			{
				j = 0;					
			}
			else
			{					
				j++;																															// Else increase IMU index					
			}
			
			NRF_LOG_INFO("swithing to imu_%d", j);
			switch_imu(j);																											// Switching IMU
			xTaskNotifyGive(xSDHandle);																					// Notify SDCard
			ulTaskNotifyTake( pdTRUE, portMAX_DELAY );													// Wair for SDCard Notify
		}
		else
		{
			i++;																															// Increase register index
		}
		vTaskDelay(xDelay);																									// Delay
	}
	NRF_LOG_INFO("Exiting Twi task");
	xTaskNotifyGive(xSDHandle);																							// Notify SDCard to unlock it
	vTaskDelete(xTwiHandle);																								// Deleting task
}

/*******************************************************************************************************
********************************************************************************************************
*******************************************************************************************************/
/*!
 * @fn vSDCardFunction (void* pvParameter)
 * @brief SDcard writing task entry function.
 *
 * @param[in] pvParameter   void pointer used to past the queue handle to the task.
 */
static void vSDCardFunction (void *pvParameter)
{
		NRF_LOG_INFO("vSDCardFunction");
		QueueHandle_t xQueue = (QueueHandle_t) pvParameter;
		IMU BufferReceive;
		BaseType_t xReadResult;		
		static FIL file;
		FRESULT ff_result;
		uint32_t bytes_written;	
		char s[50] = {0};
		int n, reg_index;
		
	
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY ); //waiting for start signal
		// signal receive, opening file
		NRF_LOG_INFO ("Opening file: ");
		ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);			
		NRF_LOG_INFO("Error code: %d", ff_result);
		is_open = true;
		// file opened notify Twi task to unlock it
	  
		for(reg_index=0; reg_index<7; reg_index++) //read first 7 RTC register
		{
			
			n = sprintf(s,"%02x",RTC_function(reg_index, m_twi));
			NRF_LOG_INFO("valeur %s du registre %d, nbr caract %d", s, reg_index, n);
			NRF_LOG_INFO("fonction %d", RTC_function(reg_index, m_twi));
			ff_result = f_write(&file, s, n, (UINT *) &bytes_written); //writing data
					if (ff_result != FR_OK)
					{
						NRF_LOG_INFO("Write failed.");
					}
					else
					{
						NRF_LOG_INFO("%d bytes written.", bytes_written);
					}
		}		
		xTaskNotifyGive(xTwiHandle);
		
	while(!stop)	
	{
			//NRF_LOG_INFO("Flag state SD %d:", is_open);
			ulTaskNotifyTake( pdTRUE, portMAX_DELAY ); //wait for Twi signal, waiting for data to write
			//check for stop flag
			//datas are available
			xReadResult = xQueueReceive(xQueue, &BufferReceive, (TickType_t) 10);	//get data
			if (xReadResult == pdTRUE)
			{  //formating datas
				n = sprintf(s,"%04x%04x%04x%04x%04x%04x%04x%04x%04x%04x%04x"
																	, BufferReceive.acc_x
																	, BufferReceive.acc_y
																	, BufferReceive.acc_z
																	, BufferReceive.gyr_x
																	, BufferReceive.gyr_y
																	, BufferReceive.gyr_z
																	, BufferReceive.mag_x
																	, BufferReceive.mag_y
																	, BufferReceive.mag_z
																	, BufferReceive.temp
																	, BufferReceive.adc
										);
				NRF_LOG_INFO("[%s] de %d", s, n);
				ff_result = f_write(&file, s, n, (UINT *) &bytes_written); //writing data
				if (ff_result != FR_OK)
				{
					NRF_LOG_INFO("Write failed.");
				}
				else
				{
					NRF_LOG_INFO("%d bytes written.", bytes_written);
				}
			}
			else
			{
				NRF_LOG_INFO("Erreur lecture");
			}
			xTaskNotifyGive(xTwiHandle);
		} 
			// stop is set
				// closing file
			ff_result = f_close(&file);
			if (ff_result != FR_OK)
			{
				NRF_LOG_INFO("Close failed code: %d.", ff_result);
			}
			else
			{
				NRF_LOG_INFO("Close success.");
			}
			NRF_LOG_INFO("Exiting SD task");
			vTaskDelete(xSDHandle);
}
/*******************************************************************************************************
***********************************************************************************************************
*************************************************************************************************************/

/**
 * @fn twi_handler (void* pvParameter)
 * @brief TWI events handler.
 *
 * @param[in] p_event   const pointer for nrf_drb_twi_evt_t, allow to capt event.
 * @param[in] p_context void pointer 
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{ 		
    ret_code_t err_code;
    //static sample_t m_sample;
    
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

/*!
 * @fn twi_init (void)
 * @brief Initialise twi
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**
 * @fn init_imu (void)
 * @brief Initialization of all IMUs registers .
 */
void init_imu(void)
{
		int imu_id = 0;
		while(imu_id != NB_IMU)
			{
				//Init MPU6050 and HMC5883
				switch_imu(imu_id);
				init_MPU6050(m_twi);
				
				nrf_delay_ms(MPU_DELAY);
				
				init_HMC5883(m_twi);
				nrf_delay_ms(MPU_DELAY);
				imu_id ++;
				switch_imu(imu_id);				
			}			
}

/*!
 * @fn stat_pin_handler (nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
 * @brief wait button interruption and send notify to StartHandle to unlock it
 * 
 * @param[in] pin nrf_drv_gpiote_pin_t varaible
 * @param[in] action nrf_gpiote_polarity_t variable
 */
void start_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	xTaskNotifyGive(xStartHandle); 
}

/*!
 * @fn stop_pin_handler (nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
 * @brief wait button interruption and send notify to StopHandle to unlock it
 * 
 * @param[in] pin nrf_drv_gpiote_pin_t varaible
 * @param[in] action nrf_gpiote_polarity_t variable
 */
void stop_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	xTaskNotifyGive(xStopHandle);
}


/*!
 * @fn init_pin_interrupt (void)
 * @brief Init gpiote interruption for bouton 0 and 1
 */
void init_pin_interrupt (void)
{
		ret_code_t err_code;
	
		err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(BSP_BUTTON_0, &in_config, start_pin_handler);
    APP_ERROR_CHECK(err_code);
	
		err_code = nrf_drv_gpiote_in_init(BSP_BUTTON_1, &in_config, stop_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(BSP_BUTTON_0, true);
		nrf_drv_gpiote_in_event_enable(BSP_BUTTON_1, true);
}

/*!
 * @fn init()
 * @brief launch all init fonction
 */
void init()
{
	twi_init();
	SDcard_init();
	init_switch_pins();
	init_imu();
	init_RTC();
	
	init_pin_interrupt();
	nrf_delay_ms(100);
}

/*!
 * @fn main (void)
 * @brief main fonction which call init() / init debug / create all task / create mutex and queue / launch scheduler
 */
int main(void)
{				
		BaseType_t xReturned;
		// creating twi mutex 
		m_twi_mutex = xSemaphoreCreateMutex();
		
	  ASSERT(NULL != m_twi_mutex);
		
    ret_code_t err_code;
		// Configure LED-pins as outputs
//		bsp_board_leds_init();
//		bsp_board_buttons_init();

	
		//init debug
		APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
		NRF_LOG_DEFAULT_BACKENDS_INIT();
		NRF_LOG_INFO("Projet MMS");
		
		init();
		
		xQueue = xQueueCreate(2, sizeof(IMU));
	
    /* Initialize clock driver for better time accuracy in FREERTOS */
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
	  /* Create task for Starting all system */
		/*
		xReturned = xTaskCreate(RTC_task_function, "RTC", configMINIMAL_STACK_SIZE + 200, NULL, 2, &xRTCHandle);
		if (xReturned == pdPASS)
		{
			NRF_LOG_INFO("RTC task created");
		}
		else
		{
			NRF_LOG_INFO("Unable to create RTC task");
		}
		*/
		xReturned = xTaskCreate(start_task_function, "STR1", configMINIMAL_STACK_SIZE + 200, NULL, 2, &xStartHandle);
		if (xReturned == pdPASS)
		{
			NRF_LOG_INFO("Start task created");
		}
		else
		{
			NRF_LOG_INFO("Unable to create starting task");
		}
		
		xReturned = xTaskCreate(stop_task_function, "STP1", configMINIMAL_STACK_SIZE + 200, NULL, 1, &xStopHandle);
		if (xReturned == pdPASS)
		{
			NRF_LOG_INFO("Stop task created");
		}
		else
		{
			NRF_LOG_INFO("Unable to create stoping task");
		}
		
    // Create task for LED0 blinking with priority set to 2
		/*
		xReturned = xTaskCreate(led_toggle_task_function, "LED0", configMINIMAL_STACK_SIZE + 200, NULL, 4, &m_led_toggle_task_handle);
		if (xReturned == pdPASS)
		{
			NRF_LOG_INFO("Led task created");
		}
		else
		{
			NRF_LOG_INFO("Unable to create led task");
		}
		*/
		//Task reading IMUs 

		xReturned = xTaskCreate(vTwiFunction, "T1", configMINIMAL_STACK_SIZE + 200, (void*) xQueue, 4, &xTwiHandle );
		if (xReturned == pdPASS)
		{
			NRF_LOG_INFO("twi task created");
		}
		else
		{
			NRF_LOG_INFO("Unable to create twi task");
		}

		xReturned = xTaskCreate(vSDCardFunction, "SD1", configMINIMAL_STACK_SIZE + 200, (void*) xQueue, 3, &xSDHandle );
		if (xReturned == pdPASS)
		{
			NRF_LOG_INFO("SD task write created");
		}
		else
		{
			NRF_LOG_INFO("Unable to create SD task");
		}
/*
		xReturned = xTaskCreate(ADC_task_function, "ADC1", configMINIMAL_STACK_SIZE + 200, (void*) NULL, 3, &m_ADC_task_handle );
		if (xReturned == pdPASS)
		{
			NRF_LOG_INFO("ADC task write created");
		}
		else
		{
			NRF_LOG_INFO("Unable to create ADC task");
		}
*/
    /* Activate deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

//    /* Start FreeRTOS scheduler. */
		vTaskStartScheduler();

    while (true)
    {
				//_WFE();
        ASSERT(false);
        /* FreeRTOS should not be here... FreeRTOS goes back to the start of stack
         * in vTaskStartScheduler function. */
		
    }
}
/**
 * @}
 */
