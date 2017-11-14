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
/** @file
 * @defgroup blinky_rtc_freertos_example_main main.c
 * @{
 * @ingroup blinky_rtc_freertos_example
 *
 * @brief Blinky FreeRTOS Example Application main file.
 *
 * This file contains the source code for a sample application using FreeRTOS to blink LEDs.
 *
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
 * @brief Reference to LED0 toggling FreeRTOS task.
 */
//static TaskHandle_t  m_led_toggle_task_handle;

/**
 * @brief Reference to Twi IMU reading task.
 */
TaskHandle_t  xTwiHandle;

/**
 * @brief Reference to Reading the Queue task.
 */
TaskHandle_t  xQHandleRead;

/**
 * @brief Reference to Writing into the Queue task.
 */
TaskHandle_t  xQHandleWrite;

/**
 * @brief Reference to Writing into the SD task.
 */
TaskHandle_t  xSDHandle;

/**
 * @brief Reference to the Queue where the datas are stocked
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

/**
 * @brief TWI instance
 *
 * Instance of the TWI
 */
static nrf_drv_twi_t const m_twi = NRF_DRV_TWI_INSTANCE(1);

/* MPU accelerometer register tab */
uint8_t mpu_acc_reg[NB_ACC_REG_MPU] = {REG_ACC_XH, REG_ACC_XL, REG_ACC_YH, REG_ACC_YL, REG_ACC_ZH, REG_ACC_ZL};

/* MPU gyrometer register tab */
uint8_t mpu_gyr_reg[NB_GYR_REG_MPU] = {REG_GYR_XH, REG_GYR_XL, REG_GYR_YH, REG_GYR_YL, REG_GYR_ZH, REG_GYR_ZL};

/* MPU thermometer register tab */
uint8_t mpu_temp_reg[NB_TEMP_REG_MPU] = {REG_TEMP_H, REG_TEMP_L};

/* HMC register tab */ 
uint8_t hmc_reg[NB_REG_HMC] = {HMC_XH, HMC_XL, HMC_YH, HMC_YL, HMC_ZH, HMC_ZL};

/**
 * @brief vTwiFunction prototype
 *
 * Prototype of the function reading all IMUs
 */
void vTwiFunction (void *pvParameter);

/**
 * @brief vQueueRead prototype
 *
 * Prototype of the function for reading all IMUs datas
 */
void vQueueRead(void* pvParameter);


static void blink_rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    BaseType_t yield_req = pdFALSE;
    ret_code_t err_code;
    bsp_board_led_invert(BSP_BOARD_LED_1);
    err_code = nrf_drv_rtc_cc_set(
        &m_rtc,
        BLINK_RTC_CC,
        (nrf_rtc_cc_get(m_rtc.p_reg, BLINK_RTC_CC) + BLINK_RTC_TICKS) & RTC_COUNTER_COUNTER_Msk,
        true);
    APP_ERROR_CHECK(err_code);

   /* The returned value may be safely ignored, if error is returned it only means that
    * the semaphore is already given (raised). */
   UNUSED_VARIABLE(xSemaphoreGiveFromISR(m_led_semaphore, &yield_req));
   portYIELD_FROM_ISR(yield_req);
}

/**
 * @brief LED0 task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
static void led_toggle_task_function (void * pvParameter)
{
    ret_code_t err_code;	
    err_code = nrf_drv_rtc_init(&m_rtc, &m_rtc_config, blink_rtc_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_rtc_cc_set(&m_rtc, BLINK_RTC_CC, BLINK_RTC_TICKS, true);
    APP_ERROR_CHECK(err_code);
    nrf_drv_rtc_enable(&m_rtc);

    m_led_semaphore = xSemaphoreCreateBinary();
    ASSERT(NULL != m_led_semaphore);

    UNUSED_PARAMETER(pvParameter);
    while (true)
    {
        bsp_board_led_invert(BSP_BOARD_LED_0);

        /* Wait for the event from the RTC */
        UNUSED_RETURN_VALUE(xSemaphoreTake(m_led_semaphore, portMAX_DELAY));

    }

    /* Tasks must be implemented to never return... */
}

void vQueueRead(void* pvParameter)
{	
		QueueHandle_t xQueue = (QueueHandle_t) pvParameter;
		
		//UNUSED_PARAMETER(pvParameter);
		uint16_t BufferReceive = 0;
		BaseType_t xReadResult;
		BaseType_t QueueItems;
		const TickType_t xDelay = 50;
	
		for ( ;; )
	{
		// wait for the notfy from twifunction
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
		/* Read values from the queue */
		
		QueueItems = uxQueueMessagesWaiting(xQueue);
		NRF_LOG_INFO("	Queue Items:%d", QueueItems);
		while(QueueItems > 0)
		{
			xReadResult = xQueueReceive(xQueue, &BufferReceive, (TickType_t) 10);		
			if (xReadResult == pdTRUE)
			{
				NRF_LOG_INFO("	valeur recue:%d", BufferReceive);
			}
			else
			{
				NRF_LOG_INFO("Erreur lecture");
			}
			QueueItems = uxQueueMessagesWaiting(xQueue);
			NRF_LOG_INFO("	Queue Items:%d", QueueItems);
		}		
		vTaskDelay(xDelay);
	}		
}	

/**
 * @brief Twi IMUs reading task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
void vTwiFunction (void *pvParameter)
{		
		QueueHandle_t xQueue = (QueueHandle_t) pvParameter;
		const TickType_t xDelay = 200;
		//UNUSED_PARAMETER(pvParameter);		
		//buffer for accelerometer values
		uint8_t acc[6] = {0};
		//buffer for gyrometer values
		uint8_t gyr[6] = {0};
		//buffer for magnetometer values
		uint8_t mag[6] = {0};
		//buffer for temperature values
		uint8_t temp[2] = {0};
		/* IMU struct creation */
		IMU imu_list[NB_IMU];
		
		int j = 0; //imu id index
		int i = 0; //imu register id
		
		for(;;)
		{
// Get values from accelerometer **********************************************
// Point at register
			//protecting the twi before transaction
			xSemaphoreTake(m_twi_mutex, portMAX_DELAY);
			nrf_drv_twi_tx(&m_twi, MPU_ADDR, &mpu_acc_reg[i], 1, false);
			nrf_delay_us(MPU_DELAY_US);
			// read register
			nrf_drv_twi_rx(&m_twi, MPU_ADDR, &acc[i], 1);
			nrf_delay_us(MPU_DELAY_US);
			xSemaphoreGive(m_twi_mutex);
			
			
// Get values from gyrometer **********************************************
// Point at register
			//protecting the twi before transaction
			xSemaphoreTake(m_twi_mutex, portMAX_DELAY);
			nrf_drv_twi_tx(&m_twi, MPU_ADDR, &mpu_gyr_reg[i], 1, false);
			nrf_delay_us(MPU_DELAY_US);
			// read register	
			nrf_drv_twi_rx(&m_twi, MPU_ADDR, &gyr[i], 1);
			nrf_delay_us(MPU_DELAY_US);
			xSemaphoreGive(m_twi_mutex);
			
			
// Get values from magnetomter **********************************************
// Point at register
			//protecting the twi before transaction
			xSemaphoreTake(m_twi_mutex, portMAX_DELAY);
			nrf_drv_twi_tx(&m_twi, HMC_ADDR, &hmc_reg[i], 1, false);
			nrf_delay_us(MPU_DELAY_US);
			// read register	
			nrf_drv_twi_rx(&m_twi, HMC_ADDR, &mag[i], 1);
			nrf_delay_us(MPU_DELAY_US);
			xSemaphoreGive(m_twi_mutex);
	
		
// Get values from temperature  **********************************************
// Point at register
		if ( i < 2)
		{
			//protecting the twi before transaction
			xSemaphoreTake(m_twi_mutex, portMAX_DELAY);
			nrf_drv_twi_tx(&m_twi, MPU_ADDR, &mpu_temp_reg[i], 1, false);
			nrf_delay_us(MPU_DELAY_US);
			// read register
			nrf_drv_twi_rx(&m_twi, MPU_ADDR, &temp[i], 1);
			nrf_delay_us(MPU_DELAY_US);
			xSemaphoreGive(m_twi_mutex);
			i++;
		}
		else if (i == 5)
			{
				// Accelerometer X
				imu_list[j].acc_x = (acc[0]<<8)|acc[1]; 
				xQueueSendToBack(xQueue, (void*) &imu_list[j].acc_x, ( TickType_t ) 10);
				NRF_LOG_INFO("	valeur envoyee:%d", imu_list[j].acc_x );
				// Accelerometer Y
				imu_list[j].acc_y = (acc[2]<<8)|acc[3];
				xQueueSendToBack(xQueue, (void*) &imu_list[j].acc_y, ( TickType_t ) 10);
				// Accelerometer Z
				imu_list[j].acc_z = (acc[4]<<8)|acc[5];
				xQueueSendToBack(xQueue, (void*) &imu_list[j].acc_z, ( TickType_t ) 10);
				// Gyrometer X
				imu_list[j].gyr_x = (gyr[0]<<8)|gyr[1];
				xQueueSendToBack(xQueue, (void*) &imu_list[j].gyr_x, ( TickType_t ) 10);
				// Gyrometer Y
				imu_list[j].gyr_y = (gyr[2]<<8)|gyr[3];
				xQueueSendToBack(xQueue, (void*) &imu_list[j].gyr_y, ( TickType_t ) 10);
				// Gyrometer Z
				imu_list[j].gyr_z = (gyr[4]<<8)|gyr[5];
				xQueueSendToBack(xQueue, (void*) &imu_list[j].gyr_z, ( TickType_t ) 10);
				// Magnetometer X
				imu_list[j].mag_x = (mag[0]<<8)|mag[1];
				xQueueSendToBack(xQueue, (void*) &imu_list[j].mag_x, ( TickType_t ) 10);
				// Magnetometer Z
				imu_list[j].mag_y = (mag[2]<<8)|mag[3];
				xQueueSendToBack(xQueue, (void*) &imu_list[j].mag_y, ( TickType_t ) 10);
				// Magnetometer Y
				imu_list[j].mag_z = (mag[4]<<8)|mag[5];
				xQueueSendToBack(xQueue, (void*) &imu_list[j].mag_z, ( TickType_t ) 10);
				//Temperature
				imu_list[j].temp = (temp[0]<<8)|temp[1];
				xQueueSendToBack(xQueue, (void*) &imu_list[j].temp, ( TickType_t ) 10);
				
				i = 0;
				
				NRF_LOG_INFO("IMU n_%d data:", j)
				NRF_LOG_INFO("	acc_x:%d", imu_list[j].acc_x);
				NRF_LOG_INFO("	acc_y:%d", imu_list[j].acc_y);
				NRF_LOG_INFO("	acc_z:%d", imu_list[j].acc_z);
				NRF_LOG_INFO("	gyr_x:%d", imu_list[j].gyr_x);
				NRF_LOG_INFO("	gyr_y:%d", imu_list[j].gyr_y);
				NRF_LOG_INFO("	gyr_z:%d", imu_list[j].gyr_z);
				NRF_LOG_INFO("	mag_x:%d", imu_list[j].mag_x);
				NRF_LOG_INFO("	mag_y:%d", imu_list[j].mag_y);
				NRF_LOG_INFO("	mag_z:%d", imu_list[j].mag_z);
				NRF_LOG_INFO("	temp:%d °C", ((imu_list[j].temp)/340)+36.53);
				
				// Send notify to unlock QueueRead function
				xTaskNotifyGive(xQHandleRead);
				
				if(j == NB_IMU-1)
				{
					j = 0;					
				}
				else
				{					
					j++;					
				}
				NRF_LOG_INFO("swithing to imu_%d", j);
				switch_imu(j);
				
			}
			else
			{
				i++;				
			}
			vTaskDelay(xDelay);
		}
}

void vQueueWrite(void* pvParameter)
{		
		QueueHandle_t xQueue = (QueueHandle_t) pvParameter;
		//UNUSED_PARAMETER(pvParameter);
		uint16_t Cpt = 0;
		const TickType_t xDelay = 200;
	for ( ;; )
	{
			// Write values to the queue 
		xQueueSendToBack(xQueue, (void*) &Cpt, ( TickType_t ) 10);
		NRF_LOG_INFO("	valeur envoyee:%d", Cpt);
		vTaskDelay(xDelay);
		Cpt ++;
	}		
}	


/*******************************************************************************************************
********************************************************************************************************
*******************************************************************************************************/

static void vSDCardFunction (void *pvParameter)
{
		SemaphoreHandle_t xSemaphore;
		xSemaphore = xSemaphoreCreateMutex();
		//static DIR dir;
		//static FILINFO fno;
		static FIL file;
		FRESULT ff_result;
		uint32_t bytes_written;
		//uint32_t bytes_to_read;
		char* s = "test d'ecriture";
	
		for(;;)
		{
			if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
			{
				NRF_LOG_INFO ("Opening file: \n");
				ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
				NRF_LOG_INFO("Error code: %d\r\n", ff_result);
				if (ff_result != FR_OK)
				{
					NRF_LOG_WARNING("Unable to open file: " FILE_NAME ".");
				}else{
					ff_result = f_write(&file, s, strlen(s), (UINT *) &bytes_written);
					if (ff_result != FR_OK)
					{
						NRF_LOG_INFO("Write failed\r\n.");
					}
					else
					{
						NRF_LOG_INFO("%d bytes written.", bytes_written);
					}
				}
				ff_result = f_close(&file);
				xSemaphoreGive( xSemaphore );
			}
		}
}
/*******************************************************************************************************
***********************************************************************************************************
*************************************************************************************************************/

/**
 * @brief TWI events handler.
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
 * @brief Initialization of all IMUs registers .
 */
void init_imu(void)
{
		int imu_id = 0;
		while(imu_id != NB_IMU)
			{
				//NRF_LOG_INFO("initialising IMU n_%d", init);
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

/*******************************************************************************************************
********************************************************************************************************
********************************************************************************************************/

void init()
{
	SDcard_init();
	init_switch_pins();
	init_imu();	
}

int main(void)
{				
		BaseType_t xReturned;
		// creating twi mutex 
		m_twi_mutex = xSemaphoreCreateMutex();
		
	  ASSERT(NULL != m_twi_mutex);
		
    ret_code_t err_code;
		// Configure LED-pins as outputs
		bsp_board_leds_init();
	
		//init debug
		APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
		NRF_LOG_DEFAULT_BACKENDS_INIT();
		NRF_LOG_INFO("Projet MMS");
		
		twi_init();
		init();
	
		xQueue = xQueueCreate(10, sizeof(uint16_t));
	
    /* Initialize clock driver for better time accuracy in FREERTOS */
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
		
		init_switch_pins();

    /* Create task for LED0 blinking with priority set to 2 */    
		/*xReturned = xTaskCreate(led_toggle_task_function, "LED0", configMINIMAL_STACK_SIZE + 200, NULL, 1, &m_led_toggle_task_handle);
		if (xReturned == pdPASS)
		{
			NRF_LOG_INFO("Led task created");
		}
		else
		{
			NRF_LOG_INFO("Unable to create led task");
		}
		*/
		
		/* Task reading IMUs */
		/*xReturned = xTaskCreate(vTwiFunction, "T1", configMINIMAL_STACK_SIZE + 200, (void*) xQueue, 1, &xTwiHandle );
		if (xReturned == pdPASS)
		{
			NRF_LOG_INFO("twi task created");
		}
		else
		{
			NRF_LOG_INFO("Unable to create twi task");
		}
		// task that read the queue 
		
		xReturned = xTaskCreate(vQueueRead, "Q1", configMINIMAL_STACK_SIZE + 200, (void*) xQueue, 3, &xQHandleRead );
		if (xReturned == pdPASS)
		{
			NRF_LOG_INFO("Queue Read task created");
		}
		else
		{
			NRF_LOG_INFO("Unable to create queue read task");
		}*/
		/*
		xReturned = xTaskCreate(vQueueWrite, "Q2", configMINIMAL_STACK_SIZE + 200, (void*) xQueue, 1, &xQHandleWrite );
		if (xReturned == pdPASS)
		{
			NRF_LOG_INFO("Queue task write created");
		}
		else
		{
			NRF_LOG_INFO("Unable to create queue write task");
		}
		*/
		xReturned = xTaskCreate(vSDCardFunction, "SD1", configMINIMAL_STACK_SIZE + 200, NULL, 1, &xSDHandle );
		if (xReturned == pdPASS)
		{
			NRF_LOG_INFO("SD task write created");
		}
		else
		{
			NRF_LOG_INFO("Unable to create SD task");
		}
	
    /* Activate deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Start FreeRTOS scheduler. */
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
