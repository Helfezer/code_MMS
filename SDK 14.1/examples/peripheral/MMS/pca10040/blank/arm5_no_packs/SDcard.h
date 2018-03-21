/*!
* @file SDcard.h
* @brief Header for SDcard fonctions
* @author Pantostier Quentin / Meissburger Jordan / Rakotovao Sebastien
* @version 1.0 
*/
#include "nrf.h"
#include "bsp.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SDC_SCK_PIN     ARDUINO_13_PIN  
#define SDC_MOSI_PIN    ARDUINO_11_PIN  
#define SDC_MISO_PIN    ARDUINO_12_PIN  
#define SDC_CS_PIN      ARDUINO_10_PIN  

/**
 * @brief  SDC block device definition (memory block on card). Config to use previous pin definition for spi communication. 
 * */
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

/*!
* @fn SDcard_init (void)
* @brief init function of SDcard devise
*/
void SDcard_init(void);

/*!
 * @fn SD_write (FIL* file, const void* buff_to_write, UINT number_char_to_write, UINT* bytes_written)
 * @brief function to write on the SDcard
 * @param the same as the fatfs write function
 * @return FRESULT as in fatfs
 */
FRESULT SD_write(FIL* file, const void* buff_to_write, UINT number_char_to_write, UINT* bytes_written);



