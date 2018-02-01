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

/*!
 * @brief Set pin use by the SDcard breakout board (see module config for more information)
 */
#define SDC_SCK_PIN     ARDUINO_13_PIN  /*!< SDC serial clock (SCK) pin.*/
#define SDC_MOSI_PIN    ARDUINO_11_PIN  /*!< SDC serial data in (DI) pin.*/
#define SDC_MISO_PIN    ARDUINO_12_PIN  /*!< SDC serial data out (DO) pin.*/
#define SDC_CS_PIN      ARDUINO_10_PIN  /*!< SDC chip select (CS) pin.*/

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
* @fn convert (int)
* @brief convertion function to adapt to ASCII (unused)
* @param[in] a int variable to convert
*
* @todo delete function?
*/
char convert(int a);


