#include "SDcard.h"

extern uint8_t failed;
uint32_t bytes_to_read;

void SDcard_init()
{
		static DIR dir;
    static FILINFO fno;
    static FATFS fs;
		FRESULT ff_result;
    DSTATUS disk_state = STA_NOINIT;
		
    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    NRF_LOG_INFO("Initializing disk 0 (SDC)...");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        NRF_LOG_INFO("Disk initialization failed.");
				failed = 1;
				return;
    }
		else
		{
			failed = 0;
		}

    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    NRF_LOG_INFO("Capacity: %d MB", capacity);

    NRF_LOG_INFO("Mounting volume...");
    ff_result = f_mount(&fs, "", 1);
		//NRF_LOG_INFO("Error code: %d\r\n", ff_result);
    if (ff_result)
    {
        NRF_LOG_INFO("Mount failed.");
        failed = 1;
				return;
    }
		else
		{
			failed = 0;
		}
		
		NRF_LOG_INFO("\r\n Listing directory: /");
    ff_result = f_opendir(&dir, "/");
    if (ff_result)
    {
        NRF_LOG_INFO("Directory listing failed!");
        return;
    }

    do
    {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            NRF_LOG_INFO("Directory read failed.");
            return;
        }

        if (fno.fname[0])
        {
            if (fno.fattrib & AM_DIR)
            {
                NRF_LOG_RAW_INFO("   <DIR>   %s",(uint32_t)fno.fname);
            }
            else
            {
                NRF_LOG_RAW_INFO("%9lu  %s \n", fno.fsize, (uint32_t)fno.fname);
								if (fno.fname == "A.TXT")
									bytes_to_read = fno.fsize;
            }
        }
    }
    while (fno.fname[0]);
    NRF_LOG_RAW_INFO("");
}

char convert(int var)
{
	char c= (char) var;
	return c;
}

