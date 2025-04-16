#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>


#include <zephyr/pm/device.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "storage_nvs.h"

static struct nvs_fs fs;

LOG_MODULE_REGISTER(STORAGE_MODULE, CONFIG_LOG_DEFAULT_LEVEL);

const struct device *spi_flash_dev;

int write_uniqueIdentifier(uint16_t *identifier) {
	nvs_write(&fs, NVS_DEVICE_ID, identifier, sizeof(uint16_t) );
	return 0;
}

int read_uniqueidentifier(uint16_t *identifier) {
	nvs_read(&fs, NVS_DEVICE_ID, identifier, sizeof(uint16_t) );
	return 0;
}

static int flash_set_suspend() {  
	int ret = pm_device_action_run(spi_flash_dev, PM_DEVICE_ACTION_SUSPEND);
	if(ret) { 
		LOG_ERR("could not suspend qspi flash device %d, %s", ret, spi_flash_dev->name);
		return ret;	
	}
	
	LOG_INF("qspi suspended");
	// ret = pm_device_action_run(spi_flash_dev, PM_DEVICE_ACTION_TURN_OFF);
	// if(ret) {
	// 	LOG_ERR("could not turn of qspi flash, %d, %s", ret, spi_flash_dev->name);
	// 	return ret;
	// }		
	// LOG_INF("qspi turned off");
	return 0;
}

int init_external_storage(void) {	
	LOG_INF("init storage");
    int rc = 0;	
	
	spi_flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));	
	if (!device_is_ready(spi_flash_dev)) {
		LOG_ERR("%s: device not ready.\n", spi_flash_dev->name);
		return rc;
	}
	
    struct flash_pages_info info;

	LOG_INF("flash device found: %s", spi_flash_dev->name);	
	
	rc = flash_get_page_info_by_offs(spi_flash_dev, 0, &info);
	if (rc) {
		LOG_ERR("Unable to get page info\n");
		return rc;
	}
		
	rc |= flash_set_suspend();	
	return rc;
}


int init_internal_storage() {
    
	LOG_INF("init internal storage");
	const struct device *flash_dev = STORAGE_PARTITION_DEVICE;
	
	if (!device_is_ready(flash_dev)) {
		LOG_ERR("%s: device not ready.\n", flash_dev->name);
		return ENODEV;
	}
	fs.flash_device = flash_dev;
    int rc = 0;	

    struct flash_pages_info info;

	LOG_INF("flash device found: %s", fs.flash_device->name);
		
	fs.offset = 0xfe000; 			
	
	rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (rc) {
		LOG_ERR("Unable to get page info\n");
		return -1;
	}
  

	fs.sector_size = info.size;
	fs.sector_count = 0x2000 / info.size; 

	// todo:implement nvs
	rc = nvs_mount(&fs);
	LOG_INF("fs sector size = %d", info.size);
	if (rc) {
		LOG_ERR("Flash Init failed\n");
		return -1;
	}  		

	return rc;
}

int init_storage_module() {
	int err = 0;
	err |= init_external_storage();
	if(err) {
		LOG_ERR("init storage failed");
	}
	err |= init_internal_storage();
	if(err) {
		LOG_ERR("init internal storage failed");
	}
	return err;
}

SYS_INIT(init_storage_module, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);