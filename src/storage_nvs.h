#ifndef NVS_STORAGE_H
#define NVS_STORAGE_H

#include <zephyr/sys/timeutil.h>

#define NVS_DEVICE_ID 5
#define FLASH_SIZE      (2 * 1024 * 1024)    // 2 MB in bytes
#define FLASH_SECTOR_SIZE 4096               // 4 KB in bytes
#define FLASH_SECTOR_COUNT (FLASH_SIZE / FLASH_SECTOR_SIZE)
#define FLASH_PAGE_SIZE 256
#define STORAGE_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(storage_partition)


int read_uniqueidentifier(uint16_t *identifier);
int write_uniqueIdentifier(uint16_t *identifier);

#endif