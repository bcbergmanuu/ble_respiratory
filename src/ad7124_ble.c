/*!
 *****************************************************************************
  @file:  ad7124_console_app.c

  @brief: Implementation for the menu functions that handle the AD7124

  @details:
 -----------------------------------------------------------------------------
Copyright (c) 2019 Analog Devices, Inc.  All rights reserved.
*/

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>                                                                                                                                                     
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>


#include "ad7124.h"
#include "ad7124_regs.h"
#include "ad7124_support.h"

#include "ad7124_ble.h"
#include "config_respiratory.h"

#define SPI_OP  SPI_OP_MODE_MASTER |SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE


LOG_MODULE_REGISTER(AD7124_BLE, LOG_LEVEL_INF);

#define AD7124_CHANNEL_COUNT 16

#define SHOW_ALL_CHANNELS     false
#define SHOW_ENABLED_CHANNELS  true

#define DISPLAY_DATA_TABULAR    0
#define DISPLAY_DATA_STREAM     1

#define SPIOP      SPI_WORD_SET(8) | SPI_TRANSFER_MSB

const struct spi_dt_spec spiDevice = SPI_DT_SPEC_GET(DT_NODELABEL(gendev), SPIOP, 0);
static void start_ad7124(struct k_work *ptr_work);
/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  bufr      pointer to data to read in register reg
 * @param  bufw      pointer to data to write in register reg
 * @param  len       number of consecutive bytes to write
 *
 */

 //primary custom service 
 static struct bt_uuid_128 uuid_ad7124_prim = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xa6c47c93, 0x1119, 0x4ca5, 0x176e, 0xdffb871f11f0));

//unique device
// static struct bt_uuid_128 uuid_identifier = BT_UUID_INIT_128(
// 	BT_UUID_128_ENCODE(0x56a331ec, 0x0319, 0x493e, 0xbd4d, 0xe778c69badd7));

//current memory position
static struct bt_uuid_128 uuid_data = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x84b45e35, 0x140b, 0x4d33, 0x92c6, 0x386d8bff160d));

// static uint8_t uniqueIdentifier_value[sizeof(uint16_t)];

// static ssize_t read_identifier(struct bt_conn *conn, const struct bt_gatt_attr *attr,
// 		void *buf, uint16_t len, uint16_t offset) {

// 	uint16_t uniqueId = 0;
// 	read_uniqueidentifier(&uniqueId);
// 	memcpy(uniqueIdentifier_value, &uniqueId, sizeof(uint16_t));

// 	const uint8_t *value = attr->user_data;	

// 	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(uniqueIdentifier_value));
// }

static int32_t do_continuous_conversion();
static int32_t do_fullscale_calibration();

// static ssize_t write_identifier(struct bt_conn *conn, const struct bt_gatt_attr *attr,
// 	const void *buf, uint16_t len, uint16_t offset,
// 	uint8_t flags) {

// 	uint8_t *value = attr->user_data;

// 	if (offset + len > sizeof(uniqueIdentifier_value)) {
// 		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
// 	}
// 	memcpy(value + offset, buf, len);

// 	uint16_t uniqueId = 0;
// 	memcpy(&uniqueId, value, sizeof(uint16_t));
// 	write_uniqueIdentifier(&uniqueId);

// 	return len;
// }

#define ble_buff_size (5*sizeof(uint32_t))
static uint8_t ad7124_ble_buff[ble_buff_size]; //size of packed protobuf


static ssize_t read_ad_buffer(struct bt_conn *conn, const struct bt_gatt_attr *attr,
	void *buf, uint16_t len, uint16_t offset)
{	

	const uint8_t *value = attr->user_data;	

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(ad7124_ble_buff));
}

K_WORK_DEFINE(continuous_conversion_task, start_ad7124);

K_THREAD_STACK_DEFINE(my_stack_area, 1024);

struct k_work_q my_work_q;



static uint8_t notify_ad_buffer_on = 0;

static void ble_ad_buffer_notify_changed(const struct bt_gatt_attr *attr, uint16_t value) {
	ARG_UNUSED(attr);
	LOG_INF("notify_changed: %d", value);
	notify_ad_buffer_on = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
	if(value == BT_GATT_CCC_NOTIFY)	{
		LOG_INF("starting conversion task");
		k_work_submit_to_queue(&my_work_q, &continuous_conversion_task);		
		LOG_INF("conversion task started");
	}
}


void ble_notify_adbuffer_proc(struct k_work *ptrWorker);

K_WORK_DEFINE(ble_notify_task, ble_notify_adbuffer_proc);



/* AD7124 readout primary Service Declaration */
BT_GATT_SERVICE_DEFINE(ad7124_svc,
	BT_GATT_PRIMARY_SERVICE(&uuid_ad7124_prim),
	
	// BT_GATT_CHARACTERISTIC(&uuid_identifier.uuid,
	// 		       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
	// 		       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
	// 		       read_identifier, write_identifier, uniqueIdentifier_value),	

	BT_GATT_CHARACTERISTIC(&uuid_data.uuid, 
	 		       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
	 		       BT_GATT_PERM_READ,
	 		       read_ad_buffer, NULL, ad7124_ble_buff),
	BT_GATT_CCC(ble_ad_buffer_notify_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/// @brief Notify BLE when data ready
/// @param ptrWorker 
void ble_notify_adbuffer_proc(struct k_work *ptrWorker) {					
	if(!notify_ad_buffer_on) return;
	
	struct bt_gatt_attr *notify_attr = bt_gatt_find_by_uuid(ad7124_svc.attrs, ad7124_svc.attr_count, &uuid_data.uuid);	
	struct bt_gatt_notify_params params = {
        .attr = notify_attr,
        .data = ad7124_ble_buff,
        .len  = sizeof(ad7124_ble_buff),
      //  .func = notify_complete_cb, 
    };

	int ret = bt_gatt_notify_cb(NULL, &params);
	if(ret < 0) {
		LOG_ERR("error notify data %d", ret);
	}
}

//advertising data packet
const struct bt_data ad[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_BAS_VAL), BT_UUID_16_ENCODE(BT_UUID_CTS_VAL)),
};

static void start_ad7124(struct k_work *ptr_work) {
	int err =0;
	err |= do_fullscale_calibration();
	err |= do_continuous_conversion();
	if(err) {
		LOG_ERR("error start ad %d", err);		
	}	
}



static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_INF("Connection failed (err 0x%02x)\n", err);
	} else {		
		LOG_INF("Connected\n");		
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason 0x%02x)\n", reason);	
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,	
};


static void bt_ready(void)
{
	int err;

	LOG_INF("Bluetooth initialized\n");	

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)\n", err);
		return;
	}

	LOG_INF("Advertising successfully started\n");
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = NULL,// auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = NULL,
	.passkey_confirm = NULL,	
};

static int32_t platform_transceive(void *handle, uint8_t *bufw, uint8_t *bufr,
                              size_t len)
{  	
	int err = 0;
	struct spi_buf tx_spi_buf		= {.buf = bufw, .len = len};
	struct spi_buf_set tx_spi_buf_set 	= {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf rx_spi_bufs 		= {.buf = bufr, .len = len};
	struct spi_buf_set rx_spi_buf_set	= {.buffers = &rx_spi_bufs, .count = 1};
	
	err |= spi_transceive_dt(&spiDevice, &tx_spi_buf_set, &rx_spi_buf_set);
	if (err < 0) {
		LOG_ERR("spi_transceive_dt() failed, err: %d", err);
	}
	return err;	
}

static void platform_delay(uint32_t ms) {	
	k_sleep(K_MSEC(ms));
}


/*
 * This is the 'live' AD7124 register map that is used by the driver
 * the other 'default' configs are used to populate this at init time
 */
static struct ad7124_st_reg ad7124_register_map[AD7124_REG_NO];

// Pointer to the struct representing the AD7124 device
static struct ad7124_dev pAd7124_dev = {0};

// Public Functions

/*!
 * @brief      Initialize the AD7124 device and the SPI port as required
 *
 * @details    This resets and then writes the default register map value to
 *  		   the device.
 */
int32_t ad7124_app_initialize(uint8_t configID)
{
	/*
	 * Copy one of the default/user configs to the live register memory map
	 * Requirement, not checked here, is that all the configs are the same size
	 */	

	switch(configID) {
		case AD7124_CONFIG_A:
		{
			memcpy(ad7124_register_map, ad7124_regs_config_b, sizeof(ad7124_register_map));
			break;
		}	
	}
	pAd7124_dev.tranceiver = platform_transceive;	
	pAd7124_dev.ptDelay = platform_delay;

	// Used to create the ad7124 device
    pAd7124_dev.regs = ad7124_register_map,
  	pAd7124_dev.spi_rdy_poll_cnt = 	10000; // Retry count for polling
  	
	int ret = ad7124_setup(&pAd7124_dev);

  	return ret;
}

// /*!
//  * @brief      reads and displays the status register on the AD7124
//  *
//  * @details
//  */
// static void read_status_register(void)
// {
// 	if (ad7124_read_register(pAd7124_dev, &pAd7124_dev->regs[AD7124_Status]) < 0) {
// 	   LOG_ERR("\r\nError Encountered reading Status register\r\n");
// 	} else {
// 	    uint32_t status_value = (uint32_t)pAd7124_dev->regs[AD7124_Status].value;
//         LOG_ERR("\r\nRead Status Register = 0x%d\r\n", status_value);
// 	}
// }

static int32_t set_idle_mode() {
	int32_t error_code = 0;
	pAd7124_dev.regs[AD7124_ADC_Control].value &= ~(AD7124_ADC_CTRL_REG_MODE(0xf)); //clear mode bits	
	pAd7124_dev.regs[AD7124_ADC_Control].value |= AD7124_ADC_CTRL_REG_MODE(4); //idle mode
	if ( (error_code = ad7124_write_register(&pAd7124_dev, pAd7124_dev.regs[AD7124_ADC_Control]) ) < 0) {
		LOG_ERR("Error (%d) setting AD7124 power mode to low.\r\n", error_code);				
	} else {
		LOG_INF("idle mode activated\n");
	}
	return error_code;
}

#define conversionBufSize 5
uint32_t conversionBuffer[conversionBufSize];


/*!
 * @brief      Continuously acquires samples in Continuous Conversion mode
 *
 * @details   The ADC is run in continuous mode, and all samples are acquired
 *            and assigned to the channel they come from. Escape key an be used
 *            to exit the loop
 */
static int32_t do_continuous_conversion()
{
	uint8_t conversionCounter = 0;
	int32_t error_code = 0;
	int32_t sample_data = 0;		
	
	//select continuous convertion mode, all zero
	pAd7124_dev.regs[AD7124_ADC_Control].value &= ~(AD7124_ADC_CTRL_REG_MODE(0xf));

	
	//select full power
	pAd7124_dev.regs[AD7124_ADC_Control].value &= ~(AD7124_ADC_CTRL_REG_POWER_MODE(0x3));
	pAd7124_dev.regs[AD7124_ADC_Control].value |= AD7124_ADC_CTRL_REG_POWER_MODE(0x2);

	error_code = ad7124_write_register(&pAd7124_dev, pAd7124_dev.regs[AD7124_ADC_Control]);
	if(error_code) {
		LOG_ERR("Error (%d) setting AD7124 Continuous conversion mode.\n", error_code);			
		return error_code;
	} 

	uint8_t channel_read = 0;
	// Continuously read the channels, and store sample values		
    while (notify_ad_buffer_on) {
		/*
		*  this polls the status register READY/ bit to determine when conversion is done
		*  this also ensures the STATUS register value is up to date and contains the
		*  channel that was sampled as well.
		*  Generally, no need to read STATUS separately, but for faster sampling
		*  enabling the DATA_STATUS bit means that status is appended to ADC data read
		*  so the channel being sampled is read back (and updated) as part of the same frame
		*/
		if ( (error_code = ad7124_wait_for_conv_ready(&pAd7124_dev, 10000)) < 0) {
				LOG_ERR("Error/Timeout waiting for conversion ready %d\n", error_code);
				return -1;
			}
		channel_read = pAd7124_dev.regs[AD7124_Status].value & 0x0000000F;
		if(pAd7124_dev.regs[AD7124_Channel_0 + channel_read].value & AD7124_CH_MAP_REG_CH_ENABLE) {

			if ( (error_code = ad7124_read_data(&pAd7124_dev, &sample_data)) < 0) {
				LOG_ERR("Error reading ADC Data (%d).\r\n", error_code);
				return -1;
			}
			
			if (!channel_read == 0) {	
			
				LOG_INF("Label");
			}
									
			LOG_INF("%i", sample_data);			
						
			conversionBuffer[conversionCounter] = sample_data;
			conversionCounter++;
			if(conversionCounter >= conversionBufSize) {
				conversionCounter = 0;
				//prepare data
				memcpy(ad7124_ble_buff, conversionBuffer, conversionBufSize * sizeof(uint32_t));
				//start notify task
				k_work_submit(&ble_notify_task);
			}
			
		}				
	}    

	error_code = set_idle_mode();
	if (error_code < 0) LOG_ERR("error occured continuous conversion");
	return error_code;
}



static int32_t set_zero_scale_calibration() {
	// 5 = system zero scale calibration
	int32_t error_code = 0;
	pAd7124_dev.regs[AD7124_ADC_Control].value &= ~(AD7124_ADC_CTRL_REG_MODE(0xf) | AD7124_ADC_CTRL_REG_POWER_MODE(0x3));
	pAd7124_dev.regs[AD7124_ADC_Control].value |= AD7124_ADC_CTRL_REG_MODE(0b0111) | AD7124_ADC_CTRL_REG_POWER_MODE(0x01);
	error_code = ad7124_write_register(&pAd7124_dev, pAd7124_dev.regs[AD7124_ADC_Control]);
	if(error_code) {
		LOG_ERR("Error (%d) setting AD7124 ADC into zero scale calibration.\r\n", error_code);		
		return error_code;
	}
	LOG_INF("zero scale calibration completed\n");	
	return error_code;
}

// static int32_t set_full_scale_calibration() {
// 	// 6 = system full scale calibration
// 	int32_t error_code = 0;
// 	pAd7124_dev->regs[AD7124_ADC_Control].value &= ~(AD7124_ADC_CTRL_REG_MODE(0xf) | AD7124_ADC_CTRL_REG_POWER_MODE(0x3) );
// 	pAd7124_dev->regs[AD7124_ADC_Control].value |= AD7124_ADC_CTRL_REG_MODE(0b1000) | AD7124_ADC_CTRL_REG_POWER_MODE(0x01);
// 	if ( (error_code = ad7124_write_register(pAd7124_dev, pAd7124_dev->regs[AD7124_ADC_Control]) ) < 0) {
// 		printf("Error (%d) setting AD7124 ADC into internal full scale calibration.\r\n", error_code);		
// 		return error_code;
// 	} else {
// 		printf("full scale calibration completed\n");
// 	}
// 	return error_code;
// }

// static int32_t read_error() {
// 	int32_t error_code;
// 	error_code = ad7124_read_register( pAd7124_dev, &pAd7124_dev->regs[AD7124_Error]);
// 		if(error_code < 0) {
// 			printf("error reading errorcode (%ld)", error_code);
// 			return error_code;
// 		} else {
// 			printf("error reg: %i\n", pAd7124_dev->regs[AD7124_Error].value);
// 		}
// 	return error_code;
// }



static int32_t set_slow_filters(bool enable) {
	//set high filter for calibration
	int32_t error_code = 0;
	enum ad7124_registers reg_nr;	
	
	for(reg_nr = AD7124_Filter_0; (reg_nr < AD7124_Offset_0) && !(error_code < 0); reg_nr++) {
		if(enable) {									
			struct ad7124_st_reg reg;
			reg=pAd7124_dev.regs[reg_nr];
			reg.value = AD7124_FILT_REG_FS(1024) | AD7124_FILT_REG_REJ60 | AD7124_FILT_REG_POST_FILTER(0b110); //cannot use higher than 1024, don't know why...
			error_code = ad7124_write_register(&pAd7124_dev, reg);			
			//printf("set slow filters: %d\n", reg.value);			
		} else {
			// struct ad7124_st_reg reg;
			// reg=pAd7124_dev->regs[reg_nr];
			// reg.value = 0;
			// error_code = ad7124_write_register(pAd7124_dev, reg); //zero all?
			error_code = ad7124_write_register(&pAd7124_dev, pAd7124_dev.regs[reg_nr]); //original value
			//printf("set original filters: %d\n", pAd7124_dev->regs[reg_nr].value);			
		}						
		if (error_code < 0) break;		
	}		
	return error_code;	
}

static int32_t switch_channel(bool enable, enum ad7124_registers channel) {
	
	if(enable) {
	 	pAd7124_dev.regs[channel].value |= AD7124_CH_MAP_REG_CH_ENABLE;
	} else {
		pAd7124_dev.regs[channel].value &= ~(AD7124_CH_MAP_REG_CH_ENABLE);
	}
	
	int32_t error_code = 0;
	error_code |= ad7124_write_register(&pAd7124_dev, pAd7124_dev.regs[channel]);										
	LOG_INF("%s channel %i\n", enable ? "enabled" : "disabled", channel);			
	
	return error_code;
}

static int32_t do_fullscale_calibration() {	
	int32_t error_code = 0;	
	int32_t enabled_channels = 0;

	for (enum ad7124_registers i = AD7124_Offset_0; i < AD7124_Gain_0; i++)
	{
		//write zero in offset register of each configuration 
		pAd7124_dev.regs[i].value = 0x800000;
		if ( (error_code = ad7124_write_register(&pAd7124_dev, pAd7124_dev.regs[i]) ) < 0) {
			LOG_ERR("Error (%d) writing offset for setup 0.\n", error_code);
			return error_code;			
		}
	}

	for (uint8_t i = 0; i < AD7124_CHANNEL_COUNT; i++) {		
		if (pAd7124_dev.regs[AD7124_Channel_0 + i].value & AD7124_CH_MAP_REG_CH_ENABLE) {
			enabled_channels += (1 << i);	
			error_code |= switch_channel(false, AD7124_Channel_0 + i);
		}	
	}	
	
	error_code |= set_slow_filters(true);

	//loop channels for calibration
	for (uint8_t i = 0; i < AD7124_CHANNEL_COUNT; i++) { 
		if(enabled_channels & (1<<i)) {
			//enable for calibration
			switch_channel(true, AD7124_Channel_0 +i);
						
			//full scale must be done before zero scale calibration
			// set_full_scale_calibration();
			// ad7124_wait_for_conv_ready(pAd7124_dev, 10000);
			
			set_zero_scale_calibration();											
			ad7124_wait_for_conv_ready(&pAd7124_dev, 10000);
						
			switch_channel(false, AD7124_Channel_0 +i);
		}
	}

	error_code |= set_slow_filters(false);

	for (uint8_t i = 0; i < AD7124_CHANNEL_COUNT; i++) {  
		if(enabled_channels & (1<<i)) {
			error_code |= switch_channel(true, AD7124_Channel_0 + i);
		}
	}
	
	error_code = set_idle_mode();
	if(error_code != 0) {		
		LOG_ERR("error fullscale calibraion");
	}
	return error_code;
}


#define ad7124ChipId 0x14

/*!
 * @brief      reads the ID register on the AD7124
 *
 * @details
 */
static int read_id(uint8_t *id)
{
	int err = 0;
	if (ad7124_read_register(&pAd7124_dev, &pAd7124_dev.regs[AD7124_ID]) < 0) {
	   LOG_ERR("\r\nError Encountered reading ID register\r\n");
	   return -1;
	} 
	*id = (uint8_t)pAd7124_dev.regs[AD7124_ID].value;
	LOG_INF("\r\nRead ID Register = 0x%d\n",*id);
	return err;	
}

int init_ad7124(void) {	
	LOG_INF("initiating ad7124..");
	int err = 0;
		
	bool spiReady = spi_is_ready_dt(&spiDevice);
	if(!spiReady) LOG_ERR("Error: SPI device is not ready: %d", spiReady);
	uint8_t chipId =0;
	err |= ad7124_app_initialize(AD7124_CONFIG_A);	
	if(err) {
		LOG_ERR("error initializing ad7124");
		return -1;
	}
	err |= read_id(&chipId);	
	if(err) {
		LOG_ERR("error reading id");
		return -1;
	}
	if(chipId != ad7124ChipId) {
		LOG_ERR("id incorrect: %#02x, but expected 0x14", chipId);
		return -1;
	} else {
		LOG_INF("chip id verified: %d", ad7124ChipId);
	}
	return 0;
}



static struct bt_gatt_cb gatt_callbacks = {
	
};

int ble_load(void)
{		
	k_work_queue_init(&my_work_q);

	k_work_queue_start(&my_work_q, my_stack_area,
					   K_THREAD_STACK_SIZEOF(my_stack_area), 95,
					   NULL);	

	LOG_INF("loading ble");
	int err;

	err = bt_enable(NULL);
	if (err) {
		LOG_INF("Bluetooth init failed (err %d)\n", err);
		return -EADV;
	}

	bt_ready();

	bt_gatt_cb_register(&gatt_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);

	//clean up bonds
	bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);
	return 0;
}


SYS_INIT(ble_load, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
SYS_INIT(init_ad7124, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);