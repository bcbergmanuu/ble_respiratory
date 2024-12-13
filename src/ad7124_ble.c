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

#include "ad7124.h"
#include "ad7124_regs.h"
#include "ad7124_support.h"

#include "ad7124_ble.h"
#include "config_respiratory.h"

#define SPI_OP  SPI_OP_MODE_MASTER |SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE


LOG_MODULE_REGISTER(AD7124_APP, LOG_LEVEL_INF);

#define AD7124_CHANNEL_COUNT 16

#define SHOW_ALL_CHANNELS     false
#define SHOW_ENABLED_CHANNELS  true

#define DISPLAY_DATA_TABULAR    0
#define DISPLAY_DATA_STREAM     1

#define SPIOP      SPI_WORD_SET(8) | SPI_TRANSFER_MSB

const struct spi_dt_spec spiDevice = SPI_DT_SPEC_GET(DT_NODELABEL(gendev), SPIOP, 0);

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

/// @brief generate delay
/// @param ms delay time ms
static void platform_delay(uint32_t ms) {
	LOG_INF("dela for %d", ms);
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


/*!
 * @brief      Continuously acquires samples in Continuous Conversion mode
 *
 * @details   The ADC is run in continuous mode, and all samples are acquired
 *            and assigned to the channel they come from. Escape key an be used
 *            to exit the loop
 */
static int32_t do_continuous_conversion(bool doVoltageConvertion)
{
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
    while (1) {
		

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
			
			if(doVoltageConvertion) {
				LOG_INF("%.8f", (double) ad7124_convert_sample_to_voltage(&pAd7124_dev, channel_read, sample_data) );				
			} else { LOG_INF("%i", sample_data); }
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
	} 
	*id = (uint8_t)pAd7124_dev.regs[AD7124_ID].value;
	LOG_INF("\r\nRead ID Register = 0x%d\n",*id);
	return err;	
}

extern void start_ad() {
	LOG_INF("Version 2");
	int err = 0;
		

	bool spiReady = spi_is_ready_dt(&spiDevice);
	if(!spiReady) LOG_ERR("Error: SPI device is not ready: %d", spiReady);
	uint8_t chipId =0;
	err |= ad7124_app_initialize(AD7124_CONFIG_A);	
	err |= read_id(&chipId);
	if(chipId != 0x14) {
		LOG_ERR("id incorrect %#02x", chipId);
	}
	err |= do_fullscale_calibration();
	err |= do_continuous_conversion(true);
	if(err) {
		LOG_ERR("error start ad %d", err);
	}
}