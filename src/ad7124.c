
/* **************************************************************************//**
*   @file    ad7124.c
*   @brief   AD7124 implementation file.
*   	     Devices: AD7124-4, AD7124-8
*
********************************************************************************
* Copyright 2015-2019(c) Analog Devices, Inc.
*/
#include "ad7124.h"
#include <stdbool.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(AD7124_CORE, LOG_LEVEL_INF);
/* Error codes */
#define INVALID_VAL -1 /* Invalid argument */
#define COMM_ERR    -2 /* Communication error on receive */
#define TIMEOUT     -3 /* A timeout has occured */

/*
 * Post reset delay required to ensure all internal config done
 * A time of 2ms should be enough based on the data sheet, but 4ms
 * chosen to provide enough margin, in case mdelay is not accurate.
 */
#define AD7124_POST_RESET_DELAY      4
#define buflen 8



/***************************************************************************//**
 * @brief Reads the value of the specified register without checking if the
 *        device is ready to accept user requests.
 *
 * @param dev   - The handler of the instance of the driver.
 * @param p_reg - Pointer to the register structure holding info about the
 *               register to be read. The read value is stored inside the
 *               register structure.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_no_check_read_register(struct ad7124_dev *dev,
				      struct ad7124_st_reg* p_reg)
{
	
	int32_t ret = 0;
	uint8_t bufsend[buflen] = {0};
	uint8_t bufrec[buflen] = {0};
	uint8_t i = 0;
	uint8_t check8 = 0, add_status_length = 0;
	uint8_t msg_buf[buflen] = {0, 0, 0, 0, 0, 0, 0, 0};

	if(!dev || !p_reg)
		return INVALID_VAL;

	/* Build the Command word */
	bufsend[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD |
		    AD7124_COMM_REG_RA(p_reg->addr);

	/*
	 * If this is an AD7124_DATA register read, and the DATA_STATUS bit is set
	 * in ADC_CONTROL, need to read 4, not 3 bytes for DATA with STATUS
	 */
	if ((p_reg->addr == AD7124_DATA_REG) &&
	    (dev->regs[AD7124_ADC_Control].value & AD7124_ADC_CTRL_REG_DATA_STATUS)) {
		add_status_length = 1;
	}

	/* Read data from the device */
	//spi_write_read_blocking(spi_default, bufsend, bufrec,
	//			 ((dev->use_crc != AD7124_DISABLE_CRC) ? p_reg->size + 1 : p_reg->size) + 1 + add_status_length);
	ret = dev->tranceiver(dev->handle, bufsend, bufrec, ((dev->use_crc != AD7124_DISABLE_CRC) ? p_reg->size + 1 : p_reg->size) + 1 + add_status_length);

	if(ret < 0) {
		LOG_ERR("error register addr %d", p_reg->addr);
		return ret;
	}
		

	/* Check the CRC */
	if(dev->use_crc == AD7124_USE_CRC) {
		msg_buf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD |
			     AD7124_COMM_REG_RA(p_reg->addr);
		for(i = 1; i < p_reg->size + 2 + add_status_length; ++i) {
			msg_buf[i] = bufrec[i]; //still allright?!
		}
		check8 = ad7124_compute_crc8(msg_buf, p_reg->size + 2 + add_status_length);
	}

	if(check8 != 0) {
		/* ReadRegister checksum failed. */
		return COMM_ERR;
	}

	/*
	 * if reading Data with 4 bytes, need to copy the status byte to the STATUS
	 * register struct value member
	 */
	if (add_status_length) {
		dev->regs[AD7124_Status].value = bufrec[p_reg->size + 1];
	}

	/* Build the result */
	p_reg->value = 0;
	for(i = 1; i < p_reg->size + 1; i++) {
		p_reg->value <<= 8;
		p_reg->value += bufrec[i];
	}

	return ret;
}

/***************************************************************************//**
 * @brief Writes the value of the specified register without checking if the
 *        device is ready to accept user requests.
 *
 * @param dev - The handler of the instance of the driver.
 * @param reg - Register structure holding info about the register to be written
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_no_check_write_register(struct ad7124_dev *dev,
				       struct ad7124_st_reg reg)
{
	int32_t ret = 0;
	int32_t reg_value = 0;
	uint8_t wr_buf[buflen] = {0}, rd_buf[buflen] = {0};

	uint8_t i = 0;
	uint8_t crc8 = 0;

	if(!dev) {
		LOG_ERR("device not found!");
		return INVALID_VAL;		
	}
		

	/* Build the Command word */
	wr_buf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_WR |
		    AD7124_COMM_REG_RA(reg.addr);

	/* Fill the write buffer */
	reg_value = reg.value;
	for(i = 0; i < reg.size; i++) {
		wr_buf[reg.size - i] = reg_value & 0xFF;
		reg_value >>= 8;
	}

	/* Compute the CRC */
	if(dev->use_crc != AD7124_DISABLE_CRC) {
		crc8 = ad7124_compute_crc8(wr_buf, reg.size + 1);
		wr_buf[reg.size + 1] = crc8;
	}

	/* Write data to the device */
	// ret = spi_write_read_blocking(spi_default,
	// 			 wr_buf, rd_buf,
	// 			 (dev->use_crc != AD7124_DISABLE_CRC) ? reg.size + 2
	// 			 : reg.size + 1);

	ret = dev->tranceiver(dev->handle, rd_buf, wr_buf, (dev->use_crc != AD7124_DISABLE_CRC) ? reg.size + 2 : reg.size + 1);

	return ret;
}

/***************************************************************************//**
 * @brief Reads the value of the specified register only when the device is ready
 *        to accept user requests. If the device ready flag is deactivated the
 *        read operation will be executed without checking the device state.
 *
 * @param dev   - The handler of the instance of the driver.
 * @param p_reg - Pointer to the register structure holding info about the
 *               register to be read. The read value is stored inside the
 *               register structure.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_read_register(struct ad7124_dev *dev,
			     struct ad7124_st_reg* p_reg)
{
	int32_t ret;

	if (p_reg->addr != AD7124_ERR_REG && dev->check_ready) {
		ret = ad7124_wait_for_spi_ready(dev,
						dev->spi_rdy_poll_cnt);
		if (ret < 0) {
			LOG_ERR("err register %d", p_reg->addr);
			return ret;
		}
			
	}
	ret = ad7124_no_check_read_register(dev,
					    p_reg);

	return ret;
}

/***************************************************************************//**
 * @brief Writes the value of the specified register only when the device is
 *        ready to accept user requests. If the device ready flag is deactivated
 *        the write operation will be executed without checking the device state.
 *
 * @param dev - The handler of the instance of the driver.
 * @param p_reg - Register structure holding info about the register to be written
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_write_register(struct ad7124_dev *dev,
			      struct ad7124_st_reg p_reg)
{
	int32_t ret;

	if (dev->check_ready) {
		ret = ad7124_wait_for_spi_ready(dev, dev->spi_rdy_poll_cnt);
		if (ret < 0) {
			LOG_ERR("timeout spi ready");
			return ret;			
		}
	}
	ret = ad7124_no_check_write_register(dev, p_reg);

	return ret;
}

/***************************************************************************//**
 * @brief Resets the device.
 *
 * @param dev - The handler of the instance of the driver.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_reset(struct ad7124_dev *dev)
{
	int32_t ret = 0;
	uint8_t wr_buf[buflen] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, rd_buf[buflen] = {0};

	if(!dev)
		return INVALID_VAL;

	//ret = spi_write_read_blocking(spi_default, wr_buf, rd_buf, 8);
	ret = dev->tranceiver(dev->handle, wr_buf, rd_buf, 8);

	/* CRC is disabled after reset */
	dev->use_crc = AD7124_DISABLE_CRC;

	/* Read POR bit to clear */
	ret = ad7124_wait_to_power_on(dev, dev->spi_rdy_poll_cnt);

	dev->ptDelay(200);

	

	return ret;
}

/***************************************************************************//**
 * @brief Waits until the device can accept read and write user actions.
 *
 * @param dev     - The handler of the instance of the driver.
 * @param timeout - Count representing the number of polls to be done until the
 *                  function returns.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_wait_for_spi_ready(struct ad7124_dev *dev,
				  uint32_t timeout)
{
	struct ad7124_st_reg *regs;
	int32_t ret;
	int8_t ready = 0;

	if(!dev)
		return INVALID_VAL;

	regs = dev->regs;

	while(!ready && --timeout) {
		/* Read the value of the Error Register */
		ret = ad7124_read_register(dev, &regs[AD7124_Error]);
		if(ret < 0) {
			LOG_ERR("error reading error register");
			return ret;
		}
			
		
		/* Check the SPI IGNORE Error bit in the Error Register */
		ready = (regs[AD7124_Error].value &
			 AD7124_ERR_REG_SPI_IGNORE_ERR) == 0;
		dev->ptDelay(100);
	}
	if(!timeout) LOG_ERR("timeout occured");

	return timeout ? 0 : TIMEOUT;
}

/***************************************************************************//**
 * @brief Waits until the device finishes the power-on reset operation.
 *
 * @param dev     - The handler of the instance of the driver.
 * @param timeout - Count representing the number of polls to be done until the
 *                  function returns.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_wait_to_power_on(struct ad7124_dev *dev,
				uint32_t timeout)
{
	struct ad7124_st_reg *regs;
	int32_t ret;
	int8_t powered_on = 0;

	if(!dev)
		return INVALID_VAL;

	regs = dev->regs;

	while(!powered_on && timeout--) {
		ret = ad7124_read_register(dev,
					   &regs[AD7124_Status]);
		if(ret < 0)
			return ret;

		/* Check the POR_FLAG bit in the Status Register */
		powered_on = (regs[AD7124_Status].value &
			      AD7124_STATUS_REG_POR_FLAG) == 0;
	}

	return (timeout || powered_on) ? 0 : TIMEOUT;
}

/***************************************************************************//**
 * @brief Waits until a new conversion result is available.
 *
 * @param dev     - The handler of the instance of the driver.
 * @param timeout - Count representing the number of polls to be done until the
 *                  function returns if no new data is available.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_wait_for_conv_ready(struct ad7124_dev *dev,
				   uint32_t timeout)
{
	struct ad7124_st_reg *regs;
	int32_t ret;
	int8_t ready = 0;

	if(!dev)
		return INVALID_VAL;

	regs = dev->regs;

	while(!ready && --timeout) {
		/* Read the value of the Status Register */
		ret = ad7124_read_register(dev, &regs[AD7124_Status]);
		if(ret < 0)
			return ret;

		/* Check the RDY bit in the Status Register */
		ready = (regs[AD7124_Status].value &
			 AD7124_STATUS_REG_RDY) == 0;
		dev->ptDelay(200);
	}

	return timeout ? 0 : TIMEOUT;
}

/***************************************************************************//**
 * @brief Reads the conversion result from the device.
 *
 * @param dev     - The handler of the instance of the driver.
 * @param p_data  - Pointer to store the read data.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_read_data(struct ad7124_dev *dev,
			 int32_t* p_data)
{
	struct ad7124_st_reg *regs;
	int32_t ret;

	if(!dev)
		return INVALID_VAL;

	regs = dev->regs;

	/* Read the value of the Status Register */
	ret = ad7124_read_register(dev, &regs[AD7124_Data]);

	/* Get the read result */
	*p_data = regs[AD7124_Data].value;

	return ret;
}

/***************************************************************************//**
 * @brief Computes the CRC checksum for a data buffer.
 *
 * @param p_buf    - Data buffer
 * @param buf_size - Data buffer size in bytes
 *
 * @return Returns the computed CRC checksum.
*******************************************************************************/
uint8_t ad7124_compute_crc8(uint8_t * p_buf, uint8_t buf_size)
{
	uint8_t i = 0;
	uint8_t crc = 0;

	while(buf_size) {
		for(i = 0x80; i != 0; i >>= 1) {
			bool cmp1 = (crc & 0x80) != 0;
			bool cmp2 = (*p_buf & i) != 0;
			if(cmp1 != cmp2) { /* MSB of CRC register XOR input Bit from Data */
				crc <<= 1;
				crc ^= AD7124_CRC8_POLYNOMIAL_REPRESENTATION;
			} else {
				crc <<= 1;
			}
		}
		p_buf++;
		buf_size--;
	}
	return crc;
}

/***************************************************************************//**
 * @brief Updates the CRC settings.
 *
 * @param dev - The handler of the instance of the driver.
 *
 * @return None.
*******************************************************************************/
void ad7124_update_crcsetting(struct ad7124_dev *dev)
{
	struct ad7124_st_reg *regs;

	if(!dev)
		return;

	regs = dev->regs;

	/* Get CRC State. */
	if (regs[AD7124_Error_En].value & AD7124_ERREN_REG_SPI_CRC_ERR_EN) {
		dev->use_crc = AD7124_USE_CRC;
	} else {
		dev->use_crc = AD7124_DISABLE_CRC;
	}
}

/***************************************************************************//**
 * @brief Updates the device SPI interface settings.
 *
 * @param dev - The handler of the instance of the driver.
 *
 * @return None.
*******************************************************************************/
void ad7124_update_dev_spi_settings(struct ad7124_dev *dev)
{
	struct ad7124_st_reg *regs;

	if(!dev)
		return;

	regs = dev->regs;

	if (regs[AD7124_Error_En].value & AD7124_ERREN_REG_SPI_IGNORE_ERR_EN) {
		dev->check_ready = 1;
	} else {
		dev->check_ready = 0;
	}
}

/***************************************************************************//**
 * @brief Initializes the AD7124.
 *
 * @param device     - The device structure.
 * @param init_param - The structure that contains the device initial
 * 		       parameters.
 *
 * @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t ad7124_setup(struct ad7124_dev *device)
{
	int32_t ret; 
	enum ad7124_registers reg_nr;			

	/*  Reset the device interface.*/
	ret = ad7124_reset(device);
	if (ret < 0)
		return ret;

	/* Update the device structure with power-on/reset settings */
	device->check_ready = 1;

	/* Initialize registers AD7124_ADC_Control through AD7124_Filter_7. */
	for(reg_nr = AD7124_Status; (reg_nr < AD7124_Offset_0) && !(ret < 0);
	    reg_nr++) {
		if (device->regs[reg_nr].rw == AD7124_RW) {
			ret = ad7124_write_register(device, device->regs[reg_nr]);
			if (ret < 0) {
				LOG_ERR("error setup writing %d", reg_nr);
				return -1;
			}
		}

		/* Get CRC State and device SPI interface settings */
		if (reg_nr == AD7124_Error_En) {
			ad7124_update_crcsetting(device);
			ad7124_update_dev_spi_settings(device);
		}
	}	

	return ret;
}
