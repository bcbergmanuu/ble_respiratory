#ifndef __respiratory_
#define __respiratory_

/*!
 *****************************************************************************
  @file:  ad7124_regs_config_a.c

  @brief: Configuration file A for AD7124

  @details:
 -----------------------------------------------------------------------------
Copyright (c) 2019 Analog Devices, Inc.
*/

#include "ad7124_regs.h"

#define boardname "respiratory"
#define filterFS 720

const static struct ad7124_st_reg ad7124_regs_config_b[AD7124_REG_NO] = {
    {0x00, 0x00,   1, 2}, /* AD7124_Status */
	{0x01, AD7124_ADC_CTRL_REG_MODE(2) | AD7124_ADC_CTRL_REG_POWER_MODE(3) | AD7124_ADC_CTRL_REG_CLK_SEL(0) | AD7124_ADC_CTRL_REG_REF_EN, 2, 1}, /* AD7124_ADC_Control */
	{0x02, 0x0000, 3, 2}, /* AD7124_Data */
	{0x03, 0x0000, 3, 1}, /* AD7124_IOCon1 */
	{0x04, 0x0000, 2, 1}, /* AD7124_IOCon2 */
	{0x05, 0x02,   1, 2}, /* AD7124_ID */
	{0x06, 0x0000, 3, 2}, /* AD7124_Error */
	{0x07, AD7124_ERREN_REG_ADC_SAT_ERR_EN | AD7124_ERREN_REG_ADC_CONV_ERR_EN | AD7124_ERREN_REG_ADC_CAL_ERR_EN | 
	AD7124_ERREN_REG_SPI_IGNORE_ERR_EN | AD7124_ERREN_REG_AINM_OV_ERR_EN | AD7124_ERREN_REG_AINM_UV_ERR_EN | AD7124_ERREN_REG_REF_DET_ERR_EN, 3, 1}, /* AD7124_Error_En (enable everything here asap) */
	{0x08, 0x0001,   1, 2}, /* AD7124_Mclk_Count */
	{0x09, AD7124_CH_MAP_REG_CH_ENABLE | AD7124_CH_MAP_REG_SETUP(0) | AD7124_CH_MAP_REG_AINP(8) | AD7124_CH_MAP_REG_AINM(9), 2, 1}, /* AD7124_Channel_0 */
	{0x0A, 0x0001, 2, 1}, /* AD7124_Channel_1 */
	{0x0B, 0x0001, 2, 1}, /* AD7124_Channel_2 */
	{0x0C, 0x0001, 2, 1}, /* AD7124_Channel_3 */
	{0x0D, 0x0001, 2, 1}, /* AD7124_Channel_4 */
	{0x0E, 0x0001, 2, 1}, /* AD7124_Channel_5 */
	{0x0F, 0x0001, 2, 1}, /* AD7124_Channel_6 */
	{0x10, 0x0001, 2, 1}, /* AD7124_Channel_7 */
	{0x11, 0x0001, 2, 1}, /* AD7124_Channel_8 */
	{0x12, 0x0001, 2, 1}, /* AD7124_Channel_9 */
	{0x13, 0x0001, 2, 1}, /* AD7124_Channel_10 */
	{0x14, 0x0001, 2, 1}, /* AD7124_Channel_11 */
	{0x15, 0x0001, 2, 1}, /* AD7124_Channel_12 */
	{0x16, 0x0001, 2, 1}, /* AD7124_Channel_13 */
	{0x17, 0x0001, 2, 1}, /* AD7124_Channel_14 */
	{0x18, 0x0001, 2, 1}, /* AD7124_Channel_15 */
	{0x19, AD7124_CFG_REG_REF_SEL(0b10) | AD7124_CFG_REG_BIPOLAR | AD7124_CFG_REG_PGA(7), 2, 1}, /* AD7124_Config_0 */
	{0x1A, 0x0001, 2, 1}, /* AD7124_Config_1 */
	{0x1B, 0x0001, 2, 1}, /* AD7124_Config_2 */
	{0x1C, 0x0001, 2, 1}, /* AD7124_Config_3 */
	{0x1D, 0x0001, 2,1 }, /* AD7124_Config_4 */
	{0x1E, 0x0001, 2,1 },/* AD7124_Config_5 */
	{0x1F, 0x0001, 2,1 },/* AD7124_Config_6 */
	{0x20, 0x0001, 2,1 },/* AD7124_Config_7 */
	{0x21, AD7124_FILT_REG_FS(filterFS), 3, 1}, /* AD7124_Filter_0 */
	{0x22, AD7124_FILT_REG_FS(filterFS), 3, 1}, /* AD7124_Filter_1 */
	{0x23, AD7124_FILT_REG_FS(filterFS), 3, 1},/* AD7124_Filter_2 */
	{0x24, AD7124_FILT_REG_FS(filterFS), 3, 1},/* AD7124_Filter_3 */
	{0x25, AD7124_FILT_REG_FS(filterFS), 3, 1},/* AD7124_Filter_4 */
	{0x26, AD7124_FILT_REG_FS(filterFS), 3, 1},/* AD7124_Filter_5 */
	{0x27, AD7124_FILT_REG_FS(filterFS), 3, 1},/* AD7124_Filter_6 */
	{0x28, AD7124_FILT_REG_FS(filterFS), 3, 1},/* AD7124_Filter_7 */
	{0x29, 0x800000, 3, 1}, /* AD7124_Offset_0 */
	{0x2A, 0x800000, 3, 1}, /* AD7124_Offset_1 */
	{0x2B, 0x800000, 3, 1}, /* AD7124_Offset_2 */
	{0x2C, 0x800000, 3, 1}, /* AD7124_Offset_3 */
	{0x2D, 0x800000, 3, 1}, /* AD7124_Offset_4 */
	{0x2E, 0x800000, 3, 1}, /* AD7124_Offset_5 */
	{0x2F, 0x800000, 3, 1}, /* AD7124_Offset_6 */
	{0x30, 0x800000, 3, 1}, /* AD7124_Offset_7 */
	{0x31, 0x500000, 3, 1}, /* AD7124_Gain_0 */
	{0x32, 0x500000, 3, 1}, /* AD7124_Gain_1 */
	{0x33, 0x500000, 3, 1}, /* AD7124_Gain_2 */
	{0x34, 0x500000, 3, 1}, /* AD7124_Gain_3 */
	{0x35, 0x500000, 3, 1}, /* AD7124_Gain_4 */
	{0x36, 0x500000, 3, 1}, /* AD7124_Gain_5 */
	{0x37, 0x500000, 3, 1}, /* AD7124_Gain_6 */
	{0x38, 0x500000, 3, 1}, /* AD7124_Gain_7 */
};


#endif