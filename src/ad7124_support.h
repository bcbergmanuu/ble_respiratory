/*!
 *****************************************************************************
  @file:  ad7124_support.h

  @brief: Provides useful support functions for the AD7124 NoOS driver

  @details:
 -----------------------------------------------------------------------------
Copyright (c) 2019 Analog Devices, Inc.  All rights reserved.

*/

#ifndef AD7124_SUPPORT_H_
#define AD7124_SUPPORT_H_

#include "ad7124.h"

/* PGA Gain Value */
#define AD7124_PGA_GAIN(x) (1 << (x))

#define AD7124_REF_VOLTAGE 2.5
#define AD7124_ADC_N_BITS 24

uint8_t ad7124_get_channel_setup(struct ad7124_dev *dev, uint8_t channel);
uint8_t ad7124_get_channel_pga(struct ad7124_dev *dev, uint8_t channel);
bool ad7124_get_channel_bipolar(struct ad7124_dev *dev, uint8_t channel);
double ad7124_convert_sample_to_voltage(struct ad7124_dev *dev, uint8_t channel,
                                       uint32_t sample);

#endif /* AD7124_SUPPORT_H_ */
