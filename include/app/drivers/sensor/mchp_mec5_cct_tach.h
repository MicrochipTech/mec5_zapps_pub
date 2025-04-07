/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_MCHP_MEC5_CCT_TACH_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_MCHP_MEC5_CCT_TACH_H_

#include <zephyr/drivers/sensor.h>

enum sensor_channel_mchp_mec5_cct_cap {
	/** capture timer output used for RPM calculation */
	SENSOR_CHAN_RPM1 = SENSOR_CHAN_PRIV_START,
	SENSOR_CHAN_RPM2,
	SENSOR_CHAN_RPM3,
	SENSOR_CHAN_RPM4,
	SENSOR_CHAN_RPM5,
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_MCHP_MEC5_CCT_TACH_H_ */
