/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _APP_I2C_TARGET1_H_
#define _APP_I2C_TARGET1_H_

int i2c_target1_config(const struct device *i2c_host_dev, const struct device *i2c_target_dev,
			uint32_t i2c_config, uint8_t port, uint8_t target_addr);

#endif /* _APP_I2C_TARGET1_H_ */