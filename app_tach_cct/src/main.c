/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <soc.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/zbus/zbus.h>

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#include <device_mec5.h>
#include <mec_retval.h>

#include "app_config.h"

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define CCT0_NODE DT_NODELABEL(cctmr0)

static const struct device *tach_cct_dev = DEVICE_DT_GET(CCT0_NODE);

/* ---- Global variables ---- */
static volatile uint32_t spin_val;
static volatile int ret_val;

/* ---- prototypes ---- */
static void spin_on(uint32_t id, int rval);

int main(void)
{
	int ret = 0;

	LOG_INF("app_tach_icct sample: board: %s", DT_N_P_compatible_IDX_0);

	if (!device_is_ready(tach_cct_dev)) {
		LOG_ERR("TACH CCT device is not ready!");
		return -EIO;
	}

	LOG_INF("Application Done (%d)", ret);
	while (1) {
		k_sleep(K_MSEC(5000));
		LOG_INF("Wake: main");
	}

	spin_on((uint32_t)__LINE__, 0);

	return 0;
}

static void spin_on(uint32_t id, int rval)
{
	spin_val = id;
	ret_val = rval;

	log_panic(); /* flush log buffers */

	while (spin_val) {
		;
	}
}
