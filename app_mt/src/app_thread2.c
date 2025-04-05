/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <soc.h>
#include <device_mec5.h>
#include <mec_retval.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/zbus/zbus.h>

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(main);

#include <app_version.h>

BUILD_ASSERT(CONFIG_APP_THREAD2_STACK_SIZE >= 256, "Thread 2 stack size < 256 bytes");

static struct k_thread app_thread2_data;
static k_tid_t app_thread2_tid;

K_THREAD_STACK_DEFINE(app_thread2_stack_area, CONFIG_APP_THREAD2_STACK_SIZE);

static void app_thread2(void *p1, void *p2, void *p3);

int app_thread2_create(uint32_t start_delay_ms)
{
	k_timeout_t start_delay_ticks = K_NO_WAIT;

	if (start_delay_ms) {
		if (start_delay_ms == UINT32_MAX) {
			start_delay_ticks = K_FOREVER;
		} else {
			start_delay_ticks = K_MSEC(start_delay_ms);
		}
	}

	app_thread2_tid = k_thread_create(&app_thread2_data, app_thread2_stack_area,
					  K_THREAD_STACK_SIZEOF(app_thread2_stack_area),
					  app_thread2,
					  NULL, NULL, NULL,
					  CONFIG_APP_THREAD2_PRIORITY,
					  0, start_delay_ticks);

	if (app_thread2_tid == NULL) {
		return -ESRCH;
	}

	return 0;
}

k_tid_t app_thread2_get_tid(void)
{
	return app_thread2_tid;
}

static void app_thread2(void *p1, void *p2, void *p3)
{
	LOG_INF("T2 entry: params %p %p %p", p1, p2, p3);

	while (1) {
		LOG_INF("T2 Sleep");
		k_sleep(K_MSEC(2000));
		LOG_INF("T2 wake");
	}
}
