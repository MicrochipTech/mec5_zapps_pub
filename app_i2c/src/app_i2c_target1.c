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
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/mchp_mec5_i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(main);

#include <app/drivers/mchp_mec5_i2c_nl.h>

#include <device_mec5.h>
#include <mec_i2c_api.h>
#include <mec_retval.h>

#include "app_config.h"
#include "app_i2c_target1.h"

#define I2C_TARGET1_THREAD_STACK_SIZE 1024
#define I2C_TARGET1_THREAD_PRIORITY 5

#define I2C_TARGET1_CAP_STATE_ENTRIES 16

#define I2C_TARGET1_RX_BUF_SIZE 32
#define I2C_TARGET1_TX_BUF_SIZE 32

enum i2c_kevents {
	I2C_TARGET_EVENT_BUF_WR_REC_POS = 0,
	I2C_TARGET_EVENT_BUF_RD_REQ_POS,
	I2C_TARGET_EVENT_STOP_POS,
};

#define I2C_KEVENTS_ALL (BIT(I2C_TARGET_EVENT_BUF_WR_REC_POS) |\
			 BIT(I2C_TARGET_EVENT_BUF_RD_REQ_POS) | BIT(I2C_TARGET_EVENT_STOP_POS))

static void i2c_target1_thread(void *p1, void *p2, void *p3);
static int i2c_target1_stop_cb(struct i2c_target_config *config);
static int i2c_target1_buf_rd_req(struct i2c_target_config *config, uint8_t **ptr, uint32_t *len);
static void i2c_target1_buf_wr_rec(struct i2c_target_config *config, uint8_t *ptr, uint32_t len);

const static struct i2c_target_callbacks target1_cbs = {
	.write_requested = NULL,
	.read_requested = NULL,
	.write_received = NULL,
	.read_processed = NULL,
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	.buf_write_received = i2c_target1_buf_wr_rec,
	.buf_read_requested = i2c_target1_buf_rd_req,
#endif
	.stop = i2c_target1_stop_cb,
};

struct i2c_thread_init_info {
	const struct device *i2c_host_dev;
	const struct device *i2c_target_dev;
	uint32_t i2c_config;
	uint8_t port;
	uint8_t target_addr;
};

static struct i2c_thread_init_info t1_init_info;
static struct i2c_target_config target1_cfg;

static uint32_t rxbuf1_len;
static uint32_t txbuf1_len;

/* Buffer for data transfer to Target 1 from external I2C Host (I2C Write) */
static uint8_t rxbuf1[I2C_TARGET1_RX_BUF_SIZE];
/* Buffer for data transfer from Target 1 to external I2C Host (I2C Read) */
static uint8_t txbuf1[I2C_TARGET1_TX_BUF_SIZE];

#define I2C_TARGET_CB_ID_BUF_WR_REC 1
#define I2C_TARGET_CB_ID_BUF_RD_REQ 2
#define I2C_TARGET_CB_ID_STOP 3

struct i2c_target_cb_capture {
	uint8_t id;
	uint16_t i2c_addr;
	uint32_t len;
	uintptr_t buf_addr;
};

static volatile uint32_t cb_cap1_idx;
static volatile struct i2c_target_cb_capture cap1[I2C_TARGET1_CAP_STATE_ENTRIES];

K_THREAD_STACK_DEFINE(i2c_thread1_stack_area, I2C_TARGET1_THREAD_STACK_SIZE);
static struct k_thread i2c_thread1_data;
static k_tid_t i2c_thread1_tid;
static struct k_event t1_events;

static void reset_cap(void)
{
	cb_cap1_idx = 0;
	memset((void *)&cap1, 0, sizeof(cap1));
}

static void add_cap(uint8_t id, uint16_t i2c_addr, uint32_t len, uintptr_t baddr)
{
	uint32_t idx = cb_cap1_idx;

	if (idx < I2C_TARGET1_CAP_STATE_ENTRIES) {
		cap1[idx].id = id;
		cap1[idx].i2c_addr = i2c_addr;
		cap1[idx].len = len;
		cap1[idx].buf_addr = baddr;
		cb_cap1_idx++;
	}
}

static void pr_capture(void)
{
	uint32_t idx = cb_cap1_idx;

	LOG_INF("PR capture: index = %u", idx);

	idx = MIN(idx, I2C_TARGET1_CAP_STATE_ENTRIES);

	for (uint8_t i = 0; i < idx; i++) {
		volatile struct i2c_target_cb_capture *c = &cap1[i];

		LOG_INF("cap1[%u]: id=%u i2c_addr=0x%0x len=%u buf_addr=0x%0lx",
			i, c->id, c->i2c_addr, c->len, c->buf_addr);
	}
}


/* Driver invokes this callback if it detects a STOP from the external I2C Host */
static volatile uint32_t t1_stop_cnt;

static int i2c_target1_stop_cb(struct i2c_target_config *config)
{
	uint16_t i2c_addr = 0xffffu;

	t1_stop_cnt++;

	if (config) {
		i2c_addr = config->address;
	}

	add_cap(I2C_TARGET_CB_ID_STOP, i2c_addr, 0, 0);

	k_event_post(&t1_events, BIT(I2C_TARGET_EVENT_STOP_POS));

	return 0;
}

/* Application supplies buffer to driver containing data for external Host to read from
 * this target.
 * Return 0 indicates data is good and let Host read.
 * Return < 0  indicates driver should ignore Host until next START.
 */
static volatile uint32_t t1_buf_rd_req_cnt;

static int i2c_target1_buf_rd_req(struct i2c_target_config *config, uint8_t **ptr, uint32_t *len)
{
	int ret = 0;
	uint16_t i2c_addr = 0xffffu;

	t1_buf_rd_req_cnt++;

	if (config) {
		i2c_addr = config->address;
	}

	if (i2c_addr == target1_cfg.address) {
		*ptr = txbuf1;
		*len = I2C_TARGET1_TX_BUF_SIZE;
	} else {
		ret = -ENXIO;
	}

	add_cap(I2C_TARGET_CB_ID_BUF_RD_REQ, i2c_addr, *len, (uintptr_t)*ptr);

	k_event_post(&t1_events, BIT(I2C_TARGET_EVENT_BUF_RD_REQ_POS));

	return ret;
}

/* Driver invokes this callback upon reception of its target address and data from the external
 * I2C Host. Driver supplies buffer of data and length in bytes.
 *
 */
static volatile uint32_t t1_buf_wr_rec_cnt;

static void i2c_target1_buf_wr_rec(struct i2c_target_config *config, uint8_t *ptr, uint32_t len)
{
	uint32_t data_len = len;
	uint16_t i2c_addr = 0xffffu;

	t1_buf_wr_rec_cnt++;
	rxbuf1_len = 0;

	if (config) {
		i2c_addr = config->address;
	}

	if (i2c_addr == target1_cfg.address) {
		if (data_len > I2C_TARGET1_RX_BUF_SIZE) {
			data_len = I2C_TARGET1_RX_BUF_SIZE;
		}
		if (ptr && data_len) {
			memcpy(rxbuf1, ptr, data_len);
			rxbuf1_len = data_len;
		}
	}

	add_cap(I2C_TARGET_CB_ID_BUF_WR_REC, i2c_addr, len, (uintptr_t)ptr);

	k_event_post(&t1_events, BIT(I2C_TARGET_EVENT_BUF_WR_REC_POS));
}

/* Configure two I2C controllers one has Host the other as target.
 * @param i2c_host_dev device structure pointer for Host I2C controller
 * @param i2c_target_dev device structure pointer for Target I2C controller
 * @param i2c_config configuration to use for both I2C controllers
 * @param I2C port to connect both controller together. Caller must have configured
 *        the SDA/SCL pins on this port.
 * @param target_addr1 First target I2C address (0 if not used).
 * @param target_addr2 Second target I2C address (0 if not used).
 * @return 0(success) else negative error code.
 *
 * Configure both for the same I2C frequency and port.
 * register target I2C address with target controller.
 */
int i2c_target1_config(const struct device *i2c_host_dev, const struct device *i2c_target_dev,
		       uint32_t i2c_config, uint8_t port, uint8_t target_addr)
{
	int ret = 0;

	if (!i2c_host_dev || !i2c_target_dev) {
		return -EINVAL;
	}

	rxbuf1_len = 0;
	txbuf1_len = 0;

	t1_stop_cnt = 0;
	t1_buf_wr_rec_cnt = 0;
	t1_buf_rd_req_cnt = 0;

	reset_cap();
	memset(&t1_init_info, 0, sizeof(t1_init_info));
	memset(txbuf1, 0, sizeof(txbuf1));
	memset(rxbuf1, 0, sizeof(rxbuf1));

	ret = i2c_mchp_configure(i2c_host_dev, i2c_config, port);
	if (ret) {
		LOG_ERR("I2C Host controller switch to port %u failed (%d)", port, ret);
		return ret;
	}

	ret = i2c_mchp_nl_configure(i2c_target_dev, i2c_config, port);
	if (ret) {
		LOG_ERR("I2C Target controller switch to port %u failed (%d)", port, ret);
		return ret;
	}

	if (target_addr) {
		memset(&target1_cfg, 0, sizeof(target1_cfg));
		target1_cfg.address = (uint16_t)target_addr;
		target1_cfg.callbacks = &target1_cbs;

		ret = i2c_target_register(i2c_target_dev, &target1_cfg);
		if (ret) {
			LOG_ERR("Register I2C target address 0x%02x failed (%d)",
				target_addr, ret);
			return ret;
		}
	}

	t1_init_info.i2c_host_dev = i2c_host_dev;
	t1_init_info.i2c_target_dev = i2c_target_dev;
	t1_init_info.i2c_config = i2c_config;
	t1_init_info.port = port;
	t1_init_info.target_addr = target_addr;

	LOG_INF("Create I2C target 1 thread");

	i2c_thread1_tid = k_thread_create(&i2c_thread1_data, i2c_thread1_stack_area,
					  K_THREAD_STACK_SIZEOF(i2c_thread1_stack_area),
					  i2c_target1_thread,
					  NULL, NULL, NULL,
					  I2C_TARGET1_THREAD_PRIORITY,
					  0, K_NO_WAIT);

	return 0;
}

#if 0
static int reconfig_i2c(void)
{
	int ret1 = i2c_mchp_nl_configure(t1_init_info.i2c_host_dev,
					 t1_init_info.i2c_config, t1_init_info.port);
	int ret2 = i2c_mchp_nl_configure(t1_init_info.i2c_target_dev,
					 t1_init_info.i2c_config, t1_init_info.port);

	if (ret1) {
		return ret1;
	}

	if (ret2) {
		return ret2;
	}

	return 0;
}
#endif

static void i2c_target1_thread(void *p1, void *p2, void *p3)
{
	uint32_t events = 0;

	LOG_INF("I2C Target 1 thread entry");

	k_event_init(&t1_events);

	while (1) {

		LOG_INF("I2C Target 1 thread wait for I2C events");
		events = k_event_wait(&t1_events, I2C_KEVENTS_ALL, true, K_MSEC(5000));
		if (events == 0) {
			LOG_INF("I2C Target 1 thread wake no events (timeout)");
			continue;
		}

		LOG_INF("I2C Target 1 thread wake: events = 0x%0x", events);

		if (events & BIT(I2C_TARGET_EVENT_BUF_WR_REC_POS)) {
			LOG_INF("I2C Target 1: Buffer write received event: evcnt=%u len = %u",
				t1_buf_wr_rec_cnt, rxbuf1_len);
			if (rxbuf1_len) {
				for (uint32_t n = 0; n < rxbuf1_len; n++) {
					LOG_INF("rxbuf1[%u] = 0x%02x", n, rxbuf1[n]);
				}
				LOG_HEXDUMP_INF(rxbuf1, rxbuf1_len, "Received data:");
			}
		}

		if (events & BIT(I2C_TARGET_EVENT_BUF_RD_REQ_POS)) {
			LOG_INF("I2C Target 1: Buffer read request event");
		}

		if (events & BIT(I2C_TARGET_EVENT_STOP_POS)) {
			LOG_INF("I2C Target 1 STOP event");
		}

		pr_capture();

	} /* while(1) */
}
