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
#include <zephyr/zbus/zbus.h>

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#include <app/drivers/mchp_mec5_i2c_nl.h>

#include <device_mec5.h>
#include <mec_i2c_api.h>
#include <mec_retval.h>

#include "app_config.h"
#ifdef CONFIG_I2C_TARGET
#include "app_i2c_target1.h"
#endif

#define APP_BOARD_HAS_FRAM_ATTACHED

#define I2C_TARGET_NOT_PRESENT_ADDR 0x56
#define I2C_TARGET_MODE_ADDR0 0x40
#define I2C_TARGET_MODE_ADDR1 0x44

/* LTC2489 ADC conversion time using internal clock.
 * If a converstion is in progress LTC2489 will NACK its address.
 */
#define LTC2489_ADC_CONV_TIME_MS 150
#define LTC2489_ADC_READ_RETRIES 10

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define I2C0_NODE	DT_ALIAS(i2c0)
#define I2C1_NODE	DT_ALIAS(i2c1)


static const struct device *i2c0_dev = DEVICE_DT_GET(I2C0_NODE);
static const struct device *i2c1_dev = DEVICE_DT_GET(I2C1_NODE);

#if DT_NODE_EXISTS(DT_ALIAS(i2c2))
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(i2c2))
#define APP_TEST_I2C_NL_ON_I2C2
static const struct device *i2c2_dev = DEVICE_DT_GET(DT_ALIAS(i2c2));
#endif
#endif

/* Access devices on an I2C bus using Device Tree child nodes of the I2C controller */
static const struct i2c_dt_spec pca9555_dts = I2C_DT_SPEC_GET(DT_NODELABEL(pca9555_evb));
static const struct i2c_dt_spec ltc2489_dts = I2C_DT_SPEC_GET(DT_NODELABEL(ltc2489_evb));

#if DT_NODE_EXISTS(DT_NODELABEL(mb85rc256v))
static const struct i2c_dt_spec mb85rc256v_dts = I2C_DT_SPEC_GET(DT_NODELABEL(mb85rc256v));
static const uint8_t fram_port = 4u; /* (uint8_t)(DT_PROP(DT_PARENT(DT_NODELABEL(mb85rc256v)), port)); */
#else
static const struct i2c_dt_spec mb85rc256v_dts = {0};
static const uint8_t fram_port = 255u;
#endif

/* ---- Global variables ---- */
static volatile uint32_t spin_val;
static volatile int ret_val;

static struct i2c_msg test_msgs[8];

#ifdef CONFIG_I2C_CALLBACK
struct i2c_cb_userdata {
	uint8_t *txbuf;
	uint32_t txlen;
	uint8_t *rxbuf;
	uint32_t rxlen;
};

volatile bool i2c_nl_cb_received;
volatile int i2c_nl_cb_result;
volatile void *i2c_nl_cb_userdata;
struct i2c_cb_userdata i2c_nl_cb_udata;
#endif

/* ---- prototypes ---- */
static void spin_on(uint32_t id, int rval);
static int config_i2c_device(const struct device *dev, uint32_t i2c_dev_config);

static int run_i2c_fram_tests(const struct device *idev, uint32_t i2c_config, uint8_t port,
			      uint16_t fram_i2c_addr);

static int test_i2c_fram(const struct device *dev, uint16_t i2c_addr,
			 uint16_t fram_offset, size_t datasz,
			 i2c_callback_t cb, void *userdata,
			 bool async);

static int test_i2c_fram_multi_msg(const struct device *dev, uint16_t i2c_addr,
				   uint16_t fram_offset, size_t datasz,
				   i2c_callback_t cb, void *userdata,
				   bool async);

#if defined(APP_TEST_I2C_NL_ON_I2C2) && defined(APP_BOARD_HAS_FRAM_ATTACHED)
static int test_i2c_nl(const struct device *i2c_nl_dev, uint32_t i2c_config,
                       uint8_t port, uint16_t i2_addr);
#ifdef CONFIG_I2C_CALLBACK
static void i2c_nl_callback(const struct device *dev, int result, void *userdata);
static int test_i2c_nl_cb(const struct device *i2c_nl_dev, uint32_t i2c_config, uint8_t port,
			  uint16_t i2c_addr);
#endif
#ifdef CONFIG_I2C_TARGET
static int test_i2c_target1(const struct device *host_i2c_dev,
                            uint32_t i2c_dev_config, uint8_t i2c_port,
                            uint16_t target_i2c_addr);
#endif
#endif

#define TEST_BUF_SIZE 512

uint8_t buf1[TEST_BUF_SIZE];
uint8_t buf2[TEST_BUF_SIZE];
uint8_t buf3[TEST_BUF_SIZE];

int main(void)
{
	int ret = 0;
	uint32_t i2c_dev_config = 0;
	uint32_t adc_retry_count = 0;
	uint32_t i2c_config = 0;
	uint32_t temp = 0;
	uint8_t nmsgs = 0;
	uint8_t target_addr = 0;
	struct i2c_msg msgs[4];

	LOG_INF("app_i2c sample: board: %s", DT_N_P_compatible_IDX_0);

	LOG_INF("size of struct k_sem   = %u bytes", sizeof(struct k_sem));
	LOG_INF("size of struct k_event = %u bytes", sizeof(struct k_event));

	memset(msgs, 0, sizeof(msgs));
	memset(buf1, 0x55, sizeof(buf1));
	memset(buf2, 0x55, sizeof(buf2));

	if (!device_is_ready(i2c0_dev)) {
		LOG_ERR("I2C0 device is not ready!");
		spin_on((uint32_t)__LINE__, -1);
	}

	i2c_dev_config = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;

	ret = config_i2c_device(i2c0_dev, i2c_dev_config);
	if (ret) {
		LOG_ERR("I2C configuration error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	ret = i2c_get_config(i2c0_dev, &temp);
	if (ret) {
		LOG_ERR("I2C get configuration error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	if (temp != i2c_dev_config) {
		LOG_ERR("I2C configuration does not match: orig(0x%08x) returned(0x%08x)\n",
			i2c_dev_config, temp);
		spin_on((uint32_t)__LINE__, ret);
	}

	ret = config_i2c_device(i2c1_dev, i2c_dev_config);
	if (ret) {
		LOG_ERR("I2C1 configuration error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	temp = UINT32_MAX;
	ret = i2c_get_config(i2c1_dev, &temp);
	if (ret) {
		LOG_ERR("I2C1 get configuration error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	if (temp != i2c_dev_config) {
		LOG_ERR("I2C1 configuration does not match: orig(0x%08x) returned(0x%08x)\n",
			i2c_dev_config, temp);
		spin_on((uint32_t)__LINE__, ret);
	}

	/* I2C write to non-existent device: check error return and if
	 * future I2C accesses to a real device work.
	 */
	LOG_INF("Attempt I2C transaction to a non-existent address");
	target_addr = I2C_TARGET_NOT_PRESENT_ADDR;
	nmsgs = 1u;
	buf1[0] = 2u;
	buf1[1] = 0x55u;
	buf1[2] = 0xaa;
	msgs[0].buf = buf1;
	msgs[0].len = 3u;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer(i2c0_dev, msgs, nmsgs, target_addr);
	if (ret != 0) {
		LOG_INF("PASS: Expected API to return an error (%d)", ret);
	} else {
		LOG_ERR("FAIL: I2C API should have returned an error!");
		spin_on((uint32_t)__LINE__, ret);
	}

	for (int i = 0; i < 3; i++) {
		LOG_INF("I2C Write 3 bytes to PCA9555 target device");

		nmsgs = 1u;
		buf1[0] = 2u; /* PCA9555 cmd=2 is output port 0 */
		buf1[1] = 0x55u;
		buf1[2] = 0xaau;
		msgs[0].buf = buf1;
		msgs[0].len = 3u;
		msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

		ret = i2c_transfer_dt(&pca9555_dts, msgs, nmsgs);
		if (ret) {
			LOG_ERR("Loop %d: I2C write to PCA9555 error (%d)", i, ret);
			spin_on((uint32_t)__LINE__, ret);
		}
	}

	LOG_INF("Write 3 bytes to PCA9555 target using multiple write buffers");
	nmsgs = 3u;
	buf1[0] = 2u;
	buf1[1] = 0x33u;
	buf1[2] = 0xcc;
	msgs[0].buf = buf1;
	msgs[0].len = 1u;
	msgs[0].flags = I2C_MSG_WRITE;
	msgs[1].buf = &buf1[1];
	msgs[1].len = 1u;
	msgs[1].flags = I2C_MSG_WRITE;
	msgs[2].buf = &buf1[2];
	msgs[2].len = 1u;
	msgs[2].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&pca9555_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C write multiple buffers to PCA9555 error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("Read both PCA9555 input ports");

	/* PCA9555 read protocol: START TX[wrAddr, cmd],
	 * Rpt-START TX[rdAddr],  RX[data,...], STOP
	 */
	nmsgs = 2u;
	buf1[0] = 0u; /* PCA9555 cmd=0 is input port 0 */
	buf2[0] = 0x55u; /* receive buffer */
	buf2[1] = 0x55u;

	msgs[0].buf = buf1;
	msgs[0].len = 1u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = buf2;
	msgs[1].len = 2u;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&pca9555_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C transfer error: write cmd byte, read 2 data bytes: (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("PCA9555 Port 0 input = 0x%02x  Port 1 input = 0x%02x", buf2[0], buf2[1]);

	LOG_INF("Read both PCA9555 input ports using different buffers for each port");
	nmsgs = 3u;

	buf1[0] = 0u; /* PCA9555 cmd=0 is input port 0 */
	buf2[0] = 0x55u; /* first receive buffer */
	buf2[1] = 0x55u;
	buf2[8] = 0x55u; /* second receive buffer */
	buf2[9] = 0x55u;

	msgs[0].buf = buf1;
	msgs[0].len = 1u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = buf2;
	msgs[1].len = 1u;
	msgs[1].flags = I2C_MSG_READ;

	msgs[2].buf = &buf2[8];
	msgs[2].len = 1u;
	msgs[2].flags = I2C_MSG_READ | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&pca9555_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C transfer error: 3 messages: (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("PCA9555 Port 0 input = 0x%02x  Port 1 input = 0x%02x", buf2[0], buf2[8]);

	LOG_INF("Select ADC channel 0 in LTC2489. This triggers a conversion!");
	nmsgs = 1;
	buf1[0] = 0xb0u; /* 1011_0000 selects channel 0 as single ended */
	msgs[0].buf = buf1;
	msgs[0].len = 1u;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&ltc2489_dts, msgs, nmsgs);
	if (ret) {
		LOG_ERR("I2C write to set LTC2489 channel failed: (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	/* wait for LTC2489 to finish a conversion */
	k_sleep(K_MSEC(LTC2489_ADC_CONV_TIME_MS));

	/* LTC2489 ADC read protocol is I2C read: START TX[rdAddr]
	 * RX[data[7:0], data[15:8], data[23:16]] STOP
	 */
	LOG_INF("Read 24-bit ADC reading from LTC2489");
	adc_retry_count = 0;
	do {
		buf2[0] = 0x55;
		buf2[1] = 0x55;
		buf2[2] = 0x55;

		nmsgs = 1;
		msgs[0].buf = buf2;
		msgs[0].len = 3u;
		msgs[0].flags = I2C_MSG_READ | I2C_MSG_STOP;

		ret = i2c_transfer_dt(&ltc2489_dts, msgs, nmsgs);
		if (ret) {
			adc_retry_count++;
			LOG_INF("LTC2489 read error (%d)", ret);
		}
	} while ((ret != 0) && (adc_retry_count < LTC2489_ADC_READ_RETRIES));

	if (ret == 0) {
		temp = ((uint32_t)(buf2[0]) + (((uint32_t)(buf2[1])) << 8)
				+ (((uint32_t)(buf2[2])) << 16));
		LOG_INF("LTC2489 reading = 0x%08x", temp);
	}

	if (device_is_ready(mb85rc256v_dts.bus)) {
		LOG_INF("MB85RC256V FRAM exists. Test I2C FRAM access");
		i2c_config = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;
		run_i2c_fram_tests(mb85rc256v_dts.bus, i2c_config, fram_port, mb85rc256v_dts.addr);

		LOG_INF("Test I2C-NL driver on I2C2");
		ret = test_i2c_nl(i2c2_dev, i2c_config, fram_port, mb85rc256v_dts.addr);
		LOG_INF("I2C-NL test returned %d", ret);

#ifdef CONFIG_I2C_CALLBACK
		LOG_INF("Test I2C-NL driver callback API on I2C2");
		ret = test_i2c_nl_cb(i2c2_dev, i2c_config, fram_port, mb85rc256v_dts.addr);
		LOG_INF("I2C-NL CB test returned %d", ret);
#endif
	} else {
		LOG_INF("MB85RC256V FRAM exists does not exist. Skipping I2C FRAM tests");
	}

#ifdef CONFIG_I2C_TARGET
	i2c_dev_config = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;

	ret = i2c_target1_config(i2c1_dev, i2c2_dev, i2c_dev_config, fram_port,
				 I2C_TARGET_MODE_ADDR0);
	if (ret) {
		LOG_ERR("I2C target 1 config error (%d)", ret);
		spin_on((uint32_t)__LINE__, ret);
	}

	LOG_INF("main thread: Let other threads run for 5 seconds for starting tests");
	k_sleep(K_MSEC(5000));

	test_i2c_target1(i2c1_dev, i2c_dev_config, fram_port, I2C_TARGET_MODE_ADDR0);
#endif

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

static int config_i2c_device(const struct device *dev, uint32_t i2c_dev_config)
{
	uint32_t temp = 0;
	int ret = 0;

	if (!dev) {
		return -EINVAL;
	}

	ret = i2c_configure(dev, i2c_dev_config);
	if (ret) {
		LOG_ERR("I2C configuration error (%d)", ret);
		return  ret;
	}

	ret = i2c_get_config(dev, &temp);
	if (ret) {
		LOG_ERR("I2C get configuration error (%d)", ret);
		return ret;
	}

	if (temp != i2c_dev_config) {
		ret = -EIO;
		LOG_ERR("I2C configuration does not match: orig(0x%08x) returned(0x%08x)\n",
			i2c_dev_config, temp);
	}

	return ret;
}

static int run_i2c_fram_tests(const struct device *idev, uint32_t i2c_config, uint8_t port,
			      uint16_t fram_i2c_addr)
{
	int ret = 0;
	uint32_t fram_datasz = 0;
	uint16_t fram_addr = 0, target_addr = 0;
	uint8_t nmsgs = 0, prev_port = 0;
	struct i2c_msg msgs[4] = {0};

	LOG_INF("Run I2C FRAM tests");

	ret = i2c_mchp_get_port(idev, &prev_port);
	if (ret) {
		LOG_ERR("MCHP I2C driver get port API error (%d)", ret);
		return ret;
	}

	LOG_INF("Re-configure to use FRAM port");
	ret = i2c_mchp_configure(idev, i2c_config, port);
	if (ret) {
		LOG_ERR("MCHP I2C driver reconfig to new port API error (%d)", ret);
		return ret;
	}

	LOG_INF("Attempt I2C transaction to a non-existent address");
	target_addr = I2C_TARGET_NOT_PRESENT_ADDR;
	nmsgs = 1u;
	buf1[0] = 2u;
	buf1[1] = 0x55u;
	buf1[2] = 0xaa;
	msgs[0].buf = buf1;
	msgs[0].len = 3u;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer(idev, msgs, nmsgs, target_addr);
	if (ret) {
		LOG_INF("API returned error (%d) as expected", ret);
	} else {
		LOG_ERR("Expected an error due to NAK. API returned success");
		return -EIO;
	}

	fram_addr = 0;
	fram_datasz = 32u;
	ret = test_i2c_fram(idev, fram_i2c_addr, fram_addr, fram_datasz, NULL, NULL, false);
	if (ret) {
		LOG_ERR("Test fail: reconfigure");
		i2c_mchp_configure(idev, i2c_config, port);
	}

	fram_addr = 0x1234u;
	fram_datasz = 64u;
	test_i2c_fram(idev, fram_i2c_addr, fram_addr, fram_datasz, NULL, NULL, false);
	if (ret) {
		LOG_ERR("Test fail: reconfigure");
		i2c_mchp_configure(idev, i2c_config, port);
	}

	fram_addr = 0x100u;
	fram_datasz = 128u;
	test_i2c_fram(idev, fram_i2c_addr, fram_addr, fram_datasz, NULL, NULL, false);
	if (ret) {
		LOG_ERR("Test fail: reconfigure");
		i2c_mchp_configure(idev, i2c_config, port);
	}

	fram_addr = 0x300u;
	fram_datasz = 512u;
	test_i2c_fram(idev, fram_i2c_addr, fram_addr, fram_datasz, NULL, NULL, false);
	if (ret) {
		LOG_ERR("Test fail: reconfigure");
		i2c_mchp_configure(idev, i2c_config, port);
	}

	LOG_INF("Test multi-message");
	fram_addr = 0x600u;
	fram_datasz = 16u;
	test_i2c_fram_multi_msg(idev, fram_i2c_addr, fram_addr, fram_datasz, NULL, NULL, false);

	LOG_INF("Re-configure to original port");
	ret = i2c_mchp_configure(idev, i2c_config, prev_port);
	if (ret) {
		LOG_ERR("MCHP I2C driver reconfig to new port API error (%d)", ret);
	}

	return 0;
}

static int test_i2c_fram(const struct device *dev, uint16_t i2c_addr,
			 uint16_t fram_offset, size_t datasz,
			 i2c_callback_t cb, void *userdata,
			 bool async)
{
	struct i2c_msg msgs[4] = { 0 };
	int ret = 0;
	uint8_t nmsgs = 0;

	if (!dev) {
		LOG_ERR("I2C FRAM test bad device parameter");
		return -EINVAL;
	}

	if (!datasz) {
		LOG_INF("I2C FRAM test data size is 0. Nothing to do");
		return 0;
	}

	if (((uint32_t)fram_offset + datasz) > (32u * 1024u)) {
		LOG_ERR("I2C FRAM test: offset + datasz overflows 32KB FRAM size");
		return -EMSGSIZE;
	}

	if (datasz > sizeof(buf2)) {
		LOG_ERR("I2C FRAM test requested data size exceeds "
			"test buffer size of %u bytes", sizeof(buf2));
		return -EMSGSIZE;
	}

	for (size_t n = 0; n < datasz; n++) {
		buf2[n] = (uint8_t)(n % 256u);
	}

	buf1[0] = (uint8_t)((fram_offset >> 8) & 0xffu); /* address b[15:8] */
	buf1[1] = (uint8_t)((fram_offset) & 0xffu); /* address b[7:0] */

	LOG_INF("MB85RC256V FRAM write %u bytes to offset 0x%x", datasz, fram_offset);

	msgs[0].buf = buf1;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = buf2;
	msgs[1].len = datasz;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;


	nmsgs = 2;
	ret = i2c_transfer(dev, msgs, nmsgs, i2c_addr);
	if (ret) {
		LOG_ERR("I2C API for FRAM write returned error %d", ret);
		return ret;
	}

	/* fill receive buffer with 0x55 */
	memset(buf3, 0x55, sizeof(buf3));

	LOG_INF("MB85RC256V FRAM read %u bytes from offset 0x%x", datasz, fram_offset);

	/* I2C Combined Write FRAM offset and read data */
	msgs[0].buf = buf1;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = buf3;
	msgs[1].len = datasz;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	nmsgs = 2;
	ret = i2c_transfer(dev, msgs, nmsgs, i2c_addr);
	if (ret) {
		LOG_ERR("I2C API for FRAM write-read returned error %d", ret);
		return ret;
	}

	ret = memcmp(buf2, buf3, datasz);
	if (ret == 0) {
		LOG_INF("I2C FRAM Test write data and read back at "
			"offset 0x%0x length %u: PASS", fram_offset, datasz);
	} else {
		LOG_ERR("I2C FRAM Test write data and read back FAIL");
	}

	return ret;
}

/* Trigger driver to parse messages into more than one group */
static int test_i2c_fram_multi_msg(const struct device *dev, uint16_t i2c_addr,
				   uint16_t fram_offset, size_t datasz,
				   i2c_callback_t cb, void *userdata,
				   bool async)
{
	struct i2c_msg msgs[4] = { 0 };
	int ret = 0;
	uint8_t nmsgs = 0;

	if (!dev) {
		LOG_ERR("I2C FRAM test bad device parameter");
		return -EINVAL;
	}

	if (!datasz) {
		LOG_INF("I2C FRAM test data size is 0. Nothing to do");
		return 0;
	}

	if (((uint32_t)fram_offset + datasz) > (32u * 1024u)) {
		LOG_ERR("I2C FRAM test: offset + datasz overflows 32KB FRAM size");
		return -EMSGSIZE;
	}

	if (datasz > sizeof(buf2)) {
		LOG_ERR("I2C FRAM test requested data size exceeds "
			"test buffer size of %u bytes", sizeof(buf2));
		return -EMSGSIZE;
	}

	/* fill receive buffer with 0x55 */
	memset(buf2, 0x55, sizeof(buf2));
	memset(buf3, 0x55, sizeof(buf3));

	for (size_t n = 0; n < datasz; n++) {
		buf2[n] = (uint8_t)(n % 256u);
	}

	buf1[0] = (uint8_t)((fram_offset >> 8) & 0xffu); /* address b[15:8] */
	buf1[1] = (uint8_t)((fram_offset) & 0xffu); /* address b[7:0] */

	LOG_INF("MB85RC256V FRAM write %u bytes to offset 0x%x and read back", datasz, fram_offset);

	msgs[0].buf = buf1;
	msgs[0].len = 2u;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = buf2;
	msgs[1].len = datasz;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	msgs[2].buf = buf1;
	msgs[2].len = 2u;
	msgs[2].flags = I2C_MSG_WRITE;

	msgs[3].buf = buf3;
	msgs[3].len = datasz;
	msgs[3].flags = I2C_MSG_READ | I2C_MSG_STOP;

	nmsgs = 4;
	ret = i2c_transfer(dev, msgs, nmsgs, i2c_addr);
	if (ret) {
		LOG_ERR("I2C API passed multi-messages returned error %d", ret);
		return ret;
	}

	ret = memcmp(buf2, buf3, datasz);
	if (ret == 0) {
		LOG_INF("I2C FRAM Test write data and read back at "
			"offset 0x%0x length %u: PASS", fram_offset, datasz);
	} else {
		LOG_ERR("I2C FRAM Test write data and read back FAIL");
	}

	return ret;
}

#if defined(APP_TEST_I2C_NL_ON_I2C2) && defined(APP_BOARD_HAS_FRAM_ATTACHED)
static int test_i2c_nl(const struct device *i2c_nl_dev, uint32_t i2c_config, uint8_t port,
			uint16_t i2c_addr)
{
	int ret = 0;
	uint8_t num_msgs = 0, prev_port = 0;

	LOG_INF("Test I2C-NL: Controller I2C02 on Port 5");

	memset(test_msgs, 0, sizeof(test_msgs));

	if (!i2c_nl_dev) {
		LOG_ERR("Test I2C-NL was passed NULL device struct!");
		return -EINVAL;
	}

	ret = i2c_mchp_nl_get_port(i2c_nl_dev, &prev_port);
	if (ret) {
		LOG_ERR("MCHP I2C driver get port API error (%d)", ret);
		return ret;
	}

	LOG_INF("Re-configure to use FRAM port");
	ret = i2c_mchp_nl_configure(i2c_nl_dev, i2c_config, port);
	if (ret) {
		LOG_ERR("MCHP I2C driver reconfig to new port API error (%d)", ret);
		return ret;
	}

	memset(buf1, 0, sizeof(buf1));
	memset(buf2, 0, sizeof(buf2));
	memset(buf3, 0x55, sizeof(buf3));

	/* I2C Write to FRAM: 2-byte offset plus 4 data bytes */
	LOG_INF("Test one message: Write-STOP: FRAM offset plus 4 data bytes");
	buf1[0] = 0x23u;
	buf1[1] = 0x14u;
	buf1[2] = 0x18u;
	buf1[3] = 0x19u;
	buf1[4] = 0x1Au;
	buf1[5] = 0x1Bu;

	test_msgs[0].buf = buf1;
	test_msgs[0].len = 6u;
	test_msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	num_msgs = 1u;
	i2c_addr = mb85rc256v_dts.addr;
	ret = i2c_transfer(i2c_nl_dev, test_msgs, num_msgs, i2c_addr);
	if (ret) {
		LOG_ERR("I2C-NL transfer API error (%d)", ret);
		return ret;
	}

	/* Test Write-Read using two messages to read back the data written above */
	LOG_INF("Test two msg Write-Read of data written above");
	buf1[0] = 0x23u;
	buf1[1] = 0x14u;
	test_msgs[0].buf = buf1;
	test_msgs[0].len = 2u;
	test_msgs[0].flags = I2C_MSG_WRITE;

	test_msgs[1].buf = buf3;
	test_msgs[1].len = 4u;
	test_msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	num_msgs = 2u;
	i2c_addr = mb85rc256v_dts.addr;
	ret = i2c_transfer(i2c_nl_dev, test_msgs, num_msgs, i2c_addr);
	if (ret) {
		LOG_ERR("I2C-NL transfer API error (%d)", ret);
		return ret;
	}

	ret = memcmp(&buf1[2], buf3, 4u);
	if (ret) {
		LOG_ERR("Write-Read 2 msg: data mismatch: FAIL");
	} else {
		LOG_INF("Write-Read 2msg: data match: PASS");
	}

	memset(buf3, 0x55, sizeof(buf3));

	/* I2C Write to FRAM: 2-byte offset plus 4 data bytes */
	LOG_INF("Test two: Write messages: FRAM offset plus 4 data bytes");
	buf1[0] = 0x23u;
	buf1[1] = 0x14u;
	buf1[4] = 0x18u;
	buf1[5] = 0x19u;
	buf1[6] = 0x1Au;
	buf1[7] = 0x1Bu;

	test_msgs[0].buf = buf1;
	test_msgs[0].len = 2u;
	test_msgs[0].flags = I2C_MSG_WRITE;

	test_msgs[1].buf = &buf1[4];
	test_msgs[1].len = 4u;
	test_msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	num_msgs = 2u;
	i2c_addr = mb85rc256v_dts.addr;
	ret = i2c_transfer(i2c_nl_dev, test_msgs, num_msgs, i2c_addr);
	if (ret) {
		LOG_ERR("I2C-NL transfer API error (%d)", ret);
		return ret;
	}

	return ret;
}

#ifdef CONFIG_I2C_CALLBACK
/* Applicaiton callback registered with I2C driver i2c_transfer_cb() API. */
static void i2c_nl_callback(const struct device *dev, int result, void *userdata)
{
	i2c_nl_cb_received = true;
	i2c_nl_cb_result = result;
	i2c_nl_cb_userdata = userdata;
}

static int test_i2c_nl_cb(const struct device *i2c_nl_dev, uint32_t i2c_config, uint8_t port,
			  uint16_t i2c_addr)
{
	uint64_t wcnt = 0;
	int ret = 0, rval = 0;
	uint8_t num_msgs = 0, prev_port = 0;

	LOG_INF("Test I2C-NL callback API: Controller I2C02 on Port 5");

	memset(test_msgs, 0, sizeof(test_msgs));

	if (!i2c_nl_dev) {
		LOG_ERR("Test I2C-NL CB was passed NULL device struct!");
		return -EINVAL;
	}

	ret = i2c_mchp_nl_get_port(i2c_nl_dev, &prev_port);
	if (ret) {
		LOG_ERR("MCHP I2C CB driver get port API error (%d)", ret);
		return ret;
	}

	LOG_INF("I2C-NL CB: Re-configure to use FRAM port");
	ret = i2c_mchp_nl_configure(i2c_nl_dev, i2c_config, port);
	if (ret) {
		LOG_ERR("I2C-NL CB: driver reconfig to new port API error (%d)", ret);
		return ret;
	}

	memset(buf1, 0, sizeof(buf1));
	memset(buf2, 0, sizeof(buf2));
	memset(buf3, 0x55, sizeof(buf3));

	/* I2C Write 16-bit FRAM offset (MSB first) plus 4 data bytes */
	LOG_INF("I2C-NL CB: Test one write-stop");

	i2c_nl_cb_received = false;
	i2c_nl_cb_result = 0;
	i2c_nl_cb_userdata = NULL;
	i2c_nl_cb_udata.txbuf = buf1;
	i2c_nl_cb_udata.txlen = 6u;
	i2c_nl_cb_udata.rxbuf = NULL;
	i2c_nl_cb_udata.rxlen = 0;

	buf1[0] = 0x65u;
	buf1[1] = 0x40u;
	buf1[2] = 0x31u;
	buf1[3] = 0x32u;
	buf1[4] = 0x33u;
	buf1[5] = 0x34u;

	test_msgs[0].buf = buf1;
	test_msgs[0].len = 6u;
	test_msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	num_msgs = 1u;
	i2c_addr = mb85rc256v_dts.addr;
	ret = i2c_transfer_cb(i2c_nl_dev, test_msgs, num_msgs, i2c_addr, i2c_nl_callback,
			      (void *)&i2c_nl_cb_udata);
	if (ret) {
		LOG_ERR("I2C-NL CB transfer API error (%d)", ret);
		return ret;
	}

	/* Spin waiting for driver to invoke callback */
	while (!i2c_nl_cb_received) {
		wcnt++;
	}

	LOG_INF("I2C-NL CB: callback received: wcnt = %llu", wcnt);
	LOG_INF("  result = %d", i2c_nl_cb_result);
	LOG_INF("  userdata ptr = %p", i2c_nl_cb_userdata);

	if (i2c_nl_cb_userdata != (void *)&i2c_nl_cb_udata) {
		LOG_ERR("  userdata ptr does pointer passed to API: %p", &i2c_nl_cb_udata);
		rval = -1;
	}

	LOG_INF("Read back 4 bytes of data using I2C combined write-read message sequence");

	wcnt = 0u;
	i2c_nl_cb_received = false;
	i2c_nl_cb_result = 0;
	i2c_nl_cb_userdata = NULL;
	i2c_nl_cb_udata.txbuf = buf1;
	i2c_nl_cb_udata.txlen = 2u;
	i2c_nl_cb_udata.rxbuf = buf3;
	i2c_nl_cb_udata.rxlen = 4u;

	buf1[0] = 0x65u;
	buf1[1] = 0x40u;
	test_msgs[0].buf = buf1;
	test_msgs[0].len = 2u;
	test_msgs[0].flags = I2C_MSG_WRITE;

	test_msgs[1].buf = buf3;
	test_msgs[1].len = 4u;
	test_msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	num_msgs = 2u;
	i2c_addr = mb85rc256v_dts.addr;
	ret = i2c_transfer_cb(i2c_nl_dev, test_msgs, num_msgs, i2c_addr, i2c_nl_callback,
			      (void *)&i2c_nl_cb_udata);
	if (ret) {
		LOG_ERR("I2C-NL CB read-back: transfer API error (%d)", ret);
		return ret;
	}

	/* Spin waiting for driver to invoke callback */
	while (!i2c_nl_cb_received) {
		wcnt++;
	}

	LOG_INF("I2C-NL CB: callback received: wcnt = %llu", wcnt);
	LOG_INF("  result = %d", i2c_nl_cb_result);
	LOG_INF("  userdata ptr = %p", i2c_nl_cb_userdata);

	if (i2c_nl_cb_userdata != (void *)&i2c_nl_cb_udata) {
		LOG_ERR("  userdata ptr does pointer passed to API: %p", &i2c_nl_cb_udata);
		rval = -1;
	}

	ret = memcmp(&buf1[2], buf3, 4u);
	if (ret) {
		LOG_ERR("Write-Read 2 msg: data mismatch: FAIL");
		rval += -2;
	} else {
		LOG_INF("Write-Read 2msg: data match: PASS");
	}

	return rval;
}
#endif

#ifdef CONFIG_I2C_TARGET
static int test_i2c_target1(const struct device *host_i2c_dev, uint32_t i2c_dev_config,
			    uint8_t i2c_port, uint16_t target_i2c_addr)
{
	int ret = 0;
	uint8_t nmsgs = 0;

	memset(test_msgs, 0, sizeof(test_msgs));

	/* I2C Write to target 1 */
	buf1[0] = 0x11u;
	buf1[1] = 0x12u;

	test_msgs[0].buf = buf1;
	test_msgs[0].len = 1u;
	test_msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	nmsgs = 1;

	ret = i2c_transfer(host_i2c_dev, test_msgs, nmsgs, target_i2c_addr);
	if (ret) {
		LOG_ERR("I2C01 tranfer to Target 1 returned (%d)", ret);
	}

	/* TODO how do we know target 1 processed the message?
	 * For now use a large delay
	 */
	k_sleep(K_MSEC(5000));


	/* I2C Write to target 1 */
	buf1[0] = 0x21u;
	buf1[1] = 0x22u;
	buf1[2] = 0x23u;
	buf1[3] = 0x24u;

	test_msgs[0].buf = buf1;
	test_msgs[0].len = 4u;
	test_msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	nmsgs = 1;

	ret = i2c_transfer(host_i2c_dev, test_msgs, nmsgs, target_i2c_addr);
	if (ret) {
		LOG_ERR("I2C01 tranfer to Target 1 returned (%d)", ret);
	}

	/* TODO how do we know target 1 processed the message?
	 * For now use a large delay
	 */
	k_sleep(K_MSEC(5000));

	return 0;
}
#endif
#endif
