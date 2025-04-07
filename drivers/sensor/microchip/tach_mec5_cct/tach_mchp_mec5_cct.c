/* Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_tach_cct

#include <errno.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>

#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include <device_mec5.h>
#include <mec_pcr_api.h>
#include <mec_tach_api.h>

LOG_MODULE_REGISTER(tach_mec5_icct, CONFIG_SENSOR_LOG_LEVEL);

#define MEC5_CCT_MAX_CAPTURE 6

struct tach_mec5_cct_dev_cfg {
	struct mec_cct_regs *const regs;
	const struct pinctrl_dev_config *pcfg;
	uint32_t compare_mux;
#ifdef CONFIG_TACH_MEC5_INTERRUPT
	void (*irq_config)(void);
#endif
};

struct tach_mec5_cct_dev_data {
#ifdef CONFIG_TACH_MEC5_CCT_INTERRUPT
	struct k_sem sync;
#endif
	uint32_t ctrl;
	uint32_t capture[MEC5_CCT_MAX_CAPTURE];
};

#define TACH_MEC5_FAN_STOPPED		0xFFFFU
#define TACH_MEC5_SEC_PER_MINUTE	60U
#define TACH_MEC5_POLL_LOOP_COUNT	20U

/* If interrupt are used wait timeout on TACH ISR */
#define TACH_MEC5_SYNC_WAIT_MS		20u

/* Microchip Capture and Compare timer used as a simple tachometer.
 * The CCT has a 32-bit free running timer with frequency divider.
 * Six 32-bit capture registers and two 32-bit compare registers.
 * Capture feature:
 * MUX selecting the pin
 * Selectable edge: falling, rising, or both edges
 * Filter clock select and enable.
 * 32-bit count from the free running counter when capture event occurs.
 * Status indicating capture occurred with an individual interrupt line to
 * ECIA GIRQ18 bits[21:26] or directly to the NVIC.
 *
 * 
 */

/* Fetch a sample from the sensor and store it in an internal driver buffer
 *
 * Read all of a sensor's active channels and, if necessary, perform any
 * additional operations necessary to make the values useful.  The user
 * may then get individual channel values by calling sensor_channel_get.
 *
 * The function blocks until the fetch operation is complete.
 *
 * Since the function communicates with the sensor device, it is unsafe
 * to call it in an ISR if the device is connected via I2C or SPI.
 *
 * dev is a pointer to the sensor device
 *
 * returns 0 if successful, negative errno code if failure.
 *
 * NOTE: If the fan stops for some reason the resulting count value is maximum.
 * Set RPM value to 0 in this case.
 */
static int tach_mec5_cct_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	return -ENOSYS;
}

/* Get a reading from a sensor device
 *
 * Return a useful value for a particular channel, from the driver's
 * internal data.  Before calling this function, a sample must be
 * obtained by calling sensor_sample_fetch or sensor_sample_fetch_chan.
 * It is guaranteed that two subsequent calls of this function for the
 * same channels will yield the same value, if sensor_sample_fetch or
 * sensor_sample_fetch_chan has not been called in the meantime.
 *
 * dev is a pointer to the sensor device
 * chan is the channel to read
 * val is where to store the value
 *
 * return 0 if successful, negative errno code if failure.
 *
 * NOTES:
 * We must convert the tachometer count to revolutions per minute (rpm)
 *
 * case 1: Tachometer reading mode is increment counter on rising edge of the
 * tachometer input signal. Not implemented at this time. Return -EIO.
 *
 * case 2: Tachometer reading mode is increment counter on rising edge of the
 * PCR slow clock (default 100 KHz).  The PCR slow clock defaults to 100 KHz but
 * can be changed.
 *
 */
static int tach_mec5_cct_channel_get(const struct device *dev,
				 enum sensor_channel chan,
				 struct sensor_value *sval)
{
	return -ENOSYS;
}

static int tach_mec5_cct_dev_init(const struct device *dev)
{
	const struct tach_mec5_cct_dev_cfg *const devcfg = dev->config;
	struct tach_mec5_cct_dev_data *data = dev->data;
	struct mec_cct_regs *const regs = devcfg->regs;
	int ret = 0;

	mec_hal_pcr_blk_reset(MEC_PCR_CCT0);
	data->ctrl = regs->TCTRL;

	regs->TCTRL |= BIT(MEC_CCT_TCTRL_ACTV_Pos);

	ret = pinctrl_apply_state(devcfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("MEC5 TACH PINCTRL init failed (%d)", ret);
		return ret;
	}

	return 0;
}

static const struct sensor_driver_api tach_mec5_cct_driver_api = {
	.sample_fetch = tach_mec5_cct_sample_fetch,
	.channel_get = tach_mec5_cct_channel_get,
};

#define TACH_MEC5_CMV(inst, idx) \
	DT_INST_PROP_BY_IDX(inst, compare_channels, idx) << ((idx) * 4)

#define TACH_MEC5_CCT_CMPMUX(i) \
	(COND_CODE_1(DT_INST_PROP_HAS_IDX(i, compare_channels, 0), (TACH_MEC5_CMV(i, 0)), (0)) |\
	 COND_CODE_1(DT_INST_PROP_HAS_IDX(i, compare_channels, 1), (TACH_MEC5_CMV(i, 1)), (0)) |\
	 COND_CODE_1(DT_INST_PROP_HAS_IDX(i, compare_channels, 2), (TACH_MEC5_CMV(i, 2)), (0)) |\
	 COND_CODE_1(DT_INST_PROP_HAS_IDX(i, compare_channels, 3), (TACH_MEC5_CMV(i, 3)), (0)) |\
	 COND_CODE_1(DT_INST_PROP_HAS_IDX(i, compare_channels, 4), (TACH_MEC5_CMV(i, 4)), (0)) |\
	 COND_CODE_1(DT_INST_PROP_HAS_IDX(i, compare_channels, 5), (TACH_MEC5_CMV(i, 5)), (0)))

#ifdef CONFIG_TACH_MEC5_CCT_INTERRUPT
#define TACH_MEC5_CCT_IRQ_CFG(i)                                        \
	static void tach_mec5_irq_cct_cfg_##i(void)                     \
	{                                                               \
		IRQ_CONNECT(DT_INST_IRQN(i),                            \
			    DT_INST_IRQ(i, priority),                   \
			    tach_mec5_cct_isr,                          \
			    DEVICE_DT_INST_GET(i), 0);                  \
		irq_enable(DT_INST_IRQN(i));                            \
	}
#define TACH_MEC5_CCT_DEV_CFG(i)                                                \
	static const struct tach_mec5_cct_dev_cfg tach_mec5_cct_devcfg_##i = {  \
		.regs = (struct mec_cct_regs *)DT_INST_REG_ADDR(i),             \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i),                      \
		.compare_mux = TACH_MEC5_CCT_CMPMUX(i),                         \
		.irq_config = tach_mec5_cct_irq_cfg_##i,                        \
	}
#else
#define TACH_MEC5_CCT_IRQ_CFG(i)
#define TACH_MEC5_CCT_DEV_CFG(i)                                                \
	static const struct tach_mec5_cct_dev_cfg tach_mec5_cct_devcfg_##i = {  \
		.regs = (struct mec_cct_regs *)DT_INST_REG_ADDR(i),             \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i),                      \
		.compare_mux = TACH_MEC5_CCT_CMPMUX(i),                         \
	}
#endif

#define TACH_MEC5_CCT_DEVICE(id)                                        \
	static struct tach_mec5_cct_dev_data tach_mec5_cct_data_##id;   \
	PINCTRL_DT_INST_DEFINE(id);                                     \
	TACH_MEC5_CCT_IRQ_CFG(id)                                       \
	TACH_MEC5_CCT_DEV_CFG(id);                                      \
	PM_DEVICE_DT_INST_DEFINE(id, tach_mec5_cct_pm_action);          \
	SENSOR_DEVICE_DT_INST_DEFINE(id,                                \
			    tach_mec5_cct_dev_init,                     \
			    PM_DEVICE_DT_INST_GET(id),                  \
			    &tach_mec5_cct_data_##id,                   \
			    &tach_mec5_cct_devcfg_##id,                 \
			    POST_KERNEL,                                \
			    CONFIG_SENSOR_INIT_PRIORITY,                \
			    &tach_mec5_cct_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TACH_MEC5_CCT_DEVICE)
