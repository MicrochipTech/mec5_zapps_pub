/* Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_tach_cct_cap

#include <errno.h>
#include <soc.h>
#include <app/drivers/sensor/mchp_mec5_cct_tach.h>
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
#include <mec_cct_api.h>
#include <mec_pcr_api.h>

LOG_MODULE_REGISTER(mec5_cct_cap, CONFIG_SENSOR_LOG_LEVEL);

#define NS_PER_SEC (NSEC_PER_MSEC * MSEC_PER_SEC)
#define TACH_INTERVAL_EDGES 3

struct cct_cap_config {
	uint8_t unit_id;
	uint8_t edge_detect;
	uint8_t filter_clk;
	uint8_t mux;
};

struct cct_cap_counts {
	volatile uint32_t count;
	volatile int64_t interval;
};

struct tach_mec5_cct_cap_dev_cfg {
	struct mec_cct_regs *const regs;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config)(void);
	uint8_t fr_clkdiv;
	uint8_t num_cap_chans;
	uint16_t cap_bm;
	const struct cct_cap_config *capcfgs;
};

struct tach_mec5_cct_cap_dev_data {
	const struct device *dev;
	struct k_mutex lock;
	struct k_event events;
	const struct sensor_trigger *trigger;
	sensor_trigger_handler_t thandler;
	uint32_t ctrl;
	uint16_t oneshot;
	volatile uint16_t ready;
	volatile uint8_t trigger_counts[MEC_CCT_CAP_ID_MAX];
	struct cct_cap_counts captures[MEC_CCT_CAP_ID_MAX];
};

static struct cct_cap_config const *get_cap_cfg(const struct device *dev, uint8_t capid)
{
	const struct tach_mec5_cct_cap_dev_cfg *const devcfg = dev->config;

	for (uint8_t i = 0; i < devcfg->num_cap_chans; i++) {
		const struct cct_cap_config *cfg = &devcfg->capcfgs[i];

		if (cfg->unit_id == capid) {
			return cfg;
		}
	}

	return NULL;
}

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
 */

static void tach_mec5_cct_fr_isr(void *arg)
{
	const struct device *dev = (struct device *)arg;
	const struct tach_mec5_cct_cap_dev_cfg *const devcfg = dev->config;
	struct mec_cct_regs *regs = devcfg->regs;

	mec_hal_cct_clear_irq(regs, BIT(MEC_CCT_IRQ_SRC_FR_POS));
}

static uint32_t cnt32_diff(uint32_t cnt1, uint32_t cnt2)
{
	if (cnt2 > cnt1) {
		return cnt2 - cnt1;
	} else {
		return (UINT32_MAX - cnt1) + cnt2;
	}
}

#if 1
static void tach_mec5_cct_cap_handler(const struct device *dev, uint8_t capid,
				      uint32_t capcnt, uint32_t frcnt)
{
	const struct tach_mec5_cct_cap_dev_cfg *const devcfg = dev->config;
	struct mec_cct_regs *regs = devcfg->regs;
	struct tach_mec5_cct_cap_dev_data *data = dev->data;
	struct cct_cap_counts *captures = &data->captures[capid]; 
	uint32_t cap_ibm = BIT(MEC_CCT_IRQ_SRC_CAP0_POS + capid);
	uint8_t ncap = data->trigger_counts[capid];

	if (ncap < TACH_INTERVAL_EDGES) {
		if (ncap == 0) {
			captures->interval = 0;
		} else {
			captures->interval += cnt32_diff(captures->count, capcnt);
		}
		ncap++;
		captures->count = capcnt;
		data->trigger_counts[capid] = ncap;
	}

	if (ncap >= TACH_INTERVAL_EDGES) {
		data->ready |= BIT(capid);
		mec_hal_cct_cap_ctrl(regs, capid, MEC_CCT_CAP_IDET_DIS);
		mec_hal_cct_clear_irq(regs, cap_ibm);
		k_event_post(&data->events, BIT(capid));
	}
}
#else /* previous */
static void tach_mec5_cct_cap_handler(const struct device *dev, uint8_t capid,
				      uint32_t capcnt, uint32_t frcnt)
{
	const struct tach_mec5_cct_cap_dev_cfg *const devcfg = dev->config;
	struct mec_cct_regs *regs = devcfg->regs;
	struct tach_mec5_cct_cap_dev_data *data = dev->data;
	struct cct_cap_counts *captures = &data->captures[capid]; 
	uint32_t cap_ibm = BIT(MEC_CCT_IRQ_SRC_CAP0_POS + capid);
	uint8_t ncap = data->trigger_counts[capid];

	if (ncap < TACH_INTERVAL_EDGES) {
		captures->counts[ncap] = capcnt;
		ncap++;
		data->trigger_counts[capid] = ncap;
	}

	if (ncap >= TACH_INTERVAL_EDGES) {
		data->ready |= BIT(capid);
		mec_hal_cct_cap_ctrl(regs, capid, MEC_CCT_CAP_IDET_DIS);
		mec_hal_cct_clear_irq(regs, cap_ibm);
		k_event_post(&data->events, BIT(capid));
	}
}
#endif

static void tach_mec5_cct_cap0_isr(void *arg)
{
	const struct device *dev = (struct device *)arg;
	const struct tach_mec5_cct_cap_dev_cfg *const devcfg = dev->config;
	struct mec_cct_regs *regs = devcfg->regs;
	uint32_t capcnt = regs->CAP0_CNT;
	uint32_t frcnt = regs->FR_COUNT;

	tach_mec5_cct_cap_handler(dev, MEC_CCT_CAP0_ID, capcnt, frcnt);
}

static void tach_mec5_cct_cap1_isr(void *arg)
{
	const struct device *dev = (struct device *)arg;
	const struct tach_mec5_cct_cap_dev_cfg *const devcfg = dev->config;
	struct mec_cct_regs *regs = devcfg->regs;
	uint32_t capcnt = regs->CAP1_CNT;
	uint32_t frcnt = regs->FR_COUNT;

	tach_mec5_cct_cap_handler(dev, MEC_CCT_CAP1_ID, capcnt, frcnt);
}

static void tach_mec5_cct_cap2_isr(void *arg)
{
	const struct device *dev = (struct device *)arg;
	const struct tach_mec5_cct_cap_dev_cfg *const devcfg = dev->config;
	struct mec_cct_regs *regs = devcfg->regs;
	uint32_t capcnt = regs->CAP2_CNT;
	uint32_t frcnt = regs->FR_COUNT;

	tach_mec5_cct_cap_handler(dev, MEC_CCT_CAP2_ID, capcnt, frcnt);
}

static void tach_mec5_cct_cap3_isr(void *arg)
{
	const struct device *dev = (struct device *)arg;
	const struct tach_mec5_cct_cap_dev_cfg *const devcfg = dev->config;
	struct mec_cct_regs *regs = devcfg->regs;
	uint32_t capcnt = regs->CAP3_CNT;
	uint32_t frcnt = regs->FR_COUNT;

	tach_mec5_cct_cap_handler(dev, MEC_CCT_CAP3_ID, capcnt, frcnt);
}

static void tach_mec5_cct_cap4_isr(void *arg)
{
	const struct device *dev = (struct device *)arg;
	const struct tach_mec5_cct_cap_dev_cfg *const devcfg = dev->config;
	struct mec_cct_regs *regs = devcfg->regs;
	uint32_t capcnt = regs->CAP4_CNT;
	uint32_t frcnt = regs->FR_COUNT;

	tach_mec5_cct_cap_handler(dev, MEC_CCT_CAP4_ID, capcnt, frcnt);
}

static void tach_mec5_cct_cap5_isr(void *arg)
{
	const struct device *dev = (struct device *)arg;
	const struct tach_mec5_cct_cap_dev_cfg *const devcfg = dev->config;
	struct mec_cct_regs *regs = devcfg->regs;
	uint32_t capcnt = regs->CAP5_CNT;
	uint32_t frcnt = regs->FR_COUNT;

	tach_mec5_cct_cap_handler(dev, MEC_CCT_CAP5_ID, capcnt, frcnt);
}

static int tach_mec5_cct_attr_get(const struct device *dev,
				  enum sensor_channel chan,
				  enum sensor_attribute attr,
				  struct sensor_value *val)
{
	return -ENODEV;
}

static int tach_mec5_cct_attr_set(const struct device *dev,
				  enum sensor_channel chan,
				  enum sensor_attribute attr,
				  const struct sensor_value *val)
{
	return -ENODEV;
}

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
	const struct tach_mec5_cct_cap_dev_cfg *const devcfg = dev->config;
	struct mec_cct_regs *regs = devcfg->regs;
	struct tach_mec5_cct_cap_dev_data *data = dev->data;
	uint32_t capid = 0, cap_ibm = 0, cbm = 0, ev = 0;
	size_t n = 0;
	int ret = 0;

	switch ((uint32_t)chan) {
	case SENSOR_CHAN_ALL:
		cap_ibm = devcfg->cap_bm << MEC_CCT_IRQ_SRC_CAP0_POS; /* adjust to position */
		break;
	case SENSOR_CHAN_RPM: /* capture 0 */
		if (devcfg->cap_bm & BIT(MEC_CCT_CAP0_ID)) {
			capid = 0;
			cap_ibm = BIT(MEC_CCT_IRQ_SRC_CAP0_POS);
		}
		break;
	case SENSOR_CHAN_RPM1:
		if (devcfg->cap_bm & BIT(MEC_CCT_CAP1_ID)) {
			capid = 1u;
			cap_ibm = BIT(MEC_CCT_IRQ_SRC_CAP1_POS);
		}
		break;
	case SENSOR_CHAN_RPM2:
		if (devcfg->cap_bm & BIT(MEC_CCT_CAP2_ID)) {
			capid = 2u;
			cap_ibm = BIT(MEC_CCT_IRQ_SRC_CAP2_POS);
		}
		break;
	case SENSOR_CHAN_RPM3:
		if (devcfg->cap_bm & BIT(MEC_CCT_CAP3_ID)) {
			capid = 3u;
			cap_ibm = BIT(MEC_CCT_IRQ_SRC_CAP3_POS);
		}
		break;
	case SENSOR_CHAN_RPM4:
		if (devcfg->cap_bm & BIT(MEC_CCT_IRQ_SRC_CAP4_POS)) {
			capid = 4u;
			cap_ibm = BIT(MEC_CCT_IRQ_SRC_CAP4_POS);
		}
		break;
	case SENSOR_CHAN_RPM5:
		if (devcfg->cap_bm & BIT(MEC_CCT_CAP5_ID)) {
			capid = 5u;
			cap_ibm = BIT(MEC_CCT_IRQ_SRC_CAP5_POS);
		}
		break;
	default:
		return -ENOTSUP;
	}

	if (!cap_ibm) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	k_event_clear(&data->events, UINT32_MAX);

	cbm = cap_ibm >> 1;
	data->ready &= (uint16_t)~cbm;
	for (n = 0; n < ARRAY_SIZE(data->trigger_counts); n++) {
		data->trigger_counts[n] = 0;
	}

	for (n = 0; n < devcfg->num_cap_chans; n++) {
		const struct cct_cap_config *ccfg = &devcfg->capcfgs[n];

		if (cbm & BIT(ccfg->unit_id)) {
			mec_hal_cct_cap_ctrl(regs, ccfg->unit_id, ccfg->edge_detect);
		}
	}

	mec_hal_cct_enable_irq(regs, cap_ibm);

	/* TODO - Is a timeout viable?
	 * Slowest freerun count is 375 KHz full 32-bit value for approximately 11453.25 seconds
	 */
	ev = k_event_wait_all(&data->events, cbm, false, K_FOREVER);
	if (ev == 0) {
		LOG_ERR("Timeout event wait");
		ret = -ETIMEDOUT;
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

/* free run counter tick time in nano-seconds */
static const uint32_t frns[MEC_CCT_CLK_DIV_MAX] = {
	21u, /* 48M */
	42u, /* 24M */
	83u, /* 12M */
	167u, /* 6M */
	333u, /* 3M */
	667u, /* 1500K */
	1333u, /* 750K */
	2667u, /* 375K */
};

static void compute_rpm(const struct device *dev, struct sensor_value *sval, int64_t cnt)
{
	const struct tach_mec5_cct_cap_dev_cfg *const devcfg = dev->config;
	int64_t ns = 0, q = 0, f = 0;

	sval->val1 = 0; /* int32_t */
	sval->val2 = 0; /* int32_t */

	ns = cnt * frns[devcfg->fr_clkdiv];
	if (ns <= 0) {
		sval->val1 = INT32_MIN;
		return;
	}

	q = ((int64_t)(NS_PER_SEC) / ns);
	f = ((int64_t)(NS_PER_SEC) % ns);
	f  = (1000000 * f) / ns; /* fractional part in millionths */

	q *= 60;
	if (q <= INT32_MAX) {
		sval->val1 = (int32_t)q;
	} else {
		sval->val1 = INT32_MAX;
	}

	if (f <= INT32_MAX) {
		sval->val2 = (int32_t)f;
	} else {
		sval->val1 = INT32_MAX;
	}
}

/*
 * Capture unit copies freerun counter into capture register when
 * edge event detected.
 * Assumptions: input signal is from a single-pole fan.
 * Falling or rising edge: ISR saves 2 consecutive values.
 * Both edges: ISR saves four consecutive values (duty cycle may not be 50%).
 * Falling/rising edges: one revolution count = abs(count[0], count[1])
 * Both edges: one revolution count = abs(count[0], count[3])
 * revolution time = revolution_count * frns[frclkdiv] in nano-seconds
 *
 */
static int tach_mec5_cct_channel_get(const struct device *dev, enum sensor_channel chan,
				     struct sensor_value *sval)
{
	const struct tach_mec5_cct_cap_dev_cfg *const devcfg = dev->config;
	struct tach_mec5_cct_cap_dev_data *data = dev->data;
	uint16_t capid = MEC_CCT_CAP_ID_MAX;
	int ret = 0;

	switch ((uint32_t)chan) {
	case SENSOR_CHAN_RPM: /* capture 0 */
		if (devcfg->cap_bm & BIT(MEC_CCT_CAP0_ID)) {
			capid = 0;
		}
		break;
	case SENSOR_CHAN_RPM1:
		if (devcfg->cap_bm & BIT(MEC_CCT_CAP1_ID)) {
			capid = 1;
		}
		break;
	case SENSOR_CHAN_RPM2:
		if (devcfg->cap_bm & BIT(MEC_CCT_CAP2_ID)) {
			capid = 2;
		}
		break;
	case SENSOR_CHAN_RPM3:
		if (devcfg->cap_bm & BIT(MEC_CCT_CAP3_ID)) {
		        capid = 3;
		}
		break;
	case SENSOR_CHAN_RPM4:
		if (devcfg->cap_bm & BIT(MEC_CCT_CAP4_ID)) {
			capid = 4;
		}
		break;
	case SENSOR_CHAN_RPM5:
		if (devcfg->cap_bm & BIT(MEC_CCT_CAP5_ID)) {
			capid = 5;
		}
		break;
	default:
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	const struct cct_cap_config *cfg = get_cap_cfg(dev, capid);

	ret = -ENOTSUP;
	if (cfg) {
		if (!(data->ready & BIT(capid))) {
			k_mutex_unlock(&data->lock);
			return -EBUSY;
		}

		struct cct_cap_counts *caps = &data->captures[capid];

		compute_rpm(dev, sval, caps->interval);

		ret = 0;
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

/**
 * @brief Activate a sensor's trigger and set the trigger handler
 *
 * The handler will be called from a thread, so I2C or SPI operations are
 * safe.  However, the thread's stack is limited and defined by the
 * driver.  It is currently up to the caller to ensure that the handler
 * does not overflow the stack.
 *
 * The user-allocated trigger will be stored by the driver as a pointer, rather
 * than a copy, and passed back to the handler. This enables the handler to use
 * CONTAINER_OF to retrieve a context pointer when the trigger is embedded in a
 * larger struct and requires that the trigger is not allocated on the stack.
 *
 * @funcprops \supervisor
 *
 * @param dev Pointer to the sensor device
 * @param trig The trigger to activate
 * @param handler The function that should be called when the trigger
 * fires
 *
 * @return 0 if successful, negative errno code if failure.
 */
static int tach_mec5_cct_trigger_set(const struct device *dev,
				    const struct sensor_trigger *trig,
				    sensor_trigger_handler_t handler)
{
	struct tach_mec5_cct_cap_dev_data *data = dev->data;

	if (trig->type != SENSOR_TRIG_DATA_READY) { /* SENSOR_TRIG_THRESHOLD */
		return -ENOTSUP;
	}

	if (!handler) {
		return -EINVAL;
	}

	data->thandler = handler;
	data->trigger = trig;

	return 0;
}

/* New Read and Decode API
 * required: sensor_submit_t
 *           should implement sensor_submit_t using RTIO
 *           may implement using a work-queue if RTIO can't be done
 * required: sensor_decode_api as pure stateless functions
 * required: sensor_get_decoder_t returning sensor_decoder_api for that device type
 */

static int tach_mec5_cct_cap_dev_init(const struct device *dev)
{
	const struct tach_mec5_cct_cap_dev_cfg *const devcfg = dev->config;
	struct tach_mec5_cct_cap_dev_data *data = dev->data;
	struct mec_cct_regs *const regs = devcfg->regs;
	struct mec_cct_config cct_cfg = {0};
	int ret = 0;

	data->dev = dev;
	data->oneshot = devcfg->cap_bm;

	mec_hal_pcr_blk_reset(MEC_PCR_CCT0);
	data->ctrl = regs->TCTRL;

	regs->TCTRL |= BIT(MEC_CCT_TCTRL_ACTV_Pos);

	ret = pinctrl_apply_state(devcfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("MEC5 TACH-CCT PINCTRL init failed (%d)", ret);
		return ret;
	}

	for (uint8_t n = 0; n < devcfg->num_cap_chans; n++) {
		const struct cct_cap_config *capcfg = &devcfg->capcfgs[n];

		cct_cfg.cap_cfg[n] = MEC_CCT_CAP_INFO(capcfg->unit_id, capcfg->edge_detect,
						      capcfg->filter_clk, capcfg->mux);
	}

	cct_cfg.flags = MEC_CCT_CFG_FR_EN;
	cct_cfg.fr_clk_div = devcfg->fr_clkdiv;

	ret = mec_hal_cct_init(regs, &cct_cfg);
	if (ret != MEC_RET_OK) {
		LOG_ERR("MEC5 TACH-CCT init error (%d)", ret);
		return -EIO;
	}

	if (devcfg->irq_config) {
		devcfg->irq_config();
	}

	return 0;
}

static const struct sensor_driver_api tach_mec5_cct_cap_driver_api = {
	.attr_set = tach_mec5_cct_attr_set,
	.attr_get = tach_mec5_cct_attr_get,
	.sample_fetch = tach_mec5_cct_sample_fetch,
	.channel_get = tach_mec5_cct_channel_get,
	.trigger_set = tach_mec5_cct_trigger_set,
};

#define TACH_MEC5_CCT_IRQ_CFG(i)                                                                   \
	static void tach_mec5_cct_irq_cfg_##i(void)                                                \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(i), DT_INST_IRQ(i, priority), tach_mec5_cct_fr_isr,       \
			    DEVICE_DT_INST_GET(i), 0);                                             \
		irq_enable(DT_INST_IRQN(i));                                                       \
		IRQ_CONNECT(DT_INST_IRQN_BY_IDX(i, 1), DT_INST_IRQ_BY_IDX(i, 1, priority),         \
			    tach_mec5_cct_cap0_isr, DEVICE_DT_INST_GET(i), 0);                     \
		irq_enable(DT_INST_IRQN_BY_IDX(i, 1));                                             \
		IRQ_CONNECT(DT_INST_IRQN_BY_IDX(i, 2), DT_INST_IRQ_BY_IDX(i, 2, priority),         \
			    tach_mec5_cct_cap1_isr, DEVICE_DT_INST_GET(i), 0);                     \
		irq_enable(DT_INST_IRQN_BY_IDX(i, 2));                                             \
		IRQ_CONNECT(DT_INST_IRQN_BY_IDX(i, 3), DT_INST_IRQ_BY_IDX(i, 3, priority),         \
			    tach_mec5_cct_cap2_isr, DEVICE_DT_INST_GET(i), 0);                     \
		irq_enable(DT_INST_IRQN_BY_IDX(i, 3));                                             \
		IRQ_CONNECT(DT_INST_IRQN_BY_IDX(i, 4), DT_INST_IRQ_BY_IDX(i, 4, priority),         \
			    tach_mec5_cct_cap3_isr, DEVICE_DT_INST_GET(i), 0);                     \
		irq_enable(DT_INST_IRQN_BY_IDX(i, 4));                                             \
		IRQ_CONNECT(DT_INST_IRQN_BY_IDX(i, 5), DT_INST_IRQ_BY_IDX(i, 5, priority),         \
			    tach_mec5_cct_cap4_isr, DEVICE_DT_INST_GET(i), 0);                     \
		irq_enable(DT_INST_IRQN_BY_IDX(i, 5));                                             \
		IRQ_CONNECT(DT_INST_IRQN_BY_IDX(i, 6), DT_INST_IRQ_BY_IDX(i, 6, priority),         \
			    tach_mec5_cct_cap5_isr, DEVICE_DT_INST_GET(i), 0);                     \
		irq_enable(DT_INST_IRQN_BY_IDX(i, 6));                                             \
	}

#define MEC5_CCT_CAP_BMV(node_id) BIT(DT_REG_ADDR(node_id))

#define MEC5_CCT_CAP_BM(i) DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(i, MEC5_CCT_CAP_BMV, (+))

#define MEC5_CCT_CAP_FILT_CLK(i) \
	(((uint8_t)DT_ENUM_IDX(node_id, filter_clk) & 0x7u) |\
	 (DT_PROP_OR(i, filter_bypass, 0) << 7))

#define MEC5_CCT_CAP_CFG(node_id)                                                                  \
	[DT_REG_ADDR(node_id)] = {                                                                 \
		.unit_id = (uint8_t)DT_REG_ADDR(node_id),                                          \
		.edge_detect = (uint8_t)DT_ENUM_IDX_OR(node_id, edge_detect, 3),                   \
		.filter_clk = (uint8_t)DT_ENUM_IDX(node_id, filter_clk),                           \
		.mux = (uint8_t)DT_PROP(node_id, input_mux),                                       \
	},

#define TACH_MEC5_CCT_DEV_CFG(i)                                                   \
	static const struct cct_cap_config capcfgs_##i[] = {                       \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(i, MEC5_CCT_CAP_CFG)             \
	};                                                                         \
	static const struct tach_mec5_cct_cap_dev_cfg tach_mec5_cct_devcfg_##i = { \
		.regs = (struct mec_cct_regs *)DT_INST_REG_ADDR(i),                \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i),                         \
		.irq_config = tach_mec5_cct_irq_cfg_##i,                           \
		.fr_clkdiv = (uint8_t)DT_INST_ENUM_IDX_OR(i, freerun_clk, 0),      \
		.capcfgs = capcfgs_##i,                                            \
		.num_cap_chans = ARRAY_SIZE(capcfgs_##i),                          \
		.cap_bm = MEC5_CCT_CAP_BM(i),                                      \
	}

#define TACH_MEC5_CCT_DEVICE(id)                                               \
	static struct tach_mec5_cct_cap_dev_data tach_mec5_cct_data_##id = {   \
		.lock = Z_MUTEX_INITIALIZER(tach_mec5_cct_data_##id.lock),     \
		.events = Z_EVENT_INITIALIZER(tach_mec5_cct_data_##id.events), \
	};                                                                     \
	PINCTRL_DT_INST_DEFINE(id);                                            \
	TACH_MEC5_CCT_IRQ_CFG(id)                                              \
	TACH_MEC5_CCT_DEV_CFG(id);                                             \
	PM_DEVICE_DT_INST_DEFINE(id, tach_mec5_cct_pm_action);                 \
	SENSOR_DEVICE_DT_INST_DEFINE(id,                                       \
			    tach_mec5_cct_cap_dev_init,                        \
			    PM_DEVICE_DT_INST_GET(id),                         \
			    &tach_mec5_cct_data_##id,                          \
			    &tach_mec5_cct_devcfg_##id,                        \
			    POST_KERNEL,                                       \
			    CONFIG_SENSOR_INIT_PRIORITY,                       \
			    &tach_mec5_cct_cap_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TACH_MEC5_CCT_DEVICE)
