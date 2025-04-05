/*
 * Copyright (c) 2025 Microchip Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_i2c_nl

#include <zephyr/kernel.h>
#include <soc.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/devicetree/dma.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/sys_io.h>
LOG_MODULE_REGISTER(i2c_nl_mchp, CONFIG_I2C_LOG_LEVEL);

#include <app/drivers/mchp_mec5_i2c_nl.h>

/* out of tree build requires this path for headers in the driver source directory */
#include <i2c/i2c-priv.h>

/* MEC5 HAL */
#include <device_mec5.h>
#include <mec_dmac_api.h>
#include <mec_ecia_api.h>
#include <mec_i2c_api.h>

#define I2C_NL_DEBUG_ISR
#define I2C_NL_DEBUG_ISR_ENTRIES 64
#define I2C_NL_DEBUG_STATE
#define I2C_NL_DEBUG_STATE_ENTRIES 256

#define I2C_MEC5_NL_USE_INTERRUPTS

#define I2C_MEC5_NL_RESET_WAIT_US 20

/* SMBus specification default timeout in milliseconds */
#define I2C_MEC5_NL_SMBUS_TMOUT_MAX_MS 35

/* I2C timeout is  10 ms (WAIT_INTERVAL * WAIT_COUNT) */
#define I2C_MEC5_NL_WAIT_INTERVAL_US 50
#define I2C_MEC5_NL_WAIT_COUNT       200
#define I2C_MEC5_NL_STOP_WAIT_COUNT  500
#define I2C_MEC5_NL_PIN_CFG_WAIT     50

#define I2C_MEC5_NL_MAX_XFR_LEN       0xfff8u
/* padding lengths: I2C-NL hardware requires target
 * address in the memory buffer.
 * start_target_address | data | rpt_start_target_address | data
 */
#define I2C_MEC5_NL_CM_TX_BUF_PAD_LEN 8
#define I2C_MEC5_NL_TM_RX_BUF_PAD_LEN 8

#define I2C_MEC5_NL_MISC_CFG_PORT_POS 0
#define I2C_MEC5_NL_MISC_CFG_PORT_MSK 0xfu

#define I2C_MEC5_NL_MAX_DMA_CFGS 3

enum i2c_mec5_nl_state {
	I2C_NL_STATE_CLOSED = 0,
	I2C_NL_STATE_CM,
	I2C_NL_STATE_TM,
	I2C_NL_STATE_MAX,
};

enum i2c_mec5_nl_direction {
	I2C_NL_DIR_NONE = 0,
	I2C_NL_DIR_WR,
	I2C_NL_DIR_RD,
};

enum i2c_mec5_nl_error {
	I2C_NL_ERR_NONE = 0,
	I2C_NL_ERR_BUS,
	I2C_NL_ERR_LOST_ARB,
	I2C_NL_ERR_NACK_FROM_TARGET,
	I2C_NL_ERR_TIMEOUT,
};

enum i2c_mec5_nl_kevents {
	I2C_NL_KEV_IDLE_POS = 0,
	I2C_NL_KEV_BERR_POS,
	I2C_NL_KEV_LAB_ERR_POS,
	I2C_NL_KEV_CM_NAK_POS,
	I2C_NL_KEV_CM_PAUSE_POS,
	I2C_NL_KEV_CM_DONE_POS,
	I2C_NL_KEV_DMA_CM_DONE_POS,
	I2C_NL_KEV_DMA_CM_ERR_POS,
	I2C_NL_KEV_TM_AAT_POS,
	I2C_NL_KEV_TM_DONE_POS,
	I2C_NL_KEV_TM_PAUSE_POS,
	I2C_NL_KEV_TM_RPT_WR_POS,
	I2C_NL_KEV_TM_RPT_RD_POS,
	I2C_NL_KEV_TM_TPROT_POS,
	I2C_NL_KEV_TM_NAKR_POS,
	I2C_NL_KEV_DMA_TM_DONE_POS,
	I2C_NL_KEV_DMA_TM_ERR_POS,
};

enum i2c_mec5_nl_xfr_flags {
	I2C_NL_XFR_FLAG_ASYNC_POS = 0,
};

#define I2C_NL_EVENTS_ERRORS                                                                       \
	(BIT(I2C_NL_KEV_BERR_POS) | BIT(I2C_NL_KEV_LAB_ERR_POS) | BIT(I2C_NL_KEV_CM_NAK_POS))

#define I2C_NL_WAIT_CM_EVENTS_MSK                                                                  \
	(BIT(I2C_NL_KEV_IDLE_POS) | BIT(I2C_NL_KEV_CM_NAK_POS) | BIT(I2C_NL_KEV_CM_PAUSE_POS) |    \
	 BIT(I2C_NL_KEV_CM_DONE_POS) | BIT(I2C_NL_KEV_DMA_CM_DONE_POS))

#define I2C_NL_WAIT_CM_AND_DMA_DONE BIT(I2C_NL_KEV_CM_DONE_POS) | BIT(I2C_NL_KEV_DMA_CM_DONE_POS)

#define I2C_NL_ERRORS                                                                              \
	(BIT(I2C_NL_KEV_DMA_CM_ERR_POS) | BIT(I2C_NL_KEV_DMA_TM_ERR_POS) |                         \
	 BIT(I2C_NL_KEV_BERR_POS) | BIT(I2C_NL_KEV_LAB_ERR_POS) | BIT(I2C_NL_KEV_CM_NAK_POS))

#define I2C_NL_ALL_TM_EVENTS                                                                       \
	(BIT(I2C_NL_KEV_TM_AAT_POS) | BIT(I2C_NL_KEV_TM_DONE_POS) | BIT(I2C_NL_KEV_TM_PAUSE_POS) | \
	 BIT(I2C_NL_KEV_TM_RPT_WR_POS) | BIT(I2C_NL_KEV_TM_RPT_RD_POS) |                           \
	 BIT(I2C_NL_KEV_TM_TPROT_POS) | BIT(I2C_NL_KEV_TM_NAKR_POS) |                              \
	 BIT(I2C_NL_KEV_DMA_TM_DONE_POS) | BIT(I2C_NL_KEV_DMA_TM_ERR_POS))

struct i2c_mec5_nl_config {
	struct mec_i2c_smb_regs *i2c_regs;
	uint32_t init_pin_wait_us;
	uint32_t cfg_pin_wait_us;
	struct gpio_dt_spec sda_gpio;
	struct gpio_dt_spec scl_gpio;
	void (*irq_config_func)(void);
	const struct pinctrl_dev_config *pcfg;
	uint8_t cm_dma_chan;
	uint8_t cm_dma_trigsrc;
	uint16_t cm_tx_buf_max_sz; /* value does not include I2C_MEC5_NL_CM_TX_BUF_PAD_LEN */
#ifdef CONFIG_I2C_TARGET
	uint16_t tm_rx_buf_sz;
	uint8_t *tm_rx_buf;
	uint8_t tm_dma_chan;
	uint8_t tm_dma_trigsrc;
#endif
};

#ifdef I2C_NL_DEBUG_ISR
struct i2c_mec5_dbg_isr_data {
	uint32_t isr_idx;
	uint32_t isr_cnt;
	uint32_t cm_dma_isr_cnt;
	uint32_t tm_dma_isr_cnt;
	uint32_t status;
	uint32_t config;
	uint32_t cm_cmd;
	uint32_t tm_cmd;
	uint32_t extlen;
	uint32_t ev;
};
#endif

struct i2c_nl_blk {
	struct i2c_nl_blk *next;
	uint32_t xflags;
	struct mec_dma_cfg4 dcfg;
};

struct i2c_mec5_nl_data {
	const struct i2c_mec5_nl_config *devcfg;
	struct mec_i2c_smb_ctx ctx;
	struct k_sem lock;
	struct k_event events;
	uint32_t ev;
	uint32_t clock_freq_hz;
	uint32_t i2c_status;
	uint32_t xfr_tmout;
	uint8_t *xfrbuf;
	uint32_t xfrlen;
	struct i2c_msg *msgs;
	uint16_t total_rx_len;
	uint16_t total_tx_len;
	uint8_t num_msgs;
	uint8_t msgidx;
	uint8_t i2c_addr;
	uint8_t state;
	uint8_t dir;
	uint8_t misc_cfg; /* b[3:0]=port, b[6:4]=rsvd, b[7]=0(CM), 1(TM) */
	uint8_t xfr_flags;
	uint8_t didx;
	uint32_t cm_cmd;
	struct i2c_nl_blk *pblk;
	struct i2c_nl_blk xblks[I2C_MEC5_NL_MAX_DMA_CFGS];
#ifdef CONFIG_I2C_TARGET
	struct i2c_target_config *target1_cfg;
	struct i2c_target_config *target2_cfg;
	struct i2c_target_config *curr_target;
	struct mec_dma_cfg4 tm_dma_cfg;
	uint16_t tm_rd_cnt;
	uint16_t tm_rd_cnt_curr;
	uint32_t tm_cmd_addr; /* DEBUG use */
	uint8_t *tm_tx_buf;
	uint32_t tm_tx_buf_sz;
	uint32_t tm_tx_buf_xfr_sz;
#endif
#ifdef I2C_NL_DEBUG_ISR
	volatile uint32_t dbg_isr_cnt;
	volatile uint32_t dbg_isr_idx;
	volatile uint32_t dbg_cm_dma_isr_cnt;
	volatile uint32_t dbg_tm_dma_isr_cnt;
	struct i2c_mec5_dbg_isr_data dbg_isr_data[I2C_NL_DEBUG_ISR_ENTRIES];
#endif
#ifdef I2C_NL_DEBUG_STATE
	atomic_t dbg_state_idx;
	uint8_t dbg_states[I2C_NL_DEBUG_STATE_ENTRIES];
#endif
};

#ifdef I2C_NL_DEBUG_ISR
static inline void i2c_mec5_nl_dbg_isr_init(struct i2c_mec5_nl_data *data)
{
	data->dbg_isr_cnt = 0;
	data->dbg_cm_dma_isr_cnt = 0;
	data->dbg_tm_dma_isr_cnt = 0;
	data->dbg_isr_idx = 0;
	memset(data->dbg_isr_data, 0, sizeof(data->dbg_isr_data));
}

static inline void i2c_mec5_nl_dbg_isr_cnt_update(struct i2c_mec5_nl_data *data)
{
	data->dbg_isr_cnt++;
}

static inline void i2c_mec5_nl_dbg_cm_dma_isr_cnt_update(struct i2c_mec5_nl_data *data)
{
	data->dbg_cm_dma_isr_cnt++;
}

static inline void i2c_mec5_nl_dbg_tm_dma_isr_cnt_update(struct i2c_mec5_nl_data *data)
{
	data->dbg_tm_dma_isr_cnt++;
}

static inline void i2c_mec5_nl_dbg_isr_data_update(struct i2c_mec5_nl_data *data)
{
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	struct mec_i2c_smb_regs *regs = hwctx->base;
	uint32_t idx = data->dbg_isr_idx;

	data->dbg_isr_idx++;

	if (idx < I2C_NL_DEBUG_ISR_ENTRIES) {
		data->dbg_isr_data[idx].isr_idx = idx;
		data->dbg_isr_data[idx].isr_cnt = data->dbg_isr_cnt;
		data->dbg_isr_data[idx].cm_dma_isr_cnt = data->dbg_cm_dma_isr_cnt;
		data->dbg_isr_data[idx].tm_dma_isr_cnt = data->dbg_tm_dma_isr_cnt;
		data->dbg_isr_data[idx].status = data->i2c_status;
		data->dbg_isr_data[idx].config = regs->CONFIG;
		data->dbg_isr_data[idx].cm_cmd = regs->CM_CMD;
		data->dbg_isr_data[idx].tm_cmd = regs->TM_CMD | (regs->SHAD_ADDR << 24);
		data->dbg_isr_data[idx].extlen = regs->EXTLEN;
		data->dbg_isr_data[idx].ev = data->ev;
	}
}

#define I2C_NL_DEBUG_ISR_INIT(d) i2c_mec5_nl_dbg_isr_init(d)
#define I2C_NL_DEBUG_ISR_COUNT_UPDATE(d) i2c_mec5_nl_dbg_isr_cnt_update(d)
#define I2C_NL_DEBUG_CM_DMA_ISR_COUNT_UPDATE(d) i2c_mec5_nl_dbg_cm_dma_isr_cnt_update(d)
#define I2C_NL_DEBUG_TM_DMA_ISR_COUNT_UPDATE(d) i2c_mec5_nl_dbg_tm_dma_isr_cnt_update(d)
#define I2C_NL_DEBUG_ISR_DATA_UPDATE(d) i2c_mec5_nl_dbg_isr_data_update(d)
#else
#define I2C_NL_DEBUG_ISR_INIT(d)
#define I2C_NL_DEBUG_ISR_COUNT_UPDATE(d)
#define I2C_NL_DEBUG_CM_DMA_ISR_COUNT_UPDATE(d)
#define I2C_NL_DEBUG_TM_DMA_ISR_COUNT_UPDATE(d)
#define I2C_NL_DEBUG_ISR_DATA_UPDATE(d)
#endif /* #ifdef I2C_NL_DEBUG_ISR */

#ifdef I2C_NL_DEBUG_STATE
static void i2c_mec5_nl_dbg_state_init(struct i2c_mec5_nl_data *data)
{
	data->dbg_state_idx = ATOMIC_INIT(0);
	memset(data->dbg_states, 0, sizeof(data->dbg_states));
}

/* note: atomic_inc returns previous value before incrementing */
static void i2c_mec5_nl_dbg_state_update(struct i2c_mec5_nl_data *data, uint8_t state)
{
	atomic_val_t idx = atomic_inc(&data->dbg_state_idx);

	if (idx < I2C_NL_DEBUG_STATE_ENTRIES) {
		data->dbg_states[idx] = state;
	}
}

#define I2C_NL_DEBUG_STATE_INIT(d) i2c_mec5_nl_dbg_state_init((d))
#define I2C_NL_DEBUG_STATE_UPDATE(d, state) i2c_mec5_nl_dbg_state_update((d), (state))
#else
#define I2C_NL_DEBUG_STATE_INIT(d)
#define I2C_NL_DEBUG_STATE_UPDATE(d, state)

#endif /* I2C_NL_DEBUG_STATE */

/* DEBUG */
static volatile uint32_t halt_in_isr;

static int wait_bus_free(const struct device *dev, uint32_t nwait)
{
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t count = nwait;
	uint32_t sts = 0;

	while (count--) {
		sts = mec_hal_i2c_smb_status(hwctx, 0);
		data->i2c_status = sts;
		if (sts & BIT(MEC_I2C_STS_LL_NBB_POS)) {
			break; /* bus is free */
		}
		k_busy_wait(I2C_MEC5_NL_WAIT_INTERVAL_US);
	}

	/* check for bus error, lost arbitration or external stop */
	if ((sts & 0xffu) == (BIT(MEC_I2C_STS_LL_NBB_POS) | BIT(MEC_I2C_STS_LL_NIPEND_POS))) {
		return 0;
	}

	if (sts & BIT(MEC_I2C_STS_LL_BER_POS)) {
		return I2C_NL_ERR_BUS;
	}

	if (sts & BIT(MEC_I2C_STS_LL_LRB_AD0_POS)) {
		return I2C_NL_ERR_LOST_ARB;
	}

	return I2C_NL_ERR_TIMEOUT;
}

static int i2c_mec5_nl_reset_config(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	struct mec_i2c_smb_cfg mcfg = {0};
	int ret = 0;

	I2C_NL_DEBUG_ISR_INIT(data);
	I2C_NL_DEBUG_STATE_INIT(data);

	hwctx->base = devcfg->i2c_regs;
	hwctx->i2c_ctrl_cached = 0;
	data->i2c_status = 0;

	if (data->clock_freq_hz >= MHZ(1)) {
		mcfg.std_freq = MEC_I2C_STD_FREQ_1M;
	} else if (data->clock_freq_hz >= KHZ(400)) {
		mcfg.std_freq = MEC_I2C_STD_FREQ_400K;
	} else {
		mcfg.std_freq = MEC_I2C_STD_FREQ_100K;
	}

	mcfg.port =
		((data->misc_cfg >> I2C_MEC5_NL_MISC_CFG_PORT_POS) & I2C_MEC5_NL_MISC_CFG_PORT_MSK);

#ifdef CONFIG_I2C_TARGET
	if (data->target1_cfg || data->target2_cfg) {
		mcfg.cfg_flags |= MEC_I2C_SMB_CFG_PRESERVE_TARGET_ADDRS;
	}
#endif

	ret = mec_hal_i2c_smb_init(hwctx, &mcfg, NULL);
	if (ret != MEC_RET_OK) {
		return -EIO;
	}

	return 0;
}

/* MEC5 I2C controller support 7-bit addressing only.
 * Format 7-bit address for as it appears on the bus as an 8-bit
 * value with R/W bit at bit[0], 0(write), 1(read).
 */
static inline uint8_t fmt_addr(uint16_t addr, enum i2c_mec5_nl_direction dir)
{
	uint8_t fmt_addr = (uint8_t)((addr & 0x7fu) << 1);

	if (dir == I2C_NL_DIR_RD) {
		fmt_addr |= BIT(0);
	}

	return fmt_addr;
}

/* Compute timeout based on message group write and read lengths
 * I2C-NL driver can handle messages of up to 0xFFFC bytes.
 * @ 100 KHz 6000 ms, @ 400 KHz 1500 ms, 1 MHz 600 ms
 * Over estimate 10 clocks / byte and add 10 ms to total.
 * if computed timeout < SMBus timeout of 35 ms then use 35 ms.
 */
static void msgs_compute_timeout(const struct device *dev)
{
	struct i2c_mec5_nl_data *const data = dev->data;
	uint32_t i2c_freq = data->clock_freq_hz;
	uint32_t i2c_clks = 0;
	uint32_t tmout = 0;

	i2c_clks = data->total_rx_len + data->total_tx_len;
	i2c_clks *= 10000u;
	tmout = i2c_clks / i2c_freq;
	tmout += 10u; /* paranoid: add 10 ms */
	if (tmout < I2C_MEC5_NL_SMBUS_TMOUT_MAX_MS) {
		tmout = I2C_MEC5_NL_SMBUS_TMOUT_MAX_MS;
	}

	data->xfr_tmout = tmout;
}

#if 0 /* not used for now */
static int i2c_mec5_nl_do_stop(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t ev, evw;

	/* Can we trust GIRQ status has been cleared before we re-enable? */
	if (mec_hal_i2c_smb_is_bus_owned(hwctx)) {
		data->dir = I2C_NL_DIR_NONE;
		mec_hal_i2c_smb_stop_gen(hwctx);
		mec_hal_i2c_smb_girq_status_clr(hwctx);
		mec_hal_i2c_smb_idle_intr_enable(hwctx, 1);
		mec_hal_i2c_smb_girq_ctrl(hwctx, MEC_I2C_SMB_GIRQ_EN);

		evw = (BIT(I2C_NL_KEV_IDLE_POS) | BIT(I2C_NL_KEV_BERR_POS) |
		       BIT(I2C_NL_KEV_LAB_ERR_POS));
		ev = k_event_wait(&data->events, evw, false, K_MSEC(100));
		if (!ev) {
			LOG_ERR("Gen STOP timeout");
		}

		mec_hal_dma_chan_stop(devcfg->cm_dma_chan);
		mec_hal_i2c_nl_cmd_clear(hwctx, MEC_I2C_NL_CM_SEL);
	}

	data->state = I2C_NL_STATE_CLOSED;

	return 0;
}
#endif

/* HW and driver limit nrx and ntx to 0xfff8u */
static void i2c_mec5_nl_xfr_data_init(const struct device *dev, struct i2c_msg *msgs,
				      uint8_t num_msgs, uint16_t addr)
{
	struct i2c_mec5_nl_data *const data = dev->data;

	data->total_rx_len = 0;
	data->total_tx_len = 0;
	data->xfrlen = 0;
	data->pblk = NULL;
	data->msgs = msgs;
	data->num_msgs = num_msgs;
	data->i2c_addr = (uint8_t)(addr & 0x7fu);
	data->msgidx = 0;
	data->didx = 0;
	data->xfr_flags = 0;
	data->state = I2C_NL_STATE_CLOSED;

	memset(data->xblks, 0, sizeof(data->xblks));

	/* one message: read or write */
	if (msgs[0].flags & I2C_MSG_READ) {
		data->total_rx_len = (uint16_t)(msgs[0].len & UINT16_MAX);
	} else {
		data->total_tx_len = (uint16_t)(msgs[0].len & UINT16_MAX);
	}

	/* if two messages first is write second is read */
	if (num_msgs == 2) {
		data->total_rx_len = (uint16_t)(msgs[1].len & UINT16_MAX);
	}
}

/* Check if HW and driver supports messages.
 * Support messages 7-bit I2C addressing only.
 * Maximum number of messages is 2.
 * One I2C write or one I2C read or I2C combined write-read.
 * Last message must have I2C_MSG_STOP flag.
 * Write message maximum length is HW limit I2C_MEC5_NL_MAX_XFR_LEN.
 * Read message maximum length is HW limit (I2C_MEC5_NL_MAX_XFR_LEN).
 */
static int req_is_supported(struct i2c_msg *msgs, uint8_t num_msgs)
{
	if (!msgs || !num_msgs) {
		LOG_ERR("NULL msgs or 0 number");
		return false;
	}

	for (uint8_t n = 0; n < num_msgs; n++) {
		struct i2c_msg *m = &msgs[n];

		if ((m->buf == NULL) || (m->len == 0)) {
			LOG_ERR("I2C msg[%u] NULL buffer and/or zero length", n);
			return false;
		}

		if (m->flags & I2C_MSG_ADDR_10_BITS) {
			LOG_ERR("I2C msg[%u] has 10-bit address flag not supported by HW", n);
			return false;
		}

		if (m->len > I2C_MEC5_NL_MAX_XFR_LEN) {
			LOG_ERR("I2C msg[%u].len = %u exceeds HW limit %u",
				n, m->len, I2C_MEC5_NL_MAX_XFR_LEN);
			return false;
		}
	}

	return true;
}

/* data->state = I2C_NL_STATE_CLOSED, I2C_NL_STATE_CM, or I2C_NL_STATE_TM */

/* I2C Read using network layer HW and DMA.
 * Protocol: (RPT-)START rdAddr [ACK] [rdData1] [ACK] ... [rdDataN] [NAK] STOP
 * We can enter this routine with HW/Bus in various states.
 * Closed where bus is idle. Requires START and transmit read address
 * TX: Requires RPT-START and transmit read address
 * RX: No (RPT-)START required unless message has I2C_MSG_RESTART flag.
 * NOTE 1: HW may not let us continue a read with/without (RPT-)START
 *         We have observed HW releasing I2C lines when HW rdCnt reaches
 *         zero even if HW STOP flag not set.
 * NOTE 2: HW has control bits for START and RPT-START.
 *
 * Experiments show I2C-NL HW FSM does not support multiple reads.
 * Once the HW read count reaches 0 the HW FSM will start NAK'ing
 * and its read-ahead logic will keep generating clocks and we see
 * on the bus 0xFF. Not if the I2C-NL is driving SDA or the target
 * I2C FRAM device.
 * Try changing this routine to generate START or RPT-START.
 *
 * Then we must enforce STOP in every read.
 */
static int do_i2c_rd(const struct device *dev, struct i2c_msg *m)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct mec_i2c_smb_regs *i2c_regs = devcfg->i2c_regs;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	struct mec_dma_cfg4 dcfg = {0};
#ifdef I2C_MEC5_NL_USE_INTERRUPTS
	k_timeout_t evtmout = K_MSEC(1000);
	uint32_t cm_flags = 0;
#else
	uint32_t cm_flags = 0, i2c_status = 0, dma_status = 0;
#endif
	int ret = 0;
	uint16_t ntx = 0, nrx = 0;
	uint8_t mflags = 0;

	/* ensure we issue I2C STOP after every read otherwise this I2C-NL will not be happy */
#ifdef I2C_MEC5_NL_USE_INTERRUPTS
	cm_flags = MEC_I2C_NL_FLAG_STOP | MEC_I2C_NL_FLAG_CM_DONE_IEN;
#else
	cm_flags = MEC_I2C_NL_FLAG_STOP;
#endif
	mflags = m->flags | I2C_MSG_STOP;
	nrx = m->len;

	if (data->state == I2C_NL_STATE_CLOSED) {
		cm_flags |= MEC_I2C_NL_FLAG_START | MEC_I2C_NL_FLAG_FLUSH_BUF;
	} else if (data->dir == I2C_NL_DIR_WR) {
		/* RPT-START only for an open transaction where previous section was write */
		cm_flags |= MEC_I2C_NL_FLAG_RPT_START;
	}

	data->state = I2C_NL_STATE_CM;
	data->dir = I2C_NL_DIR_RD;

	ntx = 1u;
	data->xfrbuf[0] = fmt_addr(data->i2c_addr, I2C_NL_DIR_RD);

	dcfg.mem_addr = (uintptr_t)data->xfrbuf;
	dcfg.len = 1u;
	dcfg.unitsz = MEC_DMAC_UNIT_SIZE_1;
	dcfg.dir = MEC_DMAC_DIR_MEM_TO_DEV;
	dcfg.hwfc_dev = devcfg->cm_dma_trigsrc;
	dcfg.flags = MEC_DMA_CFG4_FLAG_INCR_MEM_ADDR;

	ret = mec_hal_dma_chan_cfg4_by_id(devcfg->cm_dma_chan, &dcfg);
	ret |=mec_hal_dma_chan_start(devcfg->cm_dma_chan);
	ret |= mec_hal_i2c_nl_cm_start(i2c_regs, ntx, nrx, cm_flags, &data->cm_cmd);
	if (ret) {
		return -EIO;
	}

#ifdef I2C_MEC5_NL_USE_INTERRUPTS
	data->ev = k_event_wait(&data->events, I2C_NL_WAIT_CM_EVENTS_MSK, false, evtmout);
	if (data->ev == 0) {
		return -ETIMEDOUT;
	} else if (data->ev & I2C_NL_ERRORS) {
		return -EIO;
	}
	k_event_clear(&data->events, I2C_NL_WAIT_CM_EVENTS_MSK);
	if (!(data->ev & BIT(I2C_NL_KEV_CM_PAUSE_POS))) {
		LOG_ERR("Expected CM Pause HW state: ev = 0x%0x", data->ev);
		return -EIO;
	}
#else
	i2c_status = mec_hal_i2c_smb_status(hwctx, 0);
	while (!(i2c_status & BIT(MEC_I2C_STS_CM_DONE_POS))) {
		i2c_status = mec_hal_i2c_smb_status(hwctx, 0);
	}

	if (i2c_status & BIT(MEC_I2C_STS_CM_TX_NACK_POS)) {
		return -EIO;
	}
#endif
	/* HW should be in pause state. FW switches DMA direction to read
	 * and unpauses I2C-NL by setting CM_CMD.PROC=1
	 */
	dcfg.mem_addr = (uintptr_t)m->buf;
	dcfg.len = m->len;
	dcfg.unitsz = MEC_DMAC_UNIT_SIZE_1;
	dcfg.dir = MEC_DMAC_DIR_DEV_TO_MEM;
	dcfg.hwfc_dev = devcfg->cm_dma_trigsrc;
	dcfg.flags = MEC_DMA_CFG4_FLAG_INCR_MEM_ADDR | MEC_DMA_CFG4_FLAG_DONE_IEN;

	/* DEBUG */
	halt_in_isr = 1;

	ret = mec_hal_dma_chan_cfg4_by_id(devcfg->cm_dma_chan, &dcfg);
	ret |=mec_hal_dma_chan_start(devcfg->cm_dma_chan);
	ret |= mec_hal_i2c_nl_cm_proceed(hwctx);
	if (ret) {
		return -EIO;
	}

	/* Read: CM_DONE means I2C-NL has finished clocking in last data
	 * and performed ackowlegement (9th I2C clock). I2C-NL then moves the byte
	 * to its CM_RXB buffer register and triggers DMA to read the byte and store
	 * in memory.
	 * When we run using debugger breakpoint after i2c_transfer the data pattern matches.
	 * Is the EC running at 96MHz so much faster than I2C at 400KHz and DMA at 48MHz that
	 * we return from i2c_transfer and do memory compare before DMA has transferred the last
	 * byte to memory. I know that DMA does read-ahead but in this case it issues a STOP.
	 * I don't expect read-ahead to affect the timing in this case.
	 * To prove this add delay.
	 */
#ifdef I2C_MEC5_NL_USE_INTERRUPTS
	data->ev = k_event_wait_all(&data->events, I2C_NL_WAIT_CM_AND_DMA_DONE, false, evtmout);
	if (!data->ev) {
		LOG_ERR("Read timeout: events=0x%0x", data->events.events);
		return -ETIMEDOUT;
	}
	k_event_clear(&data->events, I2C_NL_WAIT_CM_AND_DMA_DONE);
#else
	i2c_status = mec_hal_i2c_smb_status(hwctx, 0);
	while (!(i2c_status & BIT(MEC_I2C_STS_CM_DONE_POS))) {
		i2c_status = mec_hal_i2c_smb_status(hwctx, 0);
	}

	/* I2C-NL HW sets CM_DONE when it decrements its read count to 0.
	 * We are not sure when this is done, before I2C-NL does (n)ACK on
	 * the 9th I2C clock? Experiments have shown I2C-NL sets CM_DONE before
	 * the DMA channel has finished reading the last byte of data from I2C-NL.CM_RXB
	 * register and writing it to memory. We need to spin on the DMA channel waiting
	 * for it to terminate before returning from the function.
	 * Tested and works.
	 */
	do {
		dma_status = 0;
		mec_hal_dma_chan_intr_status(devcfg->cm_dma_chan, &dma_status);
	} while (!dma_status);

	if (i2c_status & BIT(MEC_I2C_STS_CM_TX_NACK_POS)) {
		ret = -EIO;
	}
#endif

	if (ret || (mflags & I2C_MSG_STOP)) {
		data->state = I2C_NL_STATE_CLOSED;
	}

	return ret;
}

static int do_i2c_wr(const struct device *dev, struct i2c_msg *m)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct mec_i2c_smb_regs *i2c_regs = devcfg->i2c_regs;
	struct i2c_mec5_nl_data *const data = dev->data;
#ifdef I2C_MEC5_NL_USE_INTERRUPTS
	k_timeout_t evtmout = K_MSEC(1000);
#else
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
#endif
	struct mec_dma_cfg4 dcfg = {0};
	uint8_t *xbuf = NULL;
	uint8_t *rem = NULL;
	uint32_t cm_flags = 0, remlen = 0, maddr = 0;
#ifndef I2C_MEC5_NL_USE_INTERRUPTS
	uint32_t i2c_status = 0;
#endif
	int ret = 0;
	uint16_t ntx = 0;
	uint8_t mflags = 0;

	mflags = m->flags;
	if ((data->state == I2C_NL_STATE_CLOSED) || (data->dir == I2C_NL_DIR_RD)) {
		mflags |= I2C_MSG_RESTART; /* force START */
	}

	data->state = I2C_NL_STATE_CM;
	data->dir = I2C_NL_DIR_WR;

	if (mflags & I2C_MSG_STOP) {
		cm_flags |= MEC_I2C_NL_FLAG_STOP;
	}

	xbuf = &data->xfrbuf[0];
	if (mflags & I2C_MSG_RESTART) {
		maddr = (uint32_t)xbuf;
		ntx = 1;
		cm_flags |= MEC_I2C_NL_FLAG_START | MEC_I2C_NL_FLAG_FLUSH_BUF;
		*xbuf = fmt_addr(data->i2c_addr, I2C_NL_DIR_WR);
		xbuf++;

		if (m->len <= devcfg->cm_tx_buf_max_sz) {
			memcpy(xbuf, m->buf, m->len);
			ntx += m->len;
		} else {
			ntx++;
			*xbuf++ = m->buf[0];
			rem = &m->buf[1];
			remlen = m->len - 1;
		}
	} else {
		maddr = (uint32_t)m->buf;
		ntx = m->len;
	}

	if (!rem && (mflags & I2C_MSG_STOP)) {
		cm_flags |= MEC_I2C_NL_FLAG_STOP;
	}

#ifdef I2C_MEC5_NL_USE_INTERRUPTS
	cm_flags |= MEC_I2C_NL_FLAG_CM_DONE_IEN;
#endif
	dcfg.mem_addr = (uintptr_t)maddr;
	dcfg.len = ntx;
	dcfg.unitsz = MEC_DMAC_UNIT_SIZE_1;
	dcfg.dir = MEC_DMAC_DIR_MEM_TO_DEV;
	dcfg.hwfc_dev = devcfg->cm_dma_trigsrc;
	dcfg.flags = MEC_DMA_CFG4_FLAG_INCR_MEM_ADDR;

	ret = mec_hal_dma_chan_cfg4_by_id(devcfg->cm_dma_chan, &dcfg);
	ret |=mec_hal_dma_chan_start(devcfg->cm_dma_chan);
	ret |= mec_hal_i2c_nl_cm_start(i2c_regs, ntx, 0, cm_flags, &data->cm_cmd);
	if (ret) {
		return -EIO;
	}

	/* I2C-NL transmit: TX DMA finishes moving bytes from memory to I2C-NL.CM_TXB buffer
	 * register before I2C-NL finishes transmitting the last byte and checking (n)ACK.
	 * We deduce I2C-NL sets CM_DONE after the (n)ACK since it has a status bit for
	 * NAK from target and will stop the transfer if a NAK is received.
	 * Another timing issue: race condition with this polling code and HW.
	 * If we see CM_DONE==0 and before we read HW status again I2C-NL does STOP
	 * then I2C-NL HW will clear CM_DONE and set IDLE.
	 * This is strange because sometimes we see both CM_DONE and IDLE and
	 * we just hung here due to IDLE only: i2c_status = 0x2200_0081.
	 * An interrupt from another subsystem (kernel timer, etc.) could preempt reading
	 * I2C-NL HW allowing time for IDLE to occur. CM_DONE is supposed to be R/W1C (latched) so
	 * why do we miss CM_DONE? Because CM_CMD = 0x0002_0003. It is not done, wrCnt = 2.
	 * BUG is previous msg was I2C_MSG_WRITE | I2C_MSG_STOP, current msg is I2C_MSG_WRITE
	 * this routine did not add START!
	 */
#ifdef I2C_MEC5_NL_USE_INTERRUPTS
	data->ev = k_event_wait(&data->events, I2C_NL_WAIT_CM_EVENTS_MSK, false, evtmout);
	if (data->ev == 0) {
		return -ETIMEDOUT;
	} else if (data->ev & I2C_NL_ERRORS) {
		return -EIO;
	}
	k_event_clear(&data->events, I2C_NL_WAIT_CM_EVENTS_MSK);
#else
	i2c_status = mec_hal_i2c_smb_status(hwctx, 0);
	while (!(i2c_status & BIT(MEC_I2C_STS_CM_DONE_POS))) {
		i2c_status = mec_hal_i2c_smb_status(hwctx, 0);
	}

	if (i2c_status & BIT(MEC_I2C_STS_CM_TX_NACK_POS)) {
		return -EIO;
	}
#endif

	if (rem) {
		if (mflags & I2C_MSG_STOP) {
			cm_flags |= MEC_I2C_NL_FLAG_STOP;
		}

		dcfg.mem_addr = (uintptr_t)rem;
		dcfg.len = remlen;
		dcfg.unitsz = MEC_DMAC_UNIT_SIZE_1;
		dcfg.dir = MEC_DMAC_DIR_MEM_TO_DEV;
		dcfg.hwfc_dev = devcfg->cm_dma_trigsrc;
		dcfg.flags = MEC_DMA_CFG4_FLAG_INCR_MEM_ADDR;

		ret = mec_hal_dma_chan_cfg4_by_id(devcfg->cm_dma_chan, &dcfg);
		ret |=mec_hal_dma_chan_start(devcfg->cm_dma_chan);
		ret |= mec_hal_i2c_nl_cm_start(i2c_regs, ntx, 0, cm_flags, &data->cm_cmd);
		if (ret) {
			return -EIO;
		}

#ifdef I2C_MEC5_NL_USE_INTERRUPTS
		data->ev = k_event_wait(&data->events, I2C_NL_WAIT_CM_EVENTS_MSK, false, evtmout);
		if (data->ev == 0) {
			return -ETIMEDOUT;
		} else if (data->ev & I2C_NL_ERRORS) {
			return -EIO;
		}
		k_event_clear(&data->events, I2C_NL_WAIT_CM_EVENTS_MSK);
#else
		i2c_status = mec_hal_i2c_smb_status(hwctx, 0);
		while (!(i2c_status & BIT(MEC_I2C_STS_CM_DONE_POS))) {
			i2c_status = mec_hal_i2c_smb_status(hwctx, 0);
		}

		if (i2c_status & BIT(MEC_I2C_STS_CM_TX_NACK_POS)) {
			return -EIO;
		}
#endif
	}

	if (mflags & I2C_MSG_STOP) {
		data->dir = I2C_NL_DIR_NONE;
		data->state = I2C_NL_STATE_CLOSED;
	}

	return ret;
}

static int process_i2c_msgs(const struct device *dev)
{
	/* const struct i2c_mec5_nl_config *const devcfg = dev->config; */
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
/*	uint32_t events = 0; */
	int ret = 0;

	/* int owned = mec_hal_i2c_smb_is_bus_owned(hwctx); */

	for (uint8_t n = 0; n < data->num_msgs; n++) {
		struct i2c_msg *m = &data->msgs[n];

		data->msgidx = n;
		if (m->flags & I2C_MSG_READ) {
			ret = do_i2c_rd(dev, m);
		} else {
			ret = do_i2c_wr(dev, m);
		}

		if (ret) {
			data->state = I2C_NL_STATE_CLOSED;
			LOG_ERR("do_i2c_x for msg[%u] returned error %d", n, ret);
			break;
		}
	}

	if (!mec_hal_i2c_smb_is_bus_owned(hwctx)) {
		data->state = I2C_NL_STATE_CLOSED;
	}

	return ret;
}

/* Interrupt handler section */

#ifdef CONFIG_I2C_TARGET
static int i2c_mec5_nl_cfg_target_mode(const struct device *dev, uint8_t flags)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t tm_flags = MEC_I2C_NL_TM_FLAG_DONE_IEN | MEC_I2C_NL_TM_FLAG_RUN;
	uint32_t clrmsk = BIT(MEC_I2C_IEN_IDLE_POS) | BIT(MEC_I2C_NL_IEN_CM_DONE_POS)
			  | BIT(MEC_I2C_NL_IEN_TM_DONE_POS) | BIT(MEC_I2C_NL_IEN_AAT_POS);
	uint16_t nrx = devcfg->tm_rx_buf_sz + 1u;
	uint16_t ntx = I2C_MEC5_NL_MAX_XFR_LEN;
	enum mec_dmac_channel tm_dma_chan = (enum mec_dmac_channel)devcfg->tm_dma_chan;
#if 0
	I2C_NL_DEBUG_ISR_INIT(data);
	I2C_NL_DEBUG_STATE_INIT(data);
#endif
	mec_hal_i2c_smb_intr_ctrl(hwctx, clrmsk, 0);
	mec_hal_i2c_nl_cmd_clear(hwctx, MEC_I2C_NL_TM_SEL);
	mec_hal_i2c_nl_flush_buffers(hwctx->base);

	/* clear all events before re-arming TM */
	k_event_clear(&data->events, I2C_NL_ALL_TM_EVENTS);

	data->tm_rd_cnt = 0;

	if (flags & BIT(0)) {
		mec_hal_i2c_nl_tm_config(hwctx, ntx, nrx, tm_flags);
		mec_hal_dma_chan_cfg4_by_id(tm_dma_chan, &data->tm_dma_cfg);
		mec_hal_dma_chan_start(tm_dma_chan);
	}

	return 0;
}

/* i2c_addr is the HW captured 8-bit from the bus: b[7:1]=7-bit I2C target address
 * and bit[0] = R/nW direction bit.
 */
static struct i2c_target_config *get_target(const struct device *dev, uint16_t i2c_addr)
{
	struct i2c_mec5_nl_data *const data = dev->data;
	uint16_t target_addr = (i2c_addr >> 1) & 0x7Fu;

	if (data->target1_cfg) {
		if (data->target1_cfg->address == target_addr) {
			return data->target1_cfg;
		}
	}

	if (data->target2_cfg) {
		if (data->target2_cfg->address == target_addr) {
			return data->target2_cfg;
		}
	}

	return NULL;
}

static void tm_rd(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	struct mec_dma_cfg4 *dcfg = &data->tm_dma_cfg;
	const struct i2c_target_callbacks *cbs = data->curr_target->callbacks;
	int ret = 0;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xA0u);

	data->tm_tx_buf = NULL;
	data->tm_tx_buf_sz = 0;
	ret = cbs->buf_read_requested(data->curr_target, &data->tm_tx_buf, &data->tm_tx_buf_sz);
	if (ret) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0xA1u);
		mec_hal_i2c_smb_idle_intr_enable(hwctx, 1u);
		return;
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xA2u);
	/* configure TM DMA channel for mem2dev from I2C-NL.TM_RXB, start DMA,
	 * and unpause I2C-NL by setting PROC=1.
	 */
	dcfg->mem_addr = (uintptr_t)data->tm_tx_buf;
	dcfg->len = data->tm_tx_buf_sz;
	dcfg->unitsz = MEC_DMAC_UNIT_SIZE_1;
	dcfg->dir = MEC_DMAC_DIR_DEV_TO_MEM;
	dcfg->hwfc_dev = devcfg->tm_dma_trigsrc;
	dcfg->flags = MEC_DMA_CFG4_FLAG_INCR_MEM_ADDR;

	mec_hal_dma_chan_cfg4_by_id(devcfg->tm_dma_chan, dcfg);
	mec_hal_dma_chan_start(devcfg->tm_dma_chan);
	mec_hal_i2c_nl_cm_proceed(hwctx);

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xA3u);
}

/* Called only if events has BIT(I2C_NL_KEV_TM_DONE_POS) set
 * Host I2C write or read:
 * In both cases Host generated START and transmitted target address to this target.
 * I2C-NL rdCnt = len(driver TM RX buf) +  2.
 * TM DMA is dev2mem from I2C-NL to driver TM RX buf. len = rdCnt.
 * 1. I2C Write
 *    I2C-NL and TM DMA RX mode moved target write address and data bytes to driver TM RX buf.
 *      Host wrote n < rdCnt bytes:
 *        TM_DONE=1 when STOP detected. rdCnt decremented by n. PROC=0, RUN=0.
 *      Host wrote n == rdCnt bytes and issued STOP.
 *        Same as above except rdCnt == 0.
 *      Host wrote n > rdCnt bytes.
 *        After rdCnt bytes, I2C-NL NAK's any future bytes and sets TM_NAKR status causing TM_DONE=1
 *      Host wrote n <= rdCnt bytes where the last byte was target address for RPT-START.
 *        TM_DONE=1, rdCnt decremented by n. PROC=0, RUN=1
 *
 * 2. I2C Read
 *    Host transmitted START plus matching read address (bit[0]==1)
 *    I2C-NL and TM DMA moved target read address into driver TM RX buffer.
 *      rdCnt decremented by 1, PROC=0, RUN=1. I2C-NL is in pause state with clock stretched.
 *      We need to get application buffer via buf_read_req callback
 *      Reconfigure TM DMA for (buf, len) supplied by callback.
 *      Set TM_CMD wrCnt = buffer length.
 *      Set PROC=1 to unpause I2C-NL HW.
 *      Enable IDLE interrupt. No need for TM DMA interrupt.
 */
static void handle_tm(const struct device *dev, uint32_t tm_cmd)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct mec_i2c_smb_regs *i2c_regs = devcfg->i2c_regs;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t events = data->ev;
	uint8_t i2c_addr = mec_hal_i2c_nl_shad_addr_get(i2c_regs);

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x90u);

	data->curr_target = get_target(dev, i2c_addr);
	if (!data->curr_target) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x91);
		mec_hal_i2c_smb_idle_intr_enable(hwctx, 1u);
		return;
	}

	/* RPT-START plus new target address received */
	if (events & BIT(I2C_NL_KEV_TM_PAUSE_POS)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x92u);
		if (events & BIT(I2C_NL_KEV_TM_RPT_RD_POS)) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x93u);
			tm_rd(dev);
			return;
		}

		if (events & BIT(I2C_NL_KEV_TM_RPT_WR_POS)) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x94u);
			/* TODO */
		}
	} else { /* I2C-NL TM FSM terminated */
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x98u);
		if (i2c_addr & BIT(0)) { /* Read? */
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x99u);
			tm_rd(dev);
			return;
		} else { /* Host wrote data. Termination condition: STOP or recv buffer filled */
			/* It takes time for I2C-NL to ACK and trigger DMA to move last data
			 * into memory. If EC is idle when this happens DMA may not be finished
			 * when callback is run. Enable TM DMA done interrupt and have TM DMA
			 * ISR invoke buf_write_received callback.
			 */
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x9Au);
			data->xfr_flags |= BIT(4);
			mec_hal_dma_chan_ien(devcfg->tm_dma_chan, (BIT(MEC_DMA_CHAN_STS_DONE_POS) |
						BIT(MEC_DMA_CHAN_STS_HFC_TERM_POS)), 1u);
		}
	}
	I2C_NL_DEBUG_STATE_UPDATE(data, 0x9Fu);
}
#endif

static uint32_t get_events(uint32_t i2c_status, uint32_t cm_cmd, uint32_t tm_cmd)
{
	uint32_t events = 0;

	if (i2c_status & BIT(MEC_I2C_STS_LL_NBB_POS)) {
		events |= BIT(I2C_NL_KEV_IDLE_POS);
	}
	if (i2c_status & BIT(MEC_I2C_STS_LL_BER_POS)) {
		events |= BIT(I2C_NL_KEV_BERR_POS);
	}
	if (i2c_status & BIT(MEC_I2C_STS_LL_LAB_POS)) {
		events |= BIT(I2C_NL_KEV_LAB_ERR_POS);
	}
	if (i2c_status & BIT(MEC_I2C_STS_CM_TX_NACK_POS)) {
		events |= BIT(I2C_NL_KEV_CM_NAK_POS);
	}
	if (i2c_status & BIT(MEC_I2C_STS_CM_DONE_POS)) {
		events |= BIT(I2C_NL_KEV_CM_DONE_POS);
		if ((cm_cmd & (BIT(0) | BIT(1))) == BIT(0)) {
			events |= BIT(I2C_NL_KEV_CM_PAUSE_POS);
		}
	}
#ifdef CONFIG_I2C_TARGET
	if (i2c_status & BIT(MEC_I2C_STS_TM_DONE_POS)) {
		events |= BIT(I2C_NL_KEV_TM_DONE_POS);
		if ((tm_cmd & (BIT(0) | BIT(1))) == BIT(0)) {
			events |= BIT(I2C_NL_KEV_TM_PAUSE_POS);
		}
	}
	if (i2c_status & BIT(MEC_I2C_STS_TM_NACKR_POS)) {
		events |= BIT(I2C_NL_KEV_TM_NAKR_POS);
	}
	if (i2c_status & BIT(MEC_I2C_STS_TM_PERR_POS)) {
		events |= BIT(I2C_NL_KEV_TM_TPROT_POS);
	}
	if (i2c_status & BIT(MEC_I2C_STS_TM_RPTS_RD_POS)) {
		events |= BIT(I2C_NL_KEV_TM_RPT_RD_POS);
	}
	if (i2c_status & BIT(MEC_I2C_STS_TM_RPTS_WR_POS)) {
		events |= BIT(I2C_NL_KEV_TM_RPT_WR_POS);
	}
#endif
	return events;
}

static void i2c_mec5_nl_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
/*	const struct i2c_mec5_nl_config *const devcfg = dev->config; */
/*	struct mec_i2c_smb_regs *i2c_regs = devcfg->i2c_regs; */
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t cm_cmd = 0, tm_cmd = 0;

	I2C_NL_DEBUG_ISR_COUNT_UPDATE(data);
	I2C_NL_DEBUG_STATE_UPDATE(data, 0x80u);

	/* get I2C HW status and clear it */
	data->i2c_status = mec_hal_i2c_smb_status(hwctx, 1);
	cm_cmd = mec_hal_i2c_nl_cmd_get(hwctx, 0);
#ifdef CONFIG_I2C_TARGET
	tm_cmd = mec_hal_i2c_nl_cmd_get(hwctx, 1);
#endif
	data->ev = get_events(data->i2c_status, cm_cmd, tm_cmd);

	I2C_NL_DEBUG_ISR_DATA_UPDATE(data);

	if (data->ev & BIT(I2C_NL_KEV_IDLE_POS)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x81u);
		mec_hal_i2c_smb_idle_intr_enable(hwctx, 0);
		mec_hal_i2c_smb_idle_status_clr(hwctx);
#ifdef CONFIG_I2C_TARGET
		if (data->state == I2C_NL_STATE_TM) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x8Cu);
			data->state = I2C_NL_STATE_CLOSED;
			const struct i2c_target_callbacks *cbs = data->curr_target->callbacks;

			cbs->stop(data->curr_target);
			i2c_mec5_nl_cfg_target_mode(dev, 1u);
		}
#endif
	}

#ifdef CONFIG_I2C_TARGET
	if (data->ev & BIT(I2C_NL_KEV_TM_DONE_POS)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x8Du);
		data->state = I2C_NL_STATE_TM;
		handle_tm(dev, tm_cmd);
	}
#endif
	mec_hal_i2c_smb_girq_status_clr(hwctx);
	if (data->ev) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x8Eu);
		k_event_post(&data->events, data->ev);
	}
	I2C_NL_DEBUG_STATE_UPDATE(data, 0x8Fu);
}

static void i2c_mec5_nl_cm_dma_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	enum mec_dmac_channel chan = devcfg->cm_dma_chan;
/*	uint32_t dma_status = 0; */

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xD0u);

	mec_hal_dma_chan_intr_en(chan, 0);
/*	mec_hal_dma_chan_intr_status(chan, &dma_status); */
	mec_hal_dma_chan_intr_status_clr(chan);

	k_event_post(&data->events, BIT(I2C_NL_KEV_DMA_CM_DONE_POS));
	I2C_NL_DEBUG_STATE_UPDATE(data, 0xD1u);
}

#ifdef CONFIG_I2C_TARGET
/* TODO */
static void i2c_mec5_nl_tm_dma_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	enum mec_dmac_channel chan = devcfg->tm_dma_chan;
	uint32_t nrem = 0, dlen = 0;
	uint8_t *pdata = NULL;
/*	uint32_t dma_status = 0; */

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xD8u);

	mec_hal_dma_chan_intr_en(chan, 0);
/*	mec_hal_dma_chan_intr_status(chan, &dma_status); */
	mec_hal_dma_chan_intr_status_clr(chan);

	k_event_post(&data->events, BIT(I2C_NL_KEV_DMA_TM_DONE_POS));

	if (data->xfr_flags & BIT(4)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0xD9u);
		data->xfr_flags &= ~BIT(4);

		const struct i2c_target_callbacks *cbs = data->curr_target->callbacks;

		mec_hal_dma_chan_rem_bytes(chan, &nrem);
		dlen = data->tm_dma_cfg.len - nrem;
		pdata = devcfg->tm_rx_buf;
		/* TODO handle I2C-NL anomaly for external Host write sequence:
		 * START wrAddr, data..., STOP
		 * I2C-NL triggers DMA to store the START target address in the
		 * memory buffer. For START the target address is at offset 0.
		 * The anomly is I2C-NL trigger DMA to store a bogus 0 byte on
		 * STOP reception. We will alway receive > 2 bytes for this protocol.
		 * Adjust size by subtracting 2 and set data pointer to offset 1
		 * in buffer.
		 */
		if (dlen >= 2) {
			dlen -= 2u;
			pdata++;
		}
		cbs->buf_write_received(data->curr_target, pdata, dlen);
		mec_hal_i2c_smb_idle_intr_enable(hwctx, 1u);
	}
	I2C_NL_DEBUG_STATE_UPDATE(data, 0xDFu);
}
#endif
/* end Interrupt handler section */

/* ---- Public API ---- */

static int i2c_mec5_nl_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_mec5_nl_data *data = dev->data;
	uint32_t speed = I2C_SPEED_GET(dev_config);
	int ret = 0;

	switch (speed) {
	case I2C_SPEED_STANDARD:
		data->clock_freq_hz = KHZ(100);
		break;
	case I2C_SPEED_FAST:
		data->clock_freq_hz = KHZ(400);
		break;
	case I2C_SPEED_FAST_PLUS:
		data->clock_freq_hz = MHZ(1);
		break;
	default:
		return -EINVAL;
	}

	ret = i2c_mec5_nl_reset_config(dev);
	if (ret) {
		return ret;
	}

	/* wait for NBB=1, BER, LAB, or timeout */
	ret = wait_bus_free(dev, I2C_MEC5_NL_WAIT_COUNT);

	return ret;
}

/* Side-band API */
int i2c_mchp_nl_configure(const struct device *dev, uint32_t dev_config, uint8_t port_num)
{
	if (!dev || (port_num >= MCHP_I2C_NL_NUM_PORTS)) {
		return -EINVAL;
	}

	struct i2c_mec5_nl_data *data = dev->data;

	data->misc_cfg &= (uint32_t)~I2C_MEC5_NL_MISC_CFG_PORT_MSK;
	data->misc_cfg |= (((uint32_t)port_num << I2C_MEC5_NL_MISC_CFG_PORT_POS) &
			   I2C_MEC5_NL_MISC_CFG_PORT_MSK);

	return i2c_mec5_nl_configure(dev, dev_config);
}

int i2c_mchp_nl_get_port(const struct device *dev, uint8_t *port_num)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct mec_i2c_smb_regs *regs = devcfg->i2c_regs;
	uint8_t port = mec_hal_i2c_smb_port_get(regs);

	if (!port_num) {
		return -EINVAL;
	}

	*port_num = port;
	return 0;
}

static int i2c_mec5_nl_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct i2c_mec5_nl_data *data = dev->data;
	uint32_t bus_freq_hz = 0, cfg = 0;
	int ret = 0;

	if (!dev_config) {
		return -EINVAL;
	}

	ret = mec_hal_i2c_smb_bus_freq_get_by_ctx(&data->ctx, &bus_freq_hz);
	if (ret != MEC_RET_OK) {
		return -EIO;
	}

	ret = 0;

	if (!(data->misc_cfg & BIT(7))) {
		cfg |= I2C_MODE_CONTROLLER;
	}

	if ((bus_freq_hz >= KHZ(90)) && (bus_freq_hz <= KHZ(110))) {
		cfg |= I2C_SPEED_SET(I2C_SPEED_STANDARD);
	} else if ((bus_freq_hz >= KHZ(340)) && (bus_freq_hz <= KHZ(440))) {
		cfg |= I2C_SPEED_SET(I2C_SPEED_FAST);
	} else if ((bus_freq_hz >= KHZ(900)) && (bus_freq_hz <= KHZ(1100))) {
		cfg |= I2C_SPEED_SET(I2C_SPEED_FAST_PLUS);
	} else {
		ret = -ERANGE;
	}

	*dev_config = cfg;

	return ret;
}

static int i2c_mec5_nl_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				uint16_t addr)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	int ret = 0;

	if (addr & 0xff80u) {
		LOG_ERR("Bad I2C address: HW only supports 7-bit addresses");
		return -EINVAL;
	}

	if (!req_is_supported(msgs, num_msgs)) {
		return -EINVAL;
	}

	k_sem_take(&data->lock, K_FOREVER); /* decrements count */
	k_event_clear(&data->events, UINT32_MAX);

	I2C_NL_DEBUG_ISR_INIT(data);
	I2C_NL_DEBUG_STATE_INIT(data);

	i2c_mec5_nl_xfr_data_init(dev, msgs, num_msgs, addr);
	msgs_compute_timeout(dev);
	ret = process_i2c_msgs(dev);
	if (ret) {
		mec_hal_dma_chan_stop(devcfg->cm_dma_chan);
		i2c_mec5_nl_reset_config(dev);
	}

	data->state = I2C_NL_STATE_CLOSED;
	k_sem_give(&data->lock);

	return ret;
}

#ifdef CONFIG_I2C_TARGET
/* I2C-NL supports up to two 7-bit target addresses */
static int i2c_mec5_nl_target_register(const struct device *dev, struct i2c_target_config *cfg)
{
	struct i2c_mec5_nl_data *const data = dev->data;
	int ret = 0;
	uint8_t targ_addr = 0;

	if (!cfg || (cfg->address > 0x7fu) || (!cfg->callbacks)) {
		return -EINVAL;
	}

	const struct i2c_target_callbacks *cbs = cfg->callbacks;

	if (!cbs->buf_write_received || !cbs->buf_read_requested || !cbs->stop) {
		LOG_ERR("One or more of the three required callbacks is NULL!");
		return -EINVAL;
	}

	if (k_sem_take(&data->lock, K_NO_WAIT) != 0) {
		return -EBUSY;
	}

	I2C_NL_DEBUG_ISR_INIT(data);
	I2C_NL_DEBUG_STATE_INIT(data);

	targ_addr = (uint8_t)(cfg->address & 0x7fu);

	if (!data->target1_cfg && !data->target2_cfg) {
		I2C_NL_DEBUG_ISR_INIT(data);
		data->target1_cfg = cfg;
		mec_hal_i2c_smb_set_target_addr(&data->ctx, 0, targ_addr);
		ret = i2c_mec5_nl_cfg_target_mode(dev, 1);
	} else if (!data->target1_cfg) {
		data->target1_cfg = cfg;
		mec_hal_i2c_smb_set_target_addr(&data->ctx, 0, targ_addr);
	} else if (!data->target2_cfg) {
		data->target2_cfg = cfg;
		mec_hal_i2c_smb_set_target_addr(&data->ctx, 1, targ_addr);
	} else {
		ret = -EAGAIN;
	}

	k_sem_give(&data->lock);

	return ret;
}

static int i2c_mec5_nl_target_unregister(const struct device *dev, struct i2c_target_config *cfg)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;

	if (k_sem_take(&data->lock, K_NO_WAIT) != 0) {
		return -EBUSY;
	}

	if (data->target1_cfg == cfg) {
		data->target1_cfg = NULL;
		mec_hal_i2c_smb_set_target_addr(&data->ctx, 0, 0);

	} else if (data->target2_cfg == cfg) {
		data->target2_cfg = NULL;
		mec_hal_i2c_smb_set_target_addr(&data->ctx, 1, 0);
	}

	if (!data->target1_cfg && !data->target2_cfg) {
		mec_hal_i2c_smb_intr_ctrl(&data->ctx, BIT(MEC_I2C_NL_IEN_TM_DONE_POS), 0);
		mec_hal_i2c_nl_cmd_clear(&data->ctx, 1);
		mec_hal_i2c_smb_status(&data->ctx, 1);
		mec_hal_dma_chan_stop(devcfg->tm_dma_chan);
	}

	k_sem_give(&data->lock);

	return 0;
}
#else
/* I2C-NL supports up to two 7-bit target addresses */
static int i2c_mec5_nl_target_register(const struct device *dev, struct i2c_target_config *cfg)
{
	return -ENOTSUP;
}

static int i2c_mec5_nl_target_unregister(const struct device *dev, struct i2c_target_config *cfg)
{
	return -ENOTSUP;
}
#endif /* CONFIG_I2C_TARGET */

static const struct i2c_driver_api i2c_mec5_nl_driver_api = {
	.configure = i2c_mec5_nl_configure,
	.get_config = i2c_mec5_nl_get_config,
	.transfer = i2c_mec5_nl_transfer,
	.target_register = i2c_mec5_nl_target_register,
	.target_unregister = i2c_mec5_nl_target_unregister,
};
/* end public API */

/* Configure DMA channels used by I2C-NL driver.
 * I2C-NL uses Central DMA controller channels. We must not reset Central DMA unless
 * it has not yet been enabled.
 * NOTE: Target receive where external Controller writes to this device.
 * I2C-NL triggers TM DMA channel to store the target adddress byte followed by data.
 * We configure TM DMA memory buffer to be offset 3 in the drivers sized padded buffer.
 * This allows us to pass an aligned pointer to the application callback since all it
 * wants is the data.
 */
static int i2c_mec5_nl_dma_init(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	int ret = 0;

	/* HAL DMA init */
	if (!mec_hal_dmac_is_enabled()) {
		mec_hal_dmac_init(0);
	}

	ret = mec_hal_dma_chan_init(devcfg->cm_dma_chan);
	if (ret != MEC_RET_OK) {
		LOG_ERR("CM DMA chan init failed (%d)", ret);
		return -EIO;
	}

#ifdef CONFIG_I2C_TARGET
	ret = mec_hal_dma_chan_init((enum mec_dmac_channel)devcfg->tm_dma_chan);
	if (ret != MEC_RET_OK) {
		LOG_ERR("TM DMA chan init failed (%d)", ret);
		return -EIO;
	}

	struct mec_dma_cfg4 *dcfg = &data->tm_dma_cfg;

	dcfg->mem_addr = (uint32_t)devcfg->tm_rx_buf;
	/* add two bytes for START and possible RPT-START addresses */
	dcfg->len = devcfg->tm_rx_buf_sz + 2u;
	dcfg->unitsz = MEC_DMAC_UNIT_SIZE_1;
	dcfg->dir = MEC_DMAC_DIR_DEV_TO_MEM;
	dcfg->hwfc_dev = devcfg->tm_dma_trigsrc;
	dcfg->flags = MEC_DMA_CFG4_FLAG_INCR_MEM_ADDR;
#endif
	return 0;
}

static int i2c_mec5_nl_init(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t bitrate_cfg = 0;
	int ret = 0;

	k_sem_init(&data->lock, 1, 1);
	k_event_init(&data->events);

	data->devcfg = devcfg;
	data->i2c_status = 0;
	data->dir = I2C_NL_DIR_NONE;
	data->xfr_tmout = I2C_MEC5_NL_SMBUS_TMOUT_MAX_MS;

	hwctx->base = devcfg->i2c_regs;
	hwctx->i2c_ctrl_cached = 0;

	ret = pinctrl_apply_state(devcfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("pinctrl setup failed (%d)", ret);
		return ret;
	}

	/* HAL DMA init */
	ret = i2c_mec5_nl_dma_init(dev);
	if (ret) {
		return ret;
	}

	bitrate_cfg = i2c_map_dt_bitrate(data->clock_freq_hz);
	if (!bitrate_cfg) {
		return -EINVAL;
	}

	ret = i2c_mec5_nl_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);
	if (ret) {
		return ret;
	}

	if (devcfg->irq_config_func) {
		devcfg->irq_config_func();
		mec_hal_i2c_smb_girq_ctrl(hwctx, MEC_I2C_SMB_GIRQ_EN | MEC_I2C_SMB_GIRQ_CLR_STS);
		mec_hal_dma_chan_ia_enable(devcfg->cm_dma_chan);
#ifdef CONFIG_I2C_TARGET
		mec_hal_dma_chan_ia_enable(devcfg->tm_dma_chan);
#endif
	}

	return 0;
}

/* Node id of DMA channel */
#define I2C_MEC5_NL_DT_INST_DMA_CTLR(i, name) DT_INST_DMAS_CTLR_BY_NAME(i, name)

#define I2C_MEC5_NL_DT_INST_DMA_DEV(i, name) DEVICE_DT_GET(I2C_MEC5_NL_DT_INST_DMA_CTLR(i, name))

#define I2C_MEC5_NL_DT_MISC_CFG(i) (uint8_t)(DT_INST_PROP_OR(i, port_sel, 0) & 0x0f)

#define I2C_NL_MEC5_DMA_CHAN(i, idx) DT_INST_PHA_BY_IDX(i, dmas, idx, channel)

#define I2C_NL_MEC5_DMA_TRIGSRC(i, idx) DT_INST_PHA_BY_IDX(i, dmas, idx, trigsrc)

#define I2C_NL_MEC5_DMA_CM_IRQN(i)                                                                 \
	DT_IRQ_BY_IDX(I2C_MEC5_NL_DT_INST_DMA_CTLR(i, cm), I2C_NL_MEC5_DMA_CHAN(i, 0), irq)

#define I2C_NL_MEC5_DMA_CM_IRQ_PRI(i)                                                              \
	DT_IRQ_BY_IDX(I2C_MEC5_NL_DT_INST_DMA_CTLR(i, cm), I2C_NL_MEC5_DMA_CHAN(i, 0), priority)

#define I2C_NL_MEC5_CM_TX_BUF_SIZE(i) DT_INST_PROP(i, cm_tx_buf_size)

#define I2C_NL_MEC5_CM_TX_MEM_SIZE(i)                                                              \
	(I2C_NL_MEC5_CM_TX_BUF_SIZE(i) + I2C_MEC5_NL_CM_TX_BUF_PAD_LEN)

#define I2C_NL_MEC5_CM_TX_BUF(i)                                                                   \
	static uint8_t i2c_mec5_nl_cm_tx_buf_##i[I2C_NL_MEC5_CM_TX_MEM_SIZE(i)] __aligned(4);

#ifdef CONFIG_I2C_TARGET

BUILD_ASSERT(IS_ENABLED(CONFIG_I2C_TARGET_BUFFER_MODE),
	     "Target buffer mode must be used when Target mode enabled in the build!");

#define I2C_NL_MEC5_TM_RX_BUF_SIZE(i) DT_INST_PROP(i, tm_rx_buf_size)

#define I2C_NL_MEC5_TM_RX_MEM_SIZE(i)                                                              \
	(I2C_NL_MEC5_TM_RX_BUF_SIZE(i) + I2C_MEC5_NL_TM_RX_BUF_PAD_LEN)

#define I2C_NL_MEC5_TM_RX_BUF(i)                                                                   \
	static uint8_t i2c_mec5_nl_tm_rx_buf_##i[I2C_NL_MEC5_TM_RX_MEM_SIZE(i)] __aligned(4);

#define I2C_MEC5_NL_TM_DMA(i)                                                                      \
	.tm_rx_buf_sz = (uint16_t)I2C_NL_MEC5_TM_RX_BUF_SIZE(i),                                   \
	.tm_rx_buf = i2c_mec5_nl_tm_rx_buf_##i, .tm_dma_chan = I2C_NL_MEC5_DMA_CHAN(i, 1),         \
	.tm_dma_trigsrc = I2C_NL_MEC5_DMA_TRIGSRC(i, 1),

#define I2C_NL_MEC5_DMA_TM_IRQN(i)                                                                 \
	DT_IRQ_BY_IDX(I2C_MEC5_NL_DT_INST_DMA_CTLR(i, tm), I2C_NL_MEC5_DMA_CHAN(i, 1), irq)

#define I2C_NL_MEC5_DMA_TM_IRQ_PRI(i)                                                              \
	DT_IRQ_BY_IDX(I2C_MEC5_NL_DT_INST_DMA_CTLR(i, tm), I2C_NL_MEC5_DMA_CHAN(i, 1), priority)

#define I2C_MEC5_NL_TM_DMA_IRQ_CE(i)                                                               \
	IRQ_CONNECT(I2C_NL_MEC5_DMA_TM_IRQN(i),                                                    \
		    I2C_NL_MEC5_DMA_TM_IRQ_PRI(i),                                                 \
		    i2c_mec5_nl_tm_dma_isr,                                                        \
		    DEVICE_DT_INST_GET(i), 0);                                                     \
	irq_enable(I2C_NL_MEC5_DMA_TM_IRQN(i))

#else
#define I2C_NL_MEC5_TM_RX_BUF(i)
#define I2C_NL_MEC5_TM_RX_BUF_PTR(i)
#define I2C_MEC5_NL_TM_DMA(i)
#define I2C_MEC5_NL_TM_DMA_IRQ_CE(i)
#endif

#define I2C_MEC5_NL_DEVICE(i)                                                                      \
	BUILD_ASSERT((I2C_NL_MEC5_DMA_CM_IRQ_PRI(i) < DT_INST_IRQ(i, priority)),                   \
		"CM DMA channel ISR priority must be higher than I2C");                            \
	BUILD_ASSERT((I2C_NL_MEC5_DMA_TM_IRQ_PRI(i) < DT_INST_IRQ(i, priority)),                   \
		"TM DMA channel ISR priority must be higher than I2C");                            \
	I2C_NL_MEC5_CM_TX_BUF(i)                                                                   \
	I2C_NL_MEC5_TM_RX_BUF(i)                                                                   \
	struct i2c_mec5_nl_data i2c_mec5_nl_data_##i = {                                           \
		.clock_freq_hz = DT_INST_PROP(i, clock_frequency),                                 \
		.misc_cfg = I2C_MEC5_NL_DT_MISC_CFG(i),                                            \
		.xfrbuf = &i2c_mec5_nl_cm_tx_buf_##i[0],                                           \
	};                                                                                         \
	PINCTRL_DT_INST_DEFINE(i);                                                                 \
	static void i2c_mec5_nl_irq_config_func_##i(void)                                          \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(i), DT_INST_IRQ(i, priority), i2c_mec5_nl_isr,            \
			    DEVICE_DT_INST_GET(i), 0);                                             \
		irq_enable(DT_INST_IRQN(i));                                                       \
		IRQ_CONNECT(I2C_NL_MEC5_DMA_CM_IRQN(i),                                            \
			    I2C_NL_MEC5_DMA_CM_IRQ_PRI(i),                                         \
			    i2c_mec5_nl_cm_dma_isr,                                                \
			    DEVICE_DT_INST_GET(i), 0);                                             \
		irq_enable(I2C_NL_MEC5_DMA_CM_IRQN(i));                                            \
		I2C_MEC5_NL_TM_DMA_IRQ_CE(i);                                                      \
	}                                                                                          \
	struct i2c_mec5_nl_config i2c_mec5_nl_devcfg_##i = {                                       \
		.i2c_regs = (struct mec_i2c_smb_regs *)DT_INST_REG_ADDR(i),                        \
		.init_pin_wait_us = DT_INST_PROP_OR(i, init_pin_wait, 100),                        \
		.cfg_pin_wait_us = DT_INST_PROP_OR(i, config_pin_wait, 35000),                     \
		.sda_gpio = GPIO_DT_SPEC_INST_GET(i, sda_gpios),                                   \
		.scl_gpio = GPIO_DT_SPEC_INST_GET(i, scl_gpios),                                   \
		.irq_config_func = i2c_mec5_nl_irq_config_func_##i,                                \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i),                                         \
		.cm_dma_chan = I2C_NL_MEC5_DMA_CHAN(i, 0),                                         \
		.cm_dma_trigsrc = I2C_NL_MEC5_DMA_TRIGSRC(i, 0),                                   \
		.cm_tx_buf_max_sz = (uint16_t)I2C_NL_MEC5_CM_TX_BUF_SIZE(i),                       \
		I2C_MEC5_NL_TM_DMA(i)};                                                            \
	I2C_DEVICE_DT_INST_DEFINE(i, i2c_mec5_nl_init, NULL, &i2c_mec5_nl_data_##i,                \
				  &i2c_mec5_nl_devcfg_##i, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,  \
				  &i2c_mec5_nl_driver_api)

DT_INST_FOREACH_STATUS_OKAY(I2C_MEC5_NL_DEVICE)
