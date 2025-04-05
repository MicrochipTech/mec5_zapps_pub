/*
 * Copyright (c) 2025 Microchip Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_i2c_nl

#include <errno.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/devicetree/dma.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/sys_io.h>
LOG_MODULE_REGISTER(i2c_nl_zd_mchp, CONFIG_I2C_LOG_LEVEL);

#include <app/drivers/mchp_mec5_i2c_nl.h>

/* out of tree build requires this path for headers in the driver source
 * directory */
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
/* #define I2C_MEC5_NL_DEBUG_INTERRUPT_SPIN */

#define I2C_MEC5_NL_RESET_WAIT_US 20

/* SMBus specification default timeout in milliseconds */
#define I2C_MEC5_NL_SMBUS_TMOUT_MAX_MS 35

/* I2C timeout is  10 ms (WAIT_INTERVAL * WAIT_COUNT) */
#define I2C_MEC5_NL_WAIT_INTERVAL_US 50
#define I2C_MEC5_NL_WAIT_COUNT       200
#define I2C_MEC5_NL_STOP_WAIT_COUNT  500
#define I2C_MEC5_NL_PIN_CFG_WAIT     50

#define I2C_MEC5_NL_RECOVER_SCL_DLY_US 10
#define I2C_MEC5_NL_RECOVER_SCL_CNT    10
#define I2C_MEC5_NL_RECOVER_SDA_DLY_US 5
#define I2C_MEC5_NL_RECOVER_SDA_CNT    10

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
	I2C_NL_KEV_ALL_DONE_POS = 31,
};

enum i2c_mec5_nl_xfr_flags {
	I2C_NL_XFR_FLAG_ASYNC_POS = 0,
};

#define I2C_NL_EVENTS_ERRORS                                                                       \
	(BIT(I2C_NL_KEV_BERR_POS) | BIT(I2C_NL_KEV_LAB_ERR_POS) | BIT(I2C_NL_KEV_CM_NAK_POS))

#define I2C_NL_WAIT_CM_EVENTS_MSK                                                                  \
	(BIT(I2C_NL_KEV_CM_NAK_POS) | BIT(I2C_NL_KEV_CM_PAUSE_POS) | BIT(I2C_NL_KEV_CM_DONE_POS) | \
	 BIT(I2C_NL_KEV_DMA_CM_DONE_POS))

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
	void (*irq_config_func)(void);
	const struct pinctrl_dev_config *pcfg;
	const struct device *dma_dev_cm;
	uint8_t cm_dma_chan;
	uint8_t cm_dma_trigsrc;
	uint16_t cm_tx_buf_max_sz; /* value does not include
				      I2C_MEC5_NL_CM_TX_BUF_PAD_LEN */
#ifdef CONFIG_I2C_TARGET
	const struct device *dma_dev_tm;
	uint8_t *tm_rx_buf;
	uint16_t tm_rx_buf_sz;
	uint8_t tm_dma_chan;
	uint8_t tm_dma_trigsrc;
#endif
};

#ifdef I2C_NL_DEBUG_ISR
struct i2c_mec5_dbg_isr_data {
	uint32_t isr_idx;
	uint32_t isr_cnt;
	uint32_t status;
	uint32_t config;
	uint32_t cm_cmd;
	uint32_t extlen;
	uint32_t ev;
};
#endif

/* I2C-NL DMA usage
 * HW requires the I2C target address to be part of the DMA byte stream.
 * I2C START
 * byte offset, description
 * 0            i2c_target address for START
 * 1            tx data[0]
 * ...
 * ntx          tx data[ntx-1]
 * ntx+1        i2c_target address for RPT-START
 * ntx+2        rx data[0] from target
 * ...
 * ntx+nrx+1    rx data[nrx-1]
 * I2C STOP
 * Also, I2C-NL HW 16-bit write and read count registers are decremented
 * as each byte is shifted out/in. When these registers reach 0 the HW statemachine
 * will perform actions based on other HW transfer flags.
 * If write and read counts are both non-zero when write count reaches 0 the FSM
 * will pause clearing the command register PROCEED bit and setting the DONE status.
 * DONE can generate an interrupt. Software must then re-configure the DMA channel
 * for device to memory direction and set number of DMA bytes to nrx.
 * The HW was designed for write only and write followed by read.
 * I2C Write: START wrAddr [ACK] data0 [ACK], ..., dataNwr [ACK] STOP IDLE(interrupt).
 * I2C Read: START rdAddr [ACK] PAUSE(interrupt) [data0] ACK, ..., [dataNrd] ACK STOP
 * IDLE(interrupt) PAUSE interrupt requires FW to re-configure DMA for read phase I2C Write-Read:
 * START wrAddr [ACK] wrdata0 [ACK], ..., wrdataNwr [ACK] RPT-START rdAddr [ACK] PAUSE(interrupt)
 * [data0] ACK, ..., [dataNrd] ACK STOP IDLE(interrupt)
 *
 * The driver requires a TX buffer to create a byte stream containing the target addresses.
 * The driver TX buffer size is set by a device tree property allowing the application to
 * configure a different TX buffer size per instance.
 * driver txbuf[txbuf_size + 4]
 * Due to the I2C-NL HW limitations we can process the following message sequences:
 * 1. One I2C_MSG_WRITE with STOP
 * 2. One I2C_MSG_READ with STOP
 * 3. Two messages:
 *    first: I2C_MSG_WRITE no STOP
 *    second: I2C_MSG_READ with STOP.
 *
 * Data byte length HW limitations. I2C-NL implements separate 16-bit write and read byte
 * count registers. Due to addresses being part of the DMA transfers the maximum data
 * payload size is 0xffff - 2u = 0xfffd bytes.
 *
 * DMA considerations:
 * Zephyr DMA driver for supports a linked list of data blocks in the same direction.
 * DMA driver configuration API sets the direction.
 * DMA driver configuration structure contains a pointer to a linked list of struct dma_blocks.
 * Each DMA block describes a buffer and size to store data. The DMA ISR reloads the next block.
 * Current sizes: struct dma_config = 32 bytes and struct dma_block = 28 bytes.
 *
 * simple use cases:
 * 1. I2C write message fits in driver TX buffer
 *    One DMA config structure and one DMA block struture required.
 *
 * 2. I2C write message larger than driver TX buffer
 *    One DMA config structure
 *    First DMA block structure for target write address
 *    Second DMA block structure for TX data
 *    If Rpt-START required
 *        Third DMA block structure for Rpt-START target address
 *
 * 3. I2C read message
 *    One DMA config structure
 *    First DMA block structure for target write address
 *    Second DMA config or we re-use the first DMA Config structure for read direction
 *    Second DMA block struture for read data
 *
 * 4. Combined I2C Write-Read
 *    One DMA config structure for write direction
 *    Up to three DMA block structures for TX phase which includes RPT-Start
 *    Second DMA block config or re-use first DMA config for read direction
 *    One DMA block structure for read buffer
 *
 * driver data size:
 * 1 DMA config structure = 32
 * 3 DMA block structures = 3 * 28 = 84
 * subtotal = 116 bytes
 * TX buffer of DT specified bytes. Min = 4. Default = 16.
 * subtotal = 132 bytes.
 *
 */

/* Number of struct dma_block_config (32 byte in size)
 * We no longer care about size of driver data.
 * The I2C-NL HW is EXTREMELY sensitive about transfer configuration and timing.
 * Give it all it wants up front otherwise...
 *
 * Support these APIs from zephyr i2c.h
 * i2c_write - 1 write message with STOP
 * i2c_read - 1 read message with STOP
 * i2c_write_read - 2 messages: first is write, second is read with RESTART and STOP
 * i2c_burst_write - 2 write messages: second has STOP.
 * i2c_reg_read_byte calls i2c_write_read
 * i2c_reg_write_byte calls i2c_write
 * i2c_reg_update_byte calls i2c_reg_read_byte followed by i2c_reg_write_byte
 *
 * Our i2c_transfer API must support the above within HW limitations.
 * I2C-NL HW is limited to a full transaction START, optional RPT-START, then STOP.
 *
 * One or two consecutive writes ending with STOP.
 * One read ending with STOP.
 * One write followed by one read with STOP. We inject RPT-START before read.
 *
 * The Zephyr DMA driver has high overhead.
 * A DMA transfer requires an instance of struct dma_config (currently 32 bytes) and
 * one or more struct dma_block_config's as single linked list. (each 28 bytes).
 * I2C-NL hardware makes the DMA configuration be worse than it should be.
 * I2C-NL HW requires the target's I2C start and repeated start address be part of
 * the transmit data stream. Even with double buffering of message data in the driver,
 * the driver does not know the application message sizes. If a message exceeds the driver
 * buffer then the driver must return an error or fallback to non-buffered mode.
 *
 * 1. No driver buffering of message data.
 *    Transmit phase: struct dma_config with direction set for MEMORY_TO_PERIPHERAL
 *       First struct dma_block_config for one byte target address
 *       Second struct dma_block_config for transmit data
 *       Third struct dma_block_config for repeated start one byte target address
 *    Receive phase: struct dma_config with direction set for PERIPHERAL_TO_MEMORY
 *       First struct dma_block_config for data to receive
 *
 * Non-buffered implementation options:
 * Allocate 4 struct dma_block_config's (28 * 4 = 92 bytes). This supports the above
 * I2C API's.
 * Allocate 2 struct dma_config's (32 * 2 = 64 bytes)
 * The I2C-NL HW pauses the transaction when transmit phase is finished and there is data to
 * receive. The ISR handler must reprogram the same DMA channel for PERIPHERAL_TO_MEMORY direction
 * with a single struct dma_block_config for the receive buffer.
 * Since the transmit phase is complete, we can re-use the struct dma_config and one
 * struct dma_block config for the receive phase.
 * Allocate one struct dma_config and three struct dma_block_config's.
 * The driver must store information about the receive phase for the ISR to reprogram the
 * DMA channel.
 *
 */
#define I2C_MEC5_NL_MSG_NODES_MAX 2
#define I2C_MEC5_NL_DMA_BLK_CFGS  3

struct i2c_mec5_msg_node {
	struct i2c_msg *m;
	struct i2c_mec5_msg_node *next;
};

struct i2c_mec5_nl_data {
	const struct i2c_mec5_nl_config *devcfg;
	struct mec_i2c_smb_ctx ctx;
	struct k_sem lock;
	struct k_event events;
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
#if 0
	struct dma_config dma_cfg2;
	struct dma_block_config dma_blks[I2C_MEC5_NL_DMA_BLK_CFGS];
#endif
	uint32_t ev;
	uint32_t clock_freq_hz;
	uint32_t i2c_status;
	uint32_t xfr_tmout;
	uint8_t *xfrbuf;
	uint32_t xfrlen;
	struct i2c_mec5_msg_node mnodes[2];
	struct i2c_mec5_msg_node *pmnode;
	struct i2c_msg *msgs;
	uint8_t num_msgs;
	uint8_t msgidx;
	uint8_t didx;
	uint8_t state;
	uint8_t i2c_addr;
	uint8_t dir;
	uint8_t misc_cfg; /* b[3:0]=port, b[6:4]=rsvd, b[7]=0(CM), 1(TM) */
	volatile uint8_t xfr_flags;
	uint32_t cm_cmd;
#ifdef CONFIG_I2C_CALLBACK
	i2c_callback_t cb;
	void *userdata;
#endif
#ifdef CONFIG_I2C_TARGET
	struct dma_config dma_cfg_tm;
	struct i2c_target_config *target1_cfg;
	struct i2c_target_config *target2_cfg;
	struct i2c_target_config *curr_target;
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

static inline void i2c_mec5_nl_dbg_isr_data_update(struct i2c_mec5_nl_data *data)
{
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	struct mec_i2c_smb_regs *regs = hwctx->base;
	uint32_t idx = data->dbg_isr_idx;

	data->dbg_isr_idx++;

	if (idx < I2C_NL_DEBUG_ISR_ENTRIES) {
		data->dbg_isr_data[idx].isr_idx = idx;
		data->dbg_isr_data[idx].isr_cnt = data->dbg_isr_cnt;
		data->dbg_isr_data[idx].status = data->i2c_status;
		data->dbg_isr_data[idx].config = regs->CONFIG;
		data->dbg_isr_data[idx].cm_cmd = regs->CM_CMD;
		data->dbg_isr_data[idx].extlen = regs->EXTLEN;
		data->dbg_isr_data[idx].ev = data->ev;
	}
}

#define I2C_NL_DEBUG_ISR_INIT(d)                i2c_mec5_nl_dbg_isr_init(d)
#define I2C_NL_DEBUG_ISR_COUNT_UPDATE(d)        i2c_mec5_nl_dbg_isr_cnt_update(d)
#define I2C_NL_DEBUG_CM_DMA_ISR_COUNT_UPDATE(d) i2c_mec5_nl_dbg_cm_dma_isr_cnt_update(d)
#define I2C_NL_DEBUG_TM_DMA_ISR_COUNT_UPDATE(d) i2c_mec5_nl_dbg_tm_dma_isr_cnt_update(d)
#define I2C_NL_DEBUG_ISR_DATA_UPDATE(d)         i2c_mec5_nl_dbg_isr_data_update(d)
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

#define I2C_NL_DEBUG_STATE_INIT(d)          i2c_mec5_nl_dbg_state_init((d))
#define I2C_NL_DEBUG_STATE_UPDATE(d, state) i2c_mec5_nl_dbg_state_update((d), (state))
#else
#define I2C_NL_DEBUG_STATE_INIT(d)
#define I2C_NL_DEBUG_STATE_UPDATE(d, state)

#endif /* I2C_NL_DEBUG_STATE */

/* prototypes */
static void i2c_mec5_nl_cm_dma_cb(const struct device *dma_dev, void *user_data, uint32_t channel,
				  int status);

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

#if 0 /* not used for now */
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
#endif

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

	memset(data->mnodes, 0, sizeof(data->mnodes));
	data->pmnode = 0;
	data->xfrlen = 0;
	data->msgs = msgs;
	data->num_msgs = num_msgs;
	data->i2c_addr = (uint8_t)(addr & 0x7fu);
	data->msgidx = 0;
	data->didx = 0;
	data->xfr_flags = 0;
	data->state = I2C_NL_STATE_CLOSED;
}

/* Support these APIs from zephyr i2c.h
 * i2c_write - 1 write message with STOP
 * i2c_read - 1 read message with STOP
 * i2c_write_read - 2 messages: first is write, second is read with RESTART and STOP
 * i2c_burst_write - 2 write messages: second has STOP.
 * i2c_reg_read_byte calls i2c_write_read
 * i2c_reg_write_byte calls i2c_write
 * i2c_reg_update_byte calls i2c_reg_read_byte followed by i2c_reg_write_byte
 * Additional HW limitations:
 * 7-bit I2C addressing only.
 * Total message length of write or read on a transaction (START to STOP) is
 * I2C_MEC5_NL_MAX_XFR_LEN (currently 0xfff8, maximum is 0xfffd).
 * This includes the start and repeated start address (one byte each).
 */

static int req_is_supported(struct i2c_msg *msgs, uint8_t num_msgs)
{
	uint8_t cdir = 0, pdir = 0;

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

		if (m->len > I2C_MEC5_NL_MAX_XFR_LEN) {
			LOG_ERR("I2C msg[%u].len = %u exceeds HW limit %u", n, m->len,
				I2C_MEC5_NL_MAX_XFR_LEN);
			return false;
		}

		if (m->flags & I2C_MSG_ADDR_10_BITS) {
			LOG_ERR("I2C msg[%u] has 10-bit address flag not supported by HW", n);
			return false;
		}

		if (m->flags & I2C_MSG_STOP) {
			continue;
		}

		if (m->flags & I2C_MSG_READ) {
			/* HW requires all reads be terminated with I2C STOP */
			m->flags |= I2C_MSG_STOP;
			cdir = I2C_NL_DIR_RD;
		} else {
			cdir = I2C_NL_DIR_WR;
		}

		/* We only support two message sequences of the form:
		 * Two writes, last write with STOP.
		 * One write followed by one read with STOP.
		 */
		if (pdir == I2C_NL_DIR_WR) {
			if (cdir == I2C_MSG_READ) {
				m->flags |= I2C_MSG_RESTART;
			}
			m->flags |= I2C_MSG_STOP;
			cdir = I2C_NL_DIR_NONE;
		}

		pdir = cdir;
	}

	return true;
}

static bool have_msgs(const struct device *dev)
{
	struct i2c_mec5_nl_data *const data = dev->data;

	if (!data->msgs || !data->num_msgs) {
		return false;
	}

	if (data->msgidx >= data->num_msgs) {
		return false;
	}

	return true;
}

static void cfg_dma_block(const struct device *dev, struct dma_block_config *dblk,
			  struct dma_block_config *next, uint8_t *pdata, uint32_t dlen,
			  uint32_t flags)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct mec_i2c_smb_regs *i2c_regs = devcfg->i2c_regs;
	uint8_t dir = (uint8_t)flags & 0x3u;

	if (dir != I2C_NL_DIR_RD) { /* write? */
		dblk->source_address = (uint32_t)pdata;
		dblk->dest_address = (uint32_t)&i2c_regs->CM_TXB;
		dblk->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dblk->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		dblk->source_address = (uint32_t)&i2c_regs->CM_RXB;
		dblk->dest_address = (uint32_t)pdata;
		dblk->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dblk->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}

	dblk->source_gather_interval = 0;
	dblk->dest_scatter_interval = 0;
	dblk->dest_scatter_count = 0;
	dblk->source_gather_count = 0;
	dblk->block_size = dlen;
	dblk->next_block = next;
	dblk->source_gather_en = 0;
	dblk->dest_scatter_en = 0;
	dblk->source_reload_en = 0;
	dblk->dest_reload_en = 0;
	dblk->fifo_mode_control = 0;
	dblk->flow_control_mode = 0;
}

static int cfg_dma_dir(const struct device *dev, struct dma_config *dcfg,
		       struct dma_block_config *head, uint8_t nblocks, uint8_t dir)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	uint32_t dmadir = MEMORY_TO_PERIPHERAL;

	if (!dev || !dcfg || !head || !nblocks) {
		return -EINVAL;
	}

	if (dir == I2C_NL_DIR_RD) {
		dmadir = PERIPHERAL_TO_MEMORY;
	}

	dcfg->dma_slot = devcfg->cm_dma_trigsrc;
	dcfg->channel_direction = dmadir;
	dcfg->complete_callback_en = 0; /* callback at end of transfer list completion */
	dcfg->error_callback_dis = 0;   /* allow callback on DMA error */
	dcfg->source_handshake = 0;     /* HW */
	dcfg->dest_handshake = 0;       /* HW */
	dcfg->channel_priority = 0;     /* N/A */
	dcfg->source_chaining_en = 0;   /* N/A */
	dcfg->dest_chaining_en = 0;     /* N/A */
	dcfg->linked_channel = 0;       /* N/A */
	dcfg->cyclic = 0;               /* N/A */
	dcfg->source_data_size = 1u;    /* source data width in bytes */
	dcfg->dest_data_size = 1u;      /* dest data width in bytes */
	dcfg->source_burst_length = 0;  /* N/A */
	dcfg->dest_burst_length = 0;    /* N/A */
	dcfg->block_count = nblocks;
	dcfg->head_block = head;
	dcfg->user_data = (void *)dev;
	dcfg->dma_callback = i2c_mec5_nl_cm_dma_cb;

	return dma_config(devcfg->dma_dev_cm, devcfg->cm_dma_chan, dcfg);
}

#if 0
/* configure CM DMA channel for transfer
 * I2C Write: txdata != NULL and rxdata == NULL
 * START wrAddr txdata STOP
 * TX DMA config: Two DMA blocks: wrAddr and txdata
 * No RX DMA config
 *
 * I2C Read: txdata == NULL and rxdata != NULL
 * START rdAddr [rxdata] STOP
 * TX DMA Config: One DMA block: rdAddr
 * RX DMA Config: One DMA block: rxdata
 *
 * I2C Write-Read: txdata != NULL and rxdata != NULL
 * START wrAddr txdata RPT-START rdAddr [rxdata] STOP
 * TX DMA Config: Three DMA blocks: wrAddr, txdata, rdAddr
 * RX DMA Config: One DMA block: rxdata
 */
static int config_dma_i2c_wr(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	int ret = 0;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xA0u);

	data->xfrbuf[0] = (data->i2c_addr << 1);
	data->total_tx_len = data->ntxdata + 1u;

	cfg_dma_block(dev, &data->dma_blks[1], NULL, data->txdata, data->ntxdata, 0);
	cfg_dma_block(dev, &data->dma_blks[0], &data->dma_blks[1], data->xfrbuf, 1u, 0);

	ret = cfg_dma_dir(dev, &data->dma_cfg_tx, &data->dma_blks[0], 2u, I2C_NL_DIR_WR);
	if (ret) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0xA1u);
		return ret;
	}

	return dma_config(devcfg->dma_dev_cm, devcfg->cm_dma_chan, &data->dma_cfg_tx);
}

static int config_dma_i2c_rd(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	int ret = 0;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xA2u);

	data->xfrbuf[0] = (data->i2c_addr << 1) | BIT(0);
	data->total_tx_len = 1;

	/* transmit phase: target read address */
	cfg_dma_block(dev, &data->dma_blks[0], NULL, data->xfrbuf, 1u, 0);
	ret = cfg_dma_dir(dev, &data->dma_cfg_tx, &data->dma_blks[0], 1u, I2C_NL_DIR_WR);
	if (ret) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0xA3u);
		return ret;
	}

	/* receive phase: application buffer. ISR loads this config on PAUSE event */
	cfg_dma_block(dev, &data->dma_blks[1], NULL, data->rxdata, data->nrxdata, BIT(0));
	ret = cfg_dma_dir(dev, &data->dma_cfg_rx, &data->dma_blks[1], 1u, I2C_NL_DIR_RD);
	if (ret) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0xA4u);
		return ret;
	}

	return dma_config(devcfg->dma_dev_cm, devcfg->cm_dma_chan, &data->dma_cfg_tx);
}

/* I2C Combined Write-Read
 * TX phase: 3 DMA blocks
 *   block[0] = target I2C write address
 *   block[1] = tx write data
 *   block[2] = target I2C read address
 * RX phase: 1 DMA block
 *   block[3] = rx read data
 */
static int config_dma_i2c_combined(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	int ret = 0;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xA5u);

	data->xfrbuf[0] = (data->i2c_addr << 1);
	data->xfrbuf[1] = data->xfrbuf[0] | BIT(0);
	data->total_tx_len = data->ntxdata + 2u;

	/* TX phase */
	cfg_dma_block(dev, &data->dma_blks[0], &data->dma_blks[1], data->xfrbuf, 1u, 0);
	cfg_dma_block(dev, &data->dma_blks[1], &data->dma_blks[2], data->txdata, data->ntxdata, 0);
	cfg_dma_block(dev, &data->dma_blks[2], NULL, &data->xfrbuf[1], 1u, 0);
	ret = cfg_dma_dir(dev, &data->dma_cfg_tx, &data->dma_blks[0], 3u, I2C_NL_DIR_WR);
	if (ret) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0xA6u);
		return ret;
	}

	/* RX phase */
	cfg_dma_block(dev, &data->dma_blks[3], NULL, data->rxdata, data->nrxdata, BIT(0));
	ret = cfg_dma_dir(dev, &data->dma_cfg_rx, &data->dma_blks[3], 1u, I2C_NL_DIR_RD);
	if (ret) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0xA7u);
		return ret;
	}

	return dma_config(devcfg->dma_dev_cm, devcfg->cm_dma_chan, &data->dma_cfg_tx);
}

static int config_dma(const struct device *dev)
{
	struct i2c_mec5_nl_data *const data = dev->data;
	int ret = 0;

	if (data->txdata && !data->rxdata) {
		ret = config_dma_i2c_wr(dev);
	} else if (!data->txdata && data->rxdata) {
		ret = config_dma_i2c_rd(dev);
	} else { /* I2C combined write-read */
		ret = config_dma_i2c_combined(dev);
	}

	return ret;
}

static int process_i2c_msgs(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_regs *i2c_regs = devcfg->i2c_regs;
	uint32_t cm_flags = (MEC_I2C_NL_FLAG_START | MEC_I2C_NL_FLAG_STOP |
			     MEC_I2C_NL_FLAG_FLUSH_BUF | MEC_I2C_NL_FLAG_CM_DONE_IEN);
	int ret = 0;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x90u);

	data->txdata = NULL;
	data->ntxdata = 0;
	data->rxdata = NULL;
	data->nrxdata = 0;
	data->total_tx_len = 0;

	while (data->msgidx < data->num_msgs) {
		struct i2c_msg *m = &data->msgs[data->msgidx];

		if (m->flags & I2C_MSG_READ) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x91u);
			data->rxdata = m->buf;
			data->nrxdata = m->len;
		} else {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x92u);
			data->txdata = m->buf;
			data->ntxdata = m->len;
		}

		if (m->flags & I2C_MSG_STOP) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x93u);
			ret = config_dma(dev); /* config DMA */
			if (ret) {
				I2C_NL_DEBUG_STATE_UPDATE(data, 0x94u);
				LOG_ERR("DMA config error (%d)", ret);
				return ret;
			}

			if (data->txdata && data->rxdata) {
				I2C_NL_DEBUG_STATE_UPDATE(data, 0x95u);
				cm_flags |= MEC_I2C_NL_FLAG_RPT_START;
			}

			I2C_NL_DEBUG_STATE_UPDATE(data, 0x96u);

			ret = dma_start(devcfg->dma_dev_cm, devcfg->cm_dma_chan);
			if (ret) {
				I2C_NL_DEBUG_STATE_UPDATE(data, 0x97u);
				LOG_ERR("DMA start error (%d)", ret);
				return ret;
			}

			I2C_NL_DEBUG_STATE_UPDATE(data, 0x98u);

			data->cm_cmd = 0;
			ret = mec_hal_i2c_nl_cm_start(i2c_regs, data->total_tx_len, data->nrxdata,
						      cm_flags, &data->cm_cmd);
			if (ret) {
				I2C_NL_DEBUG_STATE_UPDATE(data, 0x99u);
				LOG_ERR("I2C-NL CM start error (%d)", ret);
				return ret;
			}

			I2C_NL_DEBUG_STATE_UPDATE(data, 0x9Au);
			data->msgidx++; /* point to next message */
			break;
		}

		I2C_NL_DEBUG_STATE_UPDATE(data, 0x9Bu);
		data->msgidx++;
	} /* end while */

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x9Fu);

	return ret;
}
#endif /* 0 */

/* All I2C complete transfers begin with START and a transmit phase.
 * We configure the I2C-NL and DMA channel to perform the transmit phase of the transfer.
 * Transmit phase consists of:
 * START transmit target write address [ACK from target] tx data [ACK per byte]
 * optional RPT-START transmit target read address [ACK] clock stretch.
 * I2C-NL HW pauses at the end of the transmit phase and signals an interrupt.
 * The I2C-NL ISR will reconfigure DMA for receive phase and set a HW bit to continue
 * the transfer. After I2C-NL HW generates STOP and releases the I2C lines, the IDLE
 * interrupt will fire if the lines stay idle. The ISR will check if more messages
 * are available and start the next transaction.
 */
static int initiate_xfr(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct mec_i2c_smb_regs *i2c_regs = devcfg->i2c_regs;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct i2c_mec5_msg_node *pn = data->mnodes;
	uint32_t cm_flags = (MEC_I2C_NL_FLAG_START | MEC_I2C_NL_FLAG_STOP |
			     MEC_I2C_NL_FLAG_FLUSH_BUF | MEC_I2C_NL_FLAG_CM_DONE_IEN);
	uint32_t dflags = 0u, ntx = 0, nrx = 0;
	struct i2c_msg *m = NULL;
	int ret = 0;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x70u);

	memset(data->mnodes, 0, sizeof(data->mnodes));
	data->pmnode = NULL;

	if (data->msgidx >= data->num_msgs) {
		return 0; /* TODO do we need a unique return value for this case? */
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x71u);
	m = &data->msgs[data->msgidx++];
	data->pmnode = pn; /* set head */
	pn->m = m;

	/* Start phase: transmit target address */
	ntx = 1u;
	data->xfrbuf[0] = ((data->i2c_addr & 0x7fu) << 1);
	if (m->flags & I2C_MSG_READ) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x72u);
		dflags = 1u;
		data->xfrbuf[0] |= BIT(0);
		nrx = m->len;
	} else {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x73u);
		ntx += m->len;
	}

	/* transfer has two messages? */
	if ((data->msgidx < data->num_msgs) && !(m->flags & I2C_MSG_STOP)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x74u);
		m = &data->msgs[data->msgidx++];
		pn->next = pn + 1u;
		++pn;
		pn->m = m;
		if (m->len) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x75u);
			if (m->flags & I2C_MSG_READ) {
				I2C_NL_DEBUG_STATE_UPDATE(data, 0x76u);
				if (ntx > 1u) { /* write-read requires RPT-START */
					I2C_NL_DEBUG_STATE_UPDATE(data, 0x77u);
					cm_flags |= MEC_I2C_NL_FLAG_RPT_START;
					data->xfrbuf[1] = data->xfrbuf[0] | BIT(0);
				}
				nrx += m->len;
			} else {
				I2C_NL_DEBUG_STATE_UPDATE(data, 0x78u);
				ntx += m->len;
			}
		}
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x79u);
	/* configure and start DMA and I2C-NL for START transmit target address */
	cfg_dma_block(dev, &data->dma_blk_cfg, NULL, data->xfrbuf, 1u, dflags);
	ret = cfg_dma_dir(dev, &data->dma_cfg, &data->dma_blk_cfg, 1u, I2C_NL_DIR_WR);
	if (ret) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x7Au);
		LOG_ERR("I2C-NL CM initiate: dma config error (%d)", ret);
		return ret;
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x7Bu);
	dma_start(devcfg->dma_dev_cm, devcfg->cm_dma_chan);
	I2C_NL_DEBUG_STATE_UPDATE(data, 0x7Cu);

	data->cm_cmd = 0;
	ret = mec_hal_i2c_nl_cm_start(i2c_regs, ntx, nrx, cm_flags, &data->cm_cmd);
	if (ret) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x7Du);
		LOG_ERR("I2C-NL CM start error (%d)", ret);
		return ret;
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x7Fu);

	return 0;
}

/* ==== Interrupt handler section ==== */

/* ---- Supported I2C transfer flows ----
 * NOTE: In all these flows, the total number of bytes written to and read from
 * the target I2C device have been programmed into I2C-NL before the transaction
 * is begun. Due to asynchronous nature of DMA and I2C-NL code cannot modify I2C-NL
 * command register while I2C-NL is clocking or generating START/STOP.
 * 1. One I2C write message
 *    START transmit target write address and receive (n)ACK bit on 9th clock.
 *    DMA channel interrupt on address written to I2C-NL TX buffer register.
 *        DMA callback will config & start DMA for TX data.
 *    After 9 I2C clocks I2C-NL has (n)ACK from target.
 *        ACK - no interrupt from I2C-NL
 *        NACK - I2C-NL interrupt with CM mode NAK status bit set
 *	      Driver sets NAK event flag and enables IDLE interrupt
 *            It should shut down DMA.
 *    If ACK, DMA writes data to I2C-NL
 *        if NAK of data I2C-NL will fire Done interrupt
 *        When DMA transferred all TX data it fires an interrupt
 *        After last byte is clocked out and (n)ACK received, I2C-NL fires interrupt.
 *
 * 2. Two I2C write messages
 *     Similar to 1 except DMA callback looks at data->pmnode->next, sees it is not NULL
 *     and reloads DMA with buffer address and length of second message. During this time
 *     I2C-NL HW will clock stretch until DMA supplies the next data byte.
 *
 * 3. One I2C read message
 *     START transmit targe read address and receive (n)ACK bit on 9th clock.
 *     DMA channel interrupt on address written to I2C-NL TX buffer register.
 *       DMA callback reconfigures DMA for PERIPH_TO_MEM direction to RX buffer.
 *     I2C-NL clocks out target address byte, decrements its write count to 0,
 *     and clocks in (n)ACK on 9th clock.
 *     If target ACK
 *       I2C-NL clears PROC bit and set CM_DONE status (interrupt)
 *       ISR enables IDLE interrupt
 *       ISR sets PROC=1 to unpause I2C-NL HW.
 *       !!! DMA must have been reprogrammed and started
 *       !!! If DMA interrupt is low HW priority then I2C-NL will continue
 *       !!! to clock stretch until DMA interrupt fires and callback reconfigures
 *       !!! and starts DMA channel. This driver checks DMA channel interrupt priority
 *       !!! and BUILD_ASSERT's if DMA channel has lower priority than I2C-NL.
 *     else
 *       I2C-NL sets NAK error and CM_DONE status (interrupt)
 *       ISR enables IDLE interrupt posts error status
 *     I2C-NL continues triggers DMA to read data from its RX buffer and write it
 *     to memory. Each DMA read of I2C-NL RX buffer register triggers I2C-NL to clock
 *     in the next byte until I2C-NL read count reaches 1. At this point I2C-NL prepares
 *     to generate STOP after ACK's the last byte it receives.  When I2C-NL read count
 *     reaches 0 it clears PROC and RUN bits and sets CM_DONE status (interrupt).
 *     !!! DMA is NOT finished. The last data byte received is still being transferred
 *     !!! from I2C-NL to DMA and then written to memory. For read driver must wait on
 *     !!! both I2C-NL all done and CM DMA done event bits.
 *
 * 4. Two I2C read messages
 *    Similar to 3. The difference is when DMA finishes reading data received from I2C-NL
 *    for first data buffer the DMA signals DONE and fires its interrupt. I2C-NL read count
 *    is still > 1 and DMA ending causes I2C-NL to clock stretch. DMA ISR callback reloads
 *    DMA for second receive buffer, starts DMA channel and exits. I2C-NL starts clocking
 *    when DMA channel begins reading data.
 *
 * 5. Two messages: I2C combined write-read.  More details.
 *    I2C-NL transfer intiated
 *    I2C-NL requests byte (target address) from DMA
 *    DMA writes data byte to I2C-NL TX buffer register and sets its DONE status (interrupt)
 *        DMA ISR callback reloads DMA channel with message data to transmit
 *    Asynchronously,
 *    I2C-NL checks SCL/SDA lines if both high withing sampling window it generates START
 *    I2C-NL clocks out target write address (8-bits) and samples SDA on 9th clock.
 *    Loop while DMA channel has data
 *    	target ACK:
 *        I2C-NL asserts data request to DMA channel
 *        DMA transfers byte from memory buffer to I2C-NL TX buffer register
 *        I2C-NL clocks out data and captures target (n)ACK on 9th clock.
 *      target NACK:
 *        I2C-NL asserts terminate signal to DMA channel
 *        I2C-NL sets clears PROC and RUN bits and sets NAK and DONE status
 *        I2C-NL ISR post event flags for DONE and NAK
 *        exit ISR
 *    end loop
 *    DMA fire interrupt and invokes callback
 *    Asychronously, I2C-NL is clocking out last byte and checking (n)ACK.
 *    DMA callback will reprogram DMA for receive to RX buffer.
 *    DMA waits for I2C-NL to signal it has data to read.
 *    When I2C-NL finishes last transmit by and checks (n)ACK it decrements
 *    its write count to 1.
 *    I2C-NL detects direction change by write count == 0 and read count != 0.
 *    I2C-NL clears PROC bit in its command register and sets DONE and NAK status
 *    in its completion register. I2C-NL generates an interrupt.
 *    I2C-NL ISR checks status and command register
 *    if NAK or other error, shutdown HW, set error event flags and exit
 *    No error, DONE and command PROC:RUN=01b indicates write to read direction
 *    change. In this driver design, the DMA channel interrupt callback has reconfigured
 *    and started the DMA channel to read data from I2C-NL. The I2C-NL ISR only needs
 *    to set command PROC:RUN=11b to unpause I2C-NL and continue the transaction.
 *    Loop while read count > 0
 *	I2C-NL clocks in byte and generates ACK on 9th clock
 *      I2C-NL moves byte from data register to RX buffer register, triggers DMA, and
 *      if read count > 1 starts clocking in the next byte.
 *      DMA reads byte from I2C-NL RX buffer register and transfer the byte to memory.
 *      DMA increments its memory address and compares it to memory end address. If matched
 *      stop accepting data requests and generate interrupt.
 *    end loop
 *    I2C-NL read count == 0
 *    I2C-NL begins generating I2C STOP (takes about 1/2 I2C clock)
 *    DMA had read last data from I2C-NL and is writing it to memory.
 *    After STOP generation is done, I2C-NL will delay 1/2 to 1 I2C clock before releasing
 *    SCL and SDA lines. After releasing the lines, I2C-NL samples the lines and if both are
 *    high for the whole sampling window, it will sets its IDLE status. IDLE interrupt will
 *    be generated if enabled.
 *    NOTE: if target device or an external I2C controller starts driving the lines during
 *    STOP or IDLE sampling phases, this controller may not signal IDLE. It may signal bus
 *    error. This means the driver does not have a reliable way to know the transaction is
 *    complete.
 *    The driver knows DMA is done. It also knows read count reached 0.
 *    If the driver returns to the application or start processing another message
 *    before I2C-NL HW has completed STOP and wait for idle then the the new transaction
 *    may fail.
 */
static void post_events(const struct device *dev, uint32_t kevents, uint32_t flags)
{
	struct i2c_mec5_nl_data *const data = dev->data;

	data->ev = kevents;
	k_event_post(&data->events, kevents);

#ifdef CONFIG_I2C_CALLBACK
	data->state = I2C_NL_STATE_CLOSED;
	k_sem_give(&data->lock);

	if ((flags & BIT(0)) && data->cb) {
		int result = 0;

		if (kevents & BIT(I2C_NL_KEV_CM_NAK_POS)) {
			result = -EFAULT;
		} else if (kevents & (BIT(I2C_NL_KEV_BERR_POS) | BIT(I2C_NL_KEV_LAB_ERR_POS))) {
			result = -EIO;
		}

		data->cb(dev, result, data->userdata);
	}
#endif
}

static void i2c_mec5_nl_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	const struct i2c_mec5_nl_config *const devcfg = dev->config;
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_regs *i2c_regs = devcfg->i2c_regs;
	struct mec_i2c_smb_ctx *hwctx = &data->ctx;
	uint32_t cfg = 0, cmd = 0, kevents = 0, status = 0;

	*(volatile uint32_t *)0x40081150u = 0x00240u;

	I2C_NL_DEBUG_ISR_COUNT_UPDATE(data);
	I2C_NL_DEBUG_ISR_DATA_UPDATE(data);

	/* get I2C HW status and clear it */
	data->i2c_status = i2c_regs->COMPL & 0xffffff00u;
	data->i2c_status |= (i2c_regs->STATUS & 0xffu);
	status = data->i2c_status;
	cmd = i2c_regs->CM_CMD;
	cfg = i2c_regs->CONFIG;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0x80u);

	i2c_regs->COMPL = 0xffffff00u;
	mec_hal_i2c_smb_girq_status_clr(hwctx);

	if ((cfg & BIT(MEC_I2C_SMB_CONFIG_ENI_IDLE_Pos)) &&
	    (status & BIT(MEC_I2C_SMB_COMPL_IDLE_Pos))) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x81u);
		kevents |= BIT(I2C_NL_KEV_IDLE_POS);
	}

	if (status & BIT(MEC_I2C_SMB_COMPL_BUSERR_Pos)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x82u);
		i2c_regs->CONFIG &= 0x00fffffful;
		kevents |= BIT(I2C_NL_KEV_BERR_POS) | BIT(I2C_NL_KEV_ALL_DONE_POS);
		post_events(dev, kevents, BIT(0));
		/* return; */
		goto i2c_mec5_nl_isr_exit;
	}

	if (status & BIT(MEC_I2C_SMB_COMPL_LABSTS_Pos)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x83u);
		i2c_regs->CONFIG &= 0x00fffffful;
		kevents |= BIT(I2C_NL_KEV_LAB_ERR_POS) | BIT(I2C_NL_KEV_ALL_DONE_POS);
		post_events(dev, kevents, BIT(0));
		/* return; */
		goto i2c_mec5_nl_isr_exit;
	}

	if (status & BIT(MEC_I2C_SMB_COMPL_CM_DONE_Pos)) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0x84u);
		switch (cmd & 0x3u) {
		case 0:
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x85u);
			kevents |= BIT(I2C_NL_KEV_ALL_DONE_POS);
			if (status & BIT(MEC_I2C_SMB_COMPL_CM_NAKX_Pos)) {
				I2C_NL_DEBUG_STATE_UPDATE(data, 0x86u);
				i2c_regs->CONFIG &= 0x00fffffful;
				kevents |= BIT(I2C_NL_KEV_CM_NAK_POS);
				post_events(dev, kevents, BIT(0));
				/* return; */
				goto i2c_mec5_nl_isr_exit;
			} else {
				i2c_regs->CONFIG |= BIT(MEC_I2C_SMB_CONFIG_ENI_IDLE_Pos);
			}
			break;
		case 1: /* Pause for write to read direction change */
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x87u);
			/* DMA callback has reconfigured DMA channel for read */
			i2c_regs->CM_CMD |= BIT(MEC_I2C_SMB_CM_CMD_PROCEED_Pos);
			/* return; */
			goto i2c_mec5_nl_isr_exit;
		default:
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x88u);
			break;
		}
	}

	if (kevents & BIT(I2C_NL_KEV_IDLE_POS)) {
		if (have_msgs(dev)) {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x89u);
			/* TODO process_i2c_msgs(dev); */
		} else {
			I2C_NL_DEBUG_STATE_UPDATE(data, 0x8Au);
			i2c_regs->CONFIG &= 0x00fffffful;
			kevents |= BIT(I2C_NL_KEV_ALL_DONE_POS);
			data->ev = kevents;
			post_events(dev, kevents, BIT(0));
		}
	}

i2c_mec5_nl_isr_exit:
	I2C_NL_DEBUG_STATE_UPDATE(data, 0x8Fu);
	*(volatile uint32_t *)0x40081150u = 0x10240u;
}

/* I2C Controller-Mode DMA channel callback */
#if 0
static void i2c_mec5_nl_cm_dma_cb(const struct device *dma_dev, void *user_data, uint32_t channel,
				  int status)
{
	struct i2c_mec5_nl_data *data = (struct i2c_mec5_nl_data *)user_data;
	uint32_t dma_events = BIT(I2C_NL_KEV_DMA_CM_DONE_POS);

	if (status != 0) {
		dma_events |= BIT(I2C_NL_KEV_DMA_CM_ERR_POS);
	}

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xD1u);
	k_event_post(&data->events, dma_events);
}
#else
/* TODO refactor to reduce code size. Probably need index or flags in data
 * Another complication: when I2C-NL wrcnt decrements to 0 on last byte transmitted,
 * the HW clears PROCEED bit and sets CM_DONE status generating an interrupt.
 * We have multiple scenarios to handle:
 * One I2C write messages
 *   START + wrAddress transmit triggers CM DMA interrupt
 *     CM DMA callback reloads DMA for TX data
 *   CM DMA finishes data phase and triggers interrupt invoking callback
 *     CM DMA callback has nothing to do. Should it set DMA Done event flag?
 *     ALTERNATIVE we only enable error callback.
 *   One I2C byte time later I2C-NL gets ACK from target on last byte -> Interrupt
 *   I2C-NL ISR sees wrcnt==0, PROC=RUN=0. Enable IDLE interrupt and exit
 * -----------------------------------------
 * One I2C read message
 *  START + rdAddress transmit triggers CM DMA interrupt
 *   CM DMA callback should do nothing.
 *   ALTERNATIVE we only enable error callback.
 *  After (n)ACK by target of rdAddress I2C-NL fires CM_DONE with PROC=0, RUN=1
 *    I2C-NL ISR configures CM DMA for PERIPH_TO_MEM of read data
 *  I2C-NL finishes clocking in last data byte and signals DMA.
 *  We are not sure when I2C-NL will generate CM_DONE interrupt with PROC=RUN=0.
 *  CM DMA channel takes several AHB clocks read last data byte from I2C-NL and store in memory.
 *    Do we want DMA callback except on error?
 *
 */
static void i2c_mec5_nl_cm_dma_cb(const struct device *dma_dev, void *user_data, uint32_t channel,
				  int status)
{
	const struct device *i2c_dev = (struct device *)user_data;
	struct i2c_mec5_nl_data *data = i2c_dev->data;
	uint32_t dma_events = BIT(I2C_NL_KEV_DMA_CM_DONE_POS);
	struct i2c_msg *m = NULL;

	I2C_NL_DEBUG_STATE_UPDATE(data, 0xD0u);

	if (status != 0) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0xD1u);
		dma_events |= BIT(I2C_NL_KEV_DMA_CM_ERR_POS);
		k_event_post(&data->events, dma_events);
		return;
	}

	if (!data->pmnode) {
		I2C_NL_DEBUG_STATE_UPDATE(data, 0xD2u);
		k_event_post(&data->events, dma_events);
		return;
	}

	m = data->pmnode->m;
	if (m->flags & I2C_MSG_READ) {
		cfg_dma_block(i2c_dev, &data->dma_blk_cfg, NULL, m->buf, m->len, I2C_NL_DIR_RD);
		cfg_dma_dir(i2c_dev, &data->dma_cfg, &data->dma_blk_cfg, 1u, I2C_NL_DIR_RD);
	} else {
		cfg_dma_block(i2c_dev, &data->dma_blk_cfg, NULL, m->buf, m->len, I2C_NL_DIR_WR);
		cfg_dma_dir(i2c_dev, &data->dma_cfg, &data->dma_blk_cfg, 1u, I2C_NL_DIR_WR);
	}

	data->pmnode = data->pmnode->next;

	dma_start(dma_dev, channel);

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

static int i2c_mec5_nl_xfr(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			   uint16_t addr, bool async, i2c_callback_t cb, void *userdata)
{
	struct i2c_mec5_nl_data *const data = dev->data;
	/*	struct mec_i2c_smb_ctx *phwctx = &data->ctx; */
	k_timeout_t tmout = K_FOREVER; /* DEBUG TODO K_MSEC(5000); */
	uint32_t events = 0, ev_wait_msk = 0;
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

#ifdef CONFIG_I2C_CALLBACK
	data->cb = cb;
	data->userdata = userdata;

	if (async) {
		data->xfr_flags |= BIT(I2C_NL_XFR_FLAG_ASYNC_POS);
	}
#endif

	/* TODO check HW condition
	 * If HW in bad state I2C.STATUS NBB==0 or BER==1 etc.
	 * try reset.
	 */
#if 0
	/* trigger IDLE interrupt (edge on enable) */
	mec_hal_i2c_smb_idle_intr_enable(phwctx, 0);
	mec_hal_i2c_smb_idle_status_clr(phwctx);
	mec_hal_i2c_smb_girq_status_clr(phwctx);
	mec_hal_i2c_smb_idle_intr_enable(phwctx, 1);
#else
	initiate_xfr(dev);
#endif
	if (data->xfr_flags & BIT(I2C_NL_XFR_FLAG_ASYNC_POS)) {
		/* return keeping lock */
		return 0;
	}

	ev_wait_msk = BIT(I2C_NL_KEV_ALL_DONE_POS); /* all done */
	events = k_event_wait(&data->events, ev_wait_msk, false, tmout);
	if (events == 0) {
		ret = -ETIMEDOUT;
	} else {
		if (events & (BIT(I2C_NL_KEV_BERR_POS) | BIT(I2C_NL_KEV_LAB_ERR_POS) |
			      BIT(I2C_NL_KEV_CM_NAK_POS))) {
			ret = -EIO;
		}
	}

	data->state = I2C_NL_STATE_CLOSED;
	k_sem_give(&data->lock);

	return ret;
}

static int i2c_mec5_nl_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				uint16_t addr)
{
	return i2c_mec5_nl_xfr(dev, msgs, num_msgs, addr, false, NULL, NULL);
}

#ifdef CONFIG_I2C_CALLBACK
static int i2c_mec5_nl_transfer_cb(const struct device *dev, struct i2c_msg *msgs, uint8_t nmsgs,
				   uint16_t addr, i2c_callback_t cb, void *userdata)
{
	return i2c_mec5_nl_xfr(dev, msgs, nmsgs, addr, true, cb, userdata);
}
#endif /* CONFIG_I2C_CALLBACK */

static int i2c_mec5_nl_recover(const struct device *dev)
{
	struct i2c_mec5_nl_data *const data = dev->data;
	struct mec_i2c_smb_ctx *phwctx = &data->ctx;
	uint32_t i2c_config = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;
	int ret = 0;
	uint8_t cnt = 0, pins = 0;

	k_sem_take(&data->lock, K_FOREVER); /* decrements count */
	k_event_clear(&data->events, UINT32_MAX);

	ret = i2c_mec5_nl_configure(dev, i2c_config);
	if (ret == MEC_RET_OK) {
		ret = 0;
		goto recover_exit;
	}

	mec_hal_i2c_smb_bbctrl(phwctx, 1u, 0);
	pins = mec_hal_i2c_smb_bbctrl_pin_states(phwctx);
	while ((pins & BIT(MEC_I2C_BB_SCL_POS)) == 0) { /* SCL stuck low? */
		cnt++;
		if (cnt > I2C_MEC5_NL_RECOVER_SCL_CNT) {
			mec_hal_i2c_smb_bbctrl(phwctx, 0, 0);
			ret = -EBUSY;
			goto recover_exit;
		}
		k_busy_wait(I2C_MEC5_NL_RECOVER_SCL_DLY_US);
		pins = mec_hal_i2c_smb_bbctrl_pin_states(phwctx);
	}

	cnt = 0;
	while ((pins & BIT(MEC_I2C_BB_SDA_POS)) == 0) { /* SDA stuck low? */
		cnt++;
		if (cnt > I2C_MEC5_NL_RECOVER_SDA_CNT) {
			mec_hal_i2c_smb_bbctrl(phwctx, 0, 0);
			ret = -EBUSY;
			goto recover_exit;
		}
		for (uint8_t i = 0; i < 9u; i++) {
			/* drive SCL low */
			mec_hal_i2c_smb_bbctrl(phwctx, 1u, BIT(MEC_I2C_BB_SCL_POS));
			k_busy_wait(I2C_MEC5_NL_RECOVER_SDA_DLY_US);
			/* stop driving SCL low */
			mec_hal_i2c_smb_bbctrl(phwctx, 1u, 0);
			k_busy_wait(I2C_MEC5_NL_RECOVER_SDA_DLY_US);
		}

		pins = mec_hal_i2c_smb_bbctrl_pin_states(phwctx);
	}

	mec_hal_i2c_smb_bbctrl(phwctx, 0, 0);

	ret = i2c_mec5_nl_configure(dev, i2c_config);
	if (ret != MEC_RET_OK) {
		ret = -EIO;
	}

recover_exit:
	k_sem_give(&data->lock);

	return ret;
}

/* I2C-NL supports up to two 7-bit target addresses */
static int i2c_mec5_nl_target_register(const struct device *dev, struct i2c_target_config *cfg)
{
	return -ENOTSUP;
}

static int i2c_mec5_nl_target_unregister(const struct device *dev, struct i2c_target_config *cfg)
{
	return -ENOTSUP;
}

static const struct i2c_driver_api i2c_mec5_nl_driver_api = {
	.configure = i2c_mec5_nl_configure,
	.get_config = i2c_mec5_nl_get_config,
	.transfer = i2c_mec5_nl_transfer,
	.target_register = i2c_mec5_nl_target_register,
	.target_unregister = i2c_mec5_nl_target_unregister,
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = i2c_mec5_nl_transfer_cb,
#endif
	.recover_bus = i2c_mec5_nl_recover,
};
/* end public API */

/* Configure DMA channels used by I2C-NL driver.
 * I2C-NL uses Central DMA controller channels. We must not reset Central DMA
 * unless it has not yet been enabled. NOTE: Target receive where external
 * Controller writes to this device. I2C-NL triggers TM DMA channel to store the
 * target adddress byte followed by data. We configure TM DMA memory buffer to
 * be offset 3 in the drivers sized padded buffer. This allows us to pass an
 * aligned pointer to the application callback since all it wants is the data.
 */
static int i2c_mec5_nl_dma_init(const struct device *dev)
{
	const struct i2c_mec5_nl_config *const devcfg = dev->config;

	if (!device_is_ready(devcfg->dma_dev_cm)) {
		LOG_ERR("DMA driver not ready!");
		return -EIO;
	}

#ifdef CONFIG_I2C_TARGET
	if (!device_is_ready(devcfg->dma_dev_tm)) {
		LOG_ERR("TM DMA channel not ready");
		return -EIO;
	}
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
	}

	return 0;
}

/* Node id of DMA channel */

#define I2C_MEC5_NL_DT_MISC_CFG(i) (uint8_t)(DT_INST_PROP_OR(i, port_sel, 0) & 0x0f)

#define I2C_NL_MEC5_DMA_NODE(i, name) DT_INST_DMAS_CTLR_BY_NAME(i, name)

#define I2C_NL_MEC5_DMA_DEVICE(i, name) DEVICE_DT_GET(I2C_NL_MEC5_DMA_NODE(i, name))

#define I2C_NL_MEC5_DMA_CHAN(i, name) DT_INST_DMAS_CELL_BY_NAME(i, name, channel)

#define I2C_NL_MEC5_DMA_TRIGSRC(i, name) DT_INST_DMAS_CELL_BY_NAME(i, name, trigsrc)

#define I2C_NL_MEC5_DMA_CM_IRQN(i)                                                                 \
	DT_IRQ_BY_IDX(I2C_MEC5_NL_DT_INST_DMA_CTLR(i, cm), I2C_NL_MEC5_DMA_CHAN(i, cm), irq)

#define I2C_NL_MEC5_DMA_CM_IRQ_PRI(i)                                                              \
	DT_IRQ_BY_IDX(I2C_NL_MEC5_DMA_NODE(i, cm), I2C_NL_MEC5_DMA_CHAN(i, cm), priority)

#define I2C_NL_MEC5_CM_CHECK_IRQ_PRI(i)                                                            \
	BUILD_ASSERT((I2C_NL_MEC5_DMA_CM_IRQ_PRI(i) < DT_INST_IRQ(i, priority)),                   \
		     "CM DMA channel ISR priority must be higher than I2C");

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
	.dma_dev_tm = DT_INST_DMAS_CTLR_BY_NAME(i, tm), .tm_rx_buf = i2c_mec5_nl_tm_rx_buf_##i,    \
	.tm_dma_chan = I2C_NL_MEC5_DMA_CHAN(i, 1),                                                 \
	.tm_rx_buf_sz = (uint16_t)I2C_NL_MEC5_TM_RX_BUF_SIZE(i),                                   \
	.tm_dma_trigsrc = I2C_NL_MEC5_DMA_TRIGSRC(i, 1),

#define I2C_NL_MEC5_DMA_TM_IRQN(i)                                                                 \
	DT_IRQ_BY_IDX(I2C_NL_MEC5_DMA_NODE(i, tm), I2C_NL_MEC5_DMA_CHAN(i, tm), irq)

#define I2C_NL_MEC5_DMA_TM_IRQ_PRI(i)                                                              \
	DT_IRQ_BY_IDX(I2C_NL_MEC5_DMA_NODE(i, tm), I2C_NL_MEC5_DMA_CHAN(i, tm), priority)

#define I2C_NL_MEC5_TM_CHECK_IRQ_PRI(i)                                                            \
	BUILD_ASSERT((I2C_NL_MEC5_DMA_TM_IRQ_PRI(i) < DT_INST_IRQ(i, priority)),                   \
		     "TM DMA channel ISR priority must be higher than I2C");

#else
#define I2C_NL_MEC5_TM_RX_BUF(i)
#define I2C_NL_MEC5_TM_RX_BUF_PTR(i)
#define I2C_MEC5_NL_TM_DMA(i)
#define I2C_MEC5_NL_TM_DMA_IRQ_CE(i)
#define I2C_NL_MEC5_TM_CHECK_IRQ_PRI(i)
#endif

#define I2C_MEC5_NL_DEVICE(i)                                                                      \
	I2C_NL_MEC5_CM_CHECK_IRQ_PRI(i)                                                            \
	I2C_NL_MEC5_TM_CHECK_IRQ_PRI(i)                                                            \
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
	}                                                                                          \
	struct i2c_mec5_nl_config i2c_mec5_nl_devcfg_##i = {                                       \
		.i2c_regs = (struct mec_i2c_smb_regs *)DT_INST_REG_ADDR(i),                        \
		.init_pin_wait_us = DT_INST_PROP_OR(i, init_pin_wait, 100),                        \
		.cfg_pin_wait_us = DT_INST_PROP_OR(i, config_pin_wait, 35000),                     \
		.irq_config_func = i2c_mec5_nl_irq_config_func_##i,                                \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i),                                         \
		.dma_dev_cm = I2C_NL_MEC5_DMA_DEVICE(i, cm),                                       \
		.cm_dma_chan = I2C_NL_MEC5_DMA_CHAN(i, cm),                                        \
		.cm_dma_trigsrc = I2C_NL_MEC5_DMA_TRIGSRC(i, cm),                                  \
		.cm_tx_buf_max_sz = (uint16_t)I2C_NL_MEC5_CM_TX_BUF_SIZE(i),                       \
		I2C_MEC5_NL_TM_DMA(i)};                                                            \
	I2C_DEVICE_DT_INST_DEFINE(i, i2c_mec5_nl_init, NULL, &i2c_mec5_nl_data_##i,                \
				  &i2c_mec5_nl_devcfg_##i, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,  \
				  &i2c_mec5_nl_driver_api)

DT_INST_FOREACH_STATUS_OKAY(I2C_MEC5_NL_DEVICE)
