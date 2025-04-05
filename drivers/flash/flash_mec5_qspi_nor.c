/* Copyright (c) 2025 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mec5_qspi_flash_controller

#include <errno.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_mec5, CONFIG_FLASH_LOG_LEVEL);

/* MEC5 HAL */
#include <device_mec5.h>
#include <mec_ecia_api.h>
#include <mec_qspi_api.h>

#define MEC5_FLASH_DRIVER_DEBUG

#define SPI_FLASH_POWER_DOWN              0xb9u
#define SPI_FLASH_POWER_DOWN_REL          0xabu

#define SPI_FLASH_JEDEC_ID_RD             0x9fu
#define SPI_FLASH_SFDP_RD                 0x5au

#define SPI_FLASH_DATA_RD_111             0x03u /* <= 33 MHz */
#define SPI_FLASH_DATA_RD_111_TSCLK8      0x0bu
#define SPI_FLASH_DATA_RD_112_TSCLK8      0x3bu
#define SPI_FLASH_DATA_RD_114_TSCLK8      0x6bu

#define SPI_FLASH_RD_STATUS1              0x05u
#define SPI_FLASH_RD_STATUS2              0x35u
#define SPI_FLASH_RD_STATUS3              0x15u

#define SPI_FLASH_SECTOR_ERASE            0x20u
#define SPI_FLASH_BLK_32KB_ERASE          0x52u
#define SPI_FLASH_BLK_64KB_ERASE          0xd8u
#define SPI_FLASH_CHIP_ERASE              0xc7u

#define SPI_FLASH_WR_EN                   0x06u
#define SPI_FLASH_WR_EN_VOLATILE          0x50u
#define SPI_FLASH_PAGE_PROG               0x02u
#define SPI_FLASH_PAGE_PROG_QUAD          0x32u

#define SPI_FLASH_WR_STATUS1              0x01u
#define SPI_FLASH_WR_STATUS2              0x31u
#define SPI_FLASH_WR_STATUS3              0x11u


/* Device constant configuration parameters */
struct flash_mec5_qspi_ctrl_devcfg {
	struct mec_qspi_regs *regs;
	uint32_t freqhz;
	uint32_t cs_timing;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(void);
	uint8_t chip_sel;
	uint8_t signal_mode;
	uint8_t ctrl_tap;
	uint8_t clock_tap;
};

struct flash_mec5_qspi_devdata {
#ifdef MEC5_FLASH_DRIVER_DEBUG
	volatile uint32_t isr_count;
#endif
	volatile uint32_t qstatus;
	volatile uint32_t xfr_flags;
	struct k_sem lock;
	struct k_sem sync;
	uint8_t txbuf[8];
	uint8_t rxbuf[8];
	struct mec_spi_command cmd;
	struct mec_qspi_context2 ctx2;
};

/*
 * enter_4byte_addr values for method to enter 4-byte instruction mode
 * 0 = no 4-byte mode
 * b[0]=1 issue opcode 0xB7
 * b[1]=1 issue write-enable 0x06, then 0xB7
 * b[2]=1 8-bit volatile extended address reg used for Addr[31:24]
 *        read it with 0xC8. Write it (one byte) with 0xC5.
 * b[3]=1 8-bit volatile bank reg used for Addr[30:24]. MSB(bit[7]) used
 *        to enable/disable 4-byte address mode. More info in JEDEC spec.
 * b[4]=1 A 16-bit non-volatile config reg controls 3/4 byte address mode.
 *        Read it with 0xB5. Bit[0] controls mode(0=3-byte, 1=4-byte),
 *        Write config reg with 0xB1. data length is 2 bytes.
 * b[5]=1 Supports dedicated 4-byte opcode set. Need flash vendor data sheet.
 * b[6]=1 Always operates in 4-byte address mode
 * b[7]=1 Reserved
 */

#define MEC5_QSPI_FLASH_CFG_USE_DUAL 0x01u

struct mec5_qspi_flash_cfg {
	const struct device *ctrl_dev;
	struct flash_parameters fparams;
	uint32_t max_freq;
	uint32_t size_in_bytes;
	uint8_t jedec_id[4];
	uint16_t page_size;
	uint8_t cs;
	uint8_t quad_en_req;
	uint8_t enter_4byte_addr;
	uint8_t flags;
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	struct flash_pages_layout layout;
#endif
};

static int flash_qspi_write_enable(const struct device *flash_dev)
{
	const struct mec5_qspi_flash_cfg *flash_cfg = flash_dev->config;
	const struct device *ctrl_dev = flash_cfg->ctrl_dev;
	const struct flash_mec5_qspi_ctrl_devcfg *ctrl_cfg = ctrl_dev->config;
	struct flash_mec5_qspi_devdata *ctrl_data = ctrl_dev->data;
	struct mec_qspi_regs *regs = ctrl_cfg->regs;
	uint32_t qflags = MEC5_QSPI_ULDMA_FLAG_IEN | MEC5_QSPI_ULDMA_FLAG_START;

	k_sem_reset(&ctrl_data->sync);

	ctrl_data->txbuf[0] = SPI_FLASH_WR_EN;

	/* TODO return values */
	mec_hal_qspi_cs_select(regs, flash_cfg->cs);

	mec_hal_qspi_xfr_fifo_fd(regs, (const uint8_t *)ctrl_data->txbuf, NULL, 1u, qflags);

	/* returns -EAGAIN if timeout is not K_FOREVER and it timed out or
	 * other entity reset the semaphore
	 */
	k_sem_take(&ctrl_data->sync, K_FOREVER); /* TODO timeout */

	return 0;
}

static int flash_qspi_read_status(const struct device *flash_dev, uint8_t id, uint8_t *status)
{
	const struct mec5_qspi_flash_cfg *flash_cfg = flash_dev->config;
	const struct device *ctrl_dev = flash_cfg->ctrl_dev;
	const struct flash_mec5_qspi_ctrl_devcfg *ctrl_cfg = ctrl_dev->config;
	struct flash_mec5_qspi_devdata *ctrl_data = ctrl_dev->data;
	struct mec_qspi_regs *regs = ctrl_cfg->regs;
	uint32_t qflags = MEC5_QSPI_ULDMA_FLAG_IEN | MEC5_QSPI_ULDMA_FLAG_START;

	k_sem_reset(&ctrl_data->sync);

	if (id == 1) {
		ctrl_data->txbuf[0] = SPI_FLASH_RD_STATUS2;
	} else if (id == 2) {
		ctrl_data->txbuf[0] = SPI_FLASH_RD_STATUS3;
	} else {
		ctrl_data->txbuf[0] = SPI_FLASH_RD_STATUS1;
	}

	ctrl_data->rxbuf[0] = 0;
	ctrl_data->rxbuf[1] = 0;

	/* TODO return values */
	mec_hal_qspi_cs_select(regs, flash_cfg->cs);
	mec_hal_qspi_xfr_fifo_fd(regs, (const uint8_t *)ctrl_data->txbuf, ctrl_data->rxbuf,
				 2u, qflags);

	/* returns -EAGAIN if timeout is not K_FOREVER and it timed out or
	 * other entity reset the semaphore
	 */
	k_sem_take(&ctrl_data->sync, K_FOREVER); /* TODO timeout */

	*status = ctrl_data->rxbuf[1];

	return 0;
}

/* ---- public API ---- */
static int flash_mec5_qspi_read(const struct device *flash_dev, off_t ofs, void *dest, size_t len)
{
	const struct mec5_qspi_flash_cfg *flash_cfg = flash_dev->config;
	const struct device *ctrl_dev = flash_cfg->ctrl_dev;
	const struct flash_mec5_qspi_ctrl_devcfg *ctrl_cfg = ctrl_dev->config;
	struct flash_mec5_qspi_devdata *ctrl_data = ctrl_dev->data;
	struct mec_spi_command *cmd = &ctrl_data->cmd;
	struct mec_qspi_context2 *ctx2 = &ctrl_data->ctx2;
	struct mec_qspi_regs *regs = ctrl_cfg->regs;
	uint32_t qflags = MEC_QSPI_STS_XFR_DONE | MEC_QSPI_STS_PROG_ERR | MEC_QSPI_STS_LDMA_RX_ERR;
	int ret = 0;

	if (!dest) {
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	if (ofs < 0 || ((ofs + len) > flash_cfg->size_in_bytes)) {
		LOG_ERR("read error: address or size "
			"exceeds expected values."
			"Addr: 0x%lx size %zu", (long)ofs, len);
		return -EINVAL;
	}

	k_sem_take(&ctrl_data->lock, K_FOREVER); /* decrements count */
	k_sem_reset(&ctrl_data->sync);

	cmd->flags = MEC_SPI_CMD_ENCODE_PINS(1, 1, 4) | MEC_SPI_CMD_ADDR24;
	cmd->opcode = SPI_FLASH_DATA_RD_114_TSCLK8;
	cmd->mode_byte = 0;
	cmd->mode_bits = 0;
	cmd->ts_clocks = 8u;

	if (flash_cfg->flags & MEC5_QSPI_FLASH_CFG_USE_DUAL) {
		cmd->flags = MEC_SPI_CMD_ENCODE_PINS(1, 1, 2) | MEC_SPI_CMD_ADDR24;
		cmd->opcode = SPI_FLASH_DATA_RD_112_TSCLK8;
	}

	/* TODO return values */
	mec_hal_qspi_context2_init_from_regs(ctx2, regs);

	mec_hal_qspi_flash_cmd_setup(ctx2, cmd, (uint32_t)ofs);

	mec_hal_qspi_flash_data_setup(ctx2, cmd, (uint8_t *)dest, len, 0);

	mec_hal_qspi_flash_xfr_load(ctx2);

	mec_hal_qspi_cs_select(regs, flash_cfg->cs);

	mec_hal_qspi_start(regs, qflags); /* TODO Interrupt flags */

	/* returns -EAGAIN if timeout is not K_FOREVER and it timed out or
	 * other entity reset the semaphore
	 */
	k_sem_take(&ctrl_data->sync, K_FOREVER); /* TODO timeout */
	k_sem_give(&ctrl_data->lock);

	return ret;
}

/*
 * Data may be inside or overlap page(s)
 * | page | page | page |
 *    |    data    |
 * Could touch multiple pages.
 * Each page touched requires:
 *   1. issue write enable
 *   2. issue page program: offset + page_data where length of page data determined by
 *                          len, page size, and offset
 *   3. loop issuing read status1 until BUSY bit clears or timeout
 *
 * end_ofs = ofs + len;
 * wrlen = page_size - (ofs % page_size)
 * if successful then ofs += wrlen
 * continue while ofs < end_ofs
 */
static int flash_mec5_qspi_write(const struct device *flash_dev, off_t ofs,
				 const void *data, size_t len)
{
	const struct mec5_qspi_flash_cfg *flash_cfg = flash_dev->config;
	const struct device *ctrl_dev = flash_cfg->ctrl_dev;
	const struct flash_mec5_qspi_ctrl_devcfg *ctrl_cfg = ctrl_dev->config;
	struct flash_mec5_qspi_devdata *ctrl_data = ctrl_dev->data;
	struct mec_spi_command *cmd = &ctrl_data->cmd;
	struct mec_qspi_context2 *ctx2 = &ctrl_data->ctx2;
	struct mec_qspi_regs *regs = ctrl_cfg->regs;
	uint32_t qflags = MEC_QSPI_STS_XFR_DONE | MEC_QSPI_STS_PROG_ERR | MEC_QSPI_STS_LDMA_RX_ERR;
	int ret = 0;

	if (ofs < 0 || ((ofs + len) > flash_cfg->size_in_bytes)) {
		LOG_ERR("write error: address or size exceeds expected values."
			"Addr: 0x%lx size %zu", (long)ofs, len);
		return -EINVAL;
	}

	/* TODO */
	k_sem_take(&ctrl_data->lock, K_FOREVER); /* decrements count */

	/* TODO begin loop */

	ret = flash_qspi_write_enable(flash_dev);
	if (ret) {
		k_sem_give(&ctrl_data->lock);
		return ret;
	}
	k_sem_reset(&ctrl_data->sync);

	cmd->flags = MEC_SPI_CMD_ENCODE_PINS(1, 1, 1) | MEC_SPI_CMD_ADDR24;
	cmd->opcode = SPI_FLASH_PAGE_PROG;
	cmd->mode_byte = 0;
	cmd->mode_bits = 0;
	cmd->ts_clocks = 0;

	/* TODO data crosses page boundary */

	/* TODO return values */
	mec_hal_qspi_context2_init_from_regs(ctx2, regs);

	mec_hal_qspi_flash_cmd_setup(ctx2, cmd, (uint32_t)ofs);

	mec_hal_qspi_flash_data_setup(ctx2, cmd, (uint8_t *)data, len, 0);

	mec_hal_qspi_flash_xfr_load(ctx2);

	mec_hal_qspi_cs_select(regs, flash_cfg->cs);

	mec_hal_qspi_start(regs, qflags); /* TODO Interrupt flags */

	/* returns -EAGAIN if timeout is not K_FOREVER and it timed out or
	 * other entity reset the semaphore
	 */
	k_sem_take(&ctrl_data->sync, K_FOREVER); /* TODO timeout */
	/* TODO end loop */

	k_sem_give(&ctrl_data->lock);

	return 0;
}

#if 0
static int flash_mec5_qspi_erase(const struct device *dev, off_t offset, size_t size)
{
	const struct mec5_qspi_flash_nor_devcfg *devcfg = dev->config;
	struct mec5_qspi_flash_nor_data *devdata = dev->data;
	struct mec_qspi_regs *regs = devcfg->regs;

	LOG_INF("MEC5 QSPI flash erase: offset=0x%0lx size=%u", offset, size);
	LOG_INF("MEC5 QSPI flash erase: devcfg=%p devdata=%p regs=%p", devcfg, devdata, regs);

	return -ENOSYS;
}
#endif

/* Get the size of the flash device.
 * Size is a constant for minimal and device tree build.
 * TODO:
 * If CONFIG_SPI_NOR_SFDP_RUNTIME is enabled then we must add large amount of logic
 * to read SFDP from flash device and extract flash size, etc. from SFDP data.
 */
static int flash_mec5_qspi_get_size(const struct device *flash_dev, uint64_t *size)
{
	const struct mec5_qspi_flash_cfg *flash_cfg = flash_dev->config;

	if (!size) {
		return -EINVAL;
	}

	*size = (uint64_t)flash_cfg->size_in_bytes;

	return 0;
}

#if 0
struct flash_parameters {
	/** Minimal write alignment and size */
	const size_t write_block_size;

	/** @cond INTERNAL_HIDDEN */
	/* User code should call flash_params_get_ functions on flash_parameters
	 * to get capabilities, rather than accessing object contents directly.
	 */
	struct {
		/* Device has no explicit erase, so it either erases on
		 * write or does not require it at all.
		 * This also includes devices that support erase but
		 * do not require it.
		 */
		bool no_explicit_erase: 1;
	} caps;
	/** @endcond */
	/** Value the device is filled in erased areas */
	uint8_t erase_value;
};
#endif /* 0 */

#if 0
static const struct flash_parameters *flash_mec5_qspi_get_params(const struct device *flash_dev)
{
	struct mec5_qspi_flash_cfg *fdev = flash_dev->config;

	return &flash_dev->fparams;
}
#endif

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
/* Return a pointer to an array of flash blocks.
 * For flash devices with uniform page sized return array length (*layout_size) of 1.
 */
static void flash_mec5_qspi_page_layout(const struct device *flash_dev,
					const struct flash_pages_layout **layout,
					size_t *layout_size)
{
	const struct mec5_qspi_flash_cfg *flash_cfg = flash_dev->config;

	if (!layout || !layout_size) {
		return -EINVAL;
	}

	flash_cfg->page_size

	*layout = &flash_cfg->layout;
	*layout_size = 1;

	return -ENOSYS;
}
#endif

#if 0
#if defined(CONFIG_FLASH_JESD216_API)
static int flash_mec5_qspi_read_sfdp(const struct device *dev, off_t offset,
				     void *data, size_t len)
{
	const struct mec5_qspi_flash_nor_devcfg *devcfg = dev->config;
	struct mec5_qspi_flash_nor_data *devdata = dev->data;
	struct mec_qspi_regs *regs = devcfg->regs;

	return -ENOSYS;
}

static int flash_mec5_qspi_read_jedec_id(const struct device *dev, uint8_t *id)
{
	const struct mec5_qspi_flash_nor_devcfg *devcfg = dev->config;
	struct mec5_qspi_flash_nor_data *devdata = dev->data;
	struct mec_qspi_regs *regs = devcfg->regs;

	return -ENOSYS;
}
#endif
/* ---- End public API ---- */
#endif /* 0 */

static void flash_mec5_qspi_isr(const struct device *dev)
{
	const struct flash_mec5_qspi_ctrl_devcfg *devcfg = dev->config;
	struct flash_mec5_qspi_devdata *devdata = dev->data;
	struct mec_qspi_regs *regs = devcfg->regs;

#ifdef MEC5_FLASH_DRIVER_DEBUG
	devdata->isr_count++;
#endif
	devdata->qstatus = regs->STATUS;
	regs->INTR_CTRL = 0;
	regs->STATUS = UINT32_MAX;

	k_sem_give(&devdata->sync);
}

static int flash_mec5_qspi_init(const struct device *flash_ctrl_dev)
{
	const struct flash_mec5_qspi_ctrl_devcfg *devcfg = flash_ctrl_dev->config;
	struct flash_mec5_qspi_devdata *devdata = flash_ctrl_dev->data;
	struct mec_qspi_regs *regs = devcfg->regs;
	int ret = 0;

	devdata->qstatus = 0;

	k_sem_init(&devdata->lock, 1, 1);
	k_sem_init(&devdata->sync, 0, 1);

	ret = pinctrl_apply_state(devcfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}


	ret = mec_hal_qspi_init(regs, devcfg->freqhz, devcfg->signal_mode, MEC_QSPI_IO_QUAD,
				devcfg->chip_sel);
	if (ret != MEC_RET_OK) {
		return -EIO;
	}

	if (devcfg->irq_config_func) {
		devcfg->irq_config_func();
	}

	return 0;
}

static int flash_mec5_init(const struct device *flash_dev)
{
	const struct mec5_qspi_flash_cfg *flash_cfg = flash_dev->config;

	if (!device_is_ready(flash_cfg->ctrl_dev)) {
		return -ENODEV;
	}

	return 0;
}

static DEVICE_API(flash, flash_mec5_qspi_driver_api) = {
	.read = flash_mec5_qspi_read,
	.write = NULL,
	.erase = NULL,
	.get_parameters = NULL,
	.get_size = flash_mec5_qspi_get_size,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_mec5_qspi_page_layout,
#endif
#if defined(CONFIG_FLASH_JESD216_API)
	.sfdp_read = NULL,
	.read_jedec_id = NULL,
#endif /* CONFIG_FLASH_JESD216_API */
};

#define MEC5_QSPI_FC_IRQ_HANDLER_CFG(i) \
	static void flash_mec5_qspi_irq_config_##i(void) \
	{ \
		IRQ_CONNECT(DT_INST_IRQN(i), DT_INST_IRQ(i, priority), \
			    flash_mec5_qspi_isr, DEVICE_DT_INST_GET(i), 0); \
		irq_enable(DT_INST_IRQN(i)); \
	}

#define MEC5_QSPI_FLASH_FLAGS(nid) \
	.flags = (DT_PROP_OR(nid, use_dual, 0) << 0)

#define MEC5_QSPI_FLL(nid) \
	.layout = { \
		((DT_PROP_OR(nid, size, 0) / 8u) / DT_PROP_OR(nid, page_size, 256)), \
		DT_PROP_OR(nid, page_size, 256), \
	},

#define MEC5_QSPI_FLASH_LAYOUT(nid) \
	COND_CODE_1(CONFIG_FLASH_PAGE_LAYOUT, (MEC5_QSPI_FLL(nid)), ())

/* Create device node's and get properities for each SPI flash device
 * attached to this controller. MEC5 QSPI supports two chip selects.
 * Child reg address properity is the chip select number: 0 or 1.
 * nid = child node indentifier
 */
#define MEC5_QSPI_FLASH_DEV_INIT(nid) \
	static struct mec5_qspi_flash_cfg mec5_qf_cfg_##nid = { \
		.ctrl_dev = DEVICE_DT_GET(DT_PARENT(nid)), \
		.max_freq = DT_PROP_OR(nid, qspi_max_frequency, MHZ(12)), \
		.size_in_bytes = (DT_PROP_OR(nid, size, 0) / 8u), \
		.jedec_id = DT_PROP_OR(nid, jedec_id, 0), \
		.page_size = DT_PROP_OR(nid, page_size, 256), \
		.cs = DT_REG_ADDR(nid), \
		.quad_en_req = DT_ENUM_IDX_OR(nid, quad-enable-requirements, 4), \
		.enter_4byte_addr = DT_PROP_OR(nid, enter_4byte_addr, 0), \
		MEC5_QSPI_FLASH_FLAGS(nid), \
		MEC5_QSPI_FLASH_LAYOUT(nid) \
	}; \
	DEVICE_DT_DEFINE(nid, flash_mec5_init, NULL, NULL,                                         \
			 &mec5_qf_cfg_##nid, POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,              \
			 &flash_mec5_qspi_driver_api);

#define MEC5_QSPI_FLASH_CONTROLLER(i) \
	PINCTRL_DT_INST_DEFINE(i); \
	MEC5_QSPI_FC_IRQ_HANDLER_CFG(i) \
	static struct flash_mec5_qspi_devdata flash_mec5_qspi_devdata_##i; \
	static const struct flash_mec5_qspi_ctrl_devcfg flash_mec5_qspi_devcfg_##i = { \
		.regs = (struct mec_qspi_regs *)DT_INST_REG_ADDR(i), \
		.freqhz = DT_INST_PROP(i, qspi_frequency), \
		.signal_mode = DT_INST_ENUM_IDX_OR(i, signal_mode, 0), \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i), \
		.irq_config_func = flash_mec5_qspi_irq_config_##i, \
	}; \
	DEVICE_DT_INST_DEFINE(i, &flash_mec5_qspi_init, NULL, &flash_mec5_qspi_devdata_##i, \
			      &flash_mec5_qspi_devcfg_##i, POST_KERNEL, \
			      CONFIG_FLASH_INIT_PRIORITY, NULL); \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(i, MEC5_QSPI_FLASH_DEV_INIT);

DT_INST_FOREACH_STATUS_OKAY(MEC5_QSPI_FLASH_CONTROLLER)
