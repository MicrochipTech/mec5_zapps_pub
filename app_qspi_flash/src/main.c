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
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/zbus/zbus.h>

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#include <app_version.h>

/* MEC5 HAL */
#include <device_mec5.h>
#include <mec_qspi_api.h>

#define SPI_FLASH_RD_JEDEC_ID_OPCODE 0x9Fu

#define SPI_FLASH_WR_EN_OPCODE 0x06u
#define SPI_FLASH_VOLATILE_SR_WR_EN_OPCODE 0x50
#define SPI_FLASH_WR_DIS_OPCODE 0x04u

#define SPI_FLASH_RD_STS1_OPCODE 0x05u
#define SPI_FLASH_RD_STS2_OPCODE 0x35u
#define SPI_FLASH_RD_STS3_OPCODE 0x15u

#define SPI_FLASH_WR_STS1_OPCODE 0x01u
#define SPI_FLASH_WR_STS2_OPCODE 0x31u
#define SPI_FLASH_WR_STS3_OPCODE 0x11u

#define SPI_FLASH_RD_DATA_111_SLOW 0x03u
#define SPI_FLASH_RD_DATA_111      0x0bu
#define SPI_FLASH_RD_DATA_112      0x3bu
#define SPI_FLASH_RD_DATA_114      0x6bu

#define SPI_FLASH_PAGE_PROG_OPCODE 0x02u
#define SPI_FLASH_PAGE_SIZE        256u

#define SPI_FLASH_STATUS1_BUSY 0x01u
#define SPI_FLASH_STATUS1_WEL  0x02u
#define SPI_FLASH_STATUS1_BP0  0x04u
#define SPI_FLASH_STATUS1_BP1  0x08u
#define SPI_FLASH_STATUS1_BP2  0x10u
#define SPI_FLASH_STATUS1_TB   0x20u
#define SPI_FLASH_STATUS1_SEC  0x40u

#define SPI_FLASH_TEST_ADDR 0x102000u

#define SPI_FLASH0_NODE DT_ALIAS(spi_flash0)
#if DT_NODE_EXISTS(DT_NODELABEL(DT_ALIAS(spi_flash1)))
#define SPI_FLASH1_NODE DT_ALIAS(spi_flash1)
#endif

/* #define ZEPHYR_USER_NODE DT_PATH(zephyr_user) */

const struct device *spi_flash0_dev = DEVICE_DT_GET(SPI_FLASH0_NODE);
#ifdef SPI_FLASH1_NODE
const struct device *spi_flash1_dev = DEVICE_DT_GET(SPI_FLASH1_NODE);
#endif

/* ---- Global variables ---- */
static volatile uint32_t spin_val;
static volatile int ret_val;

#define TEST_BUF_SIZE 512

uint8_t buf1[TEST_BUF_SIZE];
uint8_t buf2[TEST_BUF_SIZE];
uint8_t buf3[TEST_BUF_SIZE];

struct mec_spi_command qcmd;
struct mec_qspi_context2 qctx2;

uint32_t spi_flash_jedec_id;
uint8_t spi_flash_status[3];

/* ---- prototypes ---- */
static void spin_on(uint32_t id, int rval);
static void fill_buf_incr_val(uint8_t *buf, uint32_t buflen);
static int test_qspi_flash_hal(uint8_t qid, enum mec_qspi_cs cs, struct mec_qspi_context2 *ctx,
				struct mec_spi_command *cmd, uint32_t spi_offset,
				uint32_t datalen, uint8_t *databuf);

int main(void)
{
	int ret = 0;
	off_t offset = 0;
	size_t len = 0;
	uint32_t saddr = 0, slen = 0;

	LOG_INF("app_qspi_flash sample: board: %s", DT_N_P_compatible_IDX_0);

	memset(buf1, 0x55, sizeof(buf1));
	memset(buf2, 0x55, sizeof(buf2));
	memset(buf3, 0x55, sizeof(buf3));

	LOG_INF("Read SPI flash JEDEC ID");

	/* SPI flash opcode 0x05 is 1-0-1 read 8-bit status with no tri-state clocks */
	qcmd.flags = MEC_SPI_CMD_ENCODE_PINS(1, 0, 1);
	qcmd.opcode = SPI_FLASH_RD_JEDEC_ID_OPCODE;
	qcmd.mode_byte = 0;
	qcmd.mode_bits = 0; /* no mode byte */
	qcmd.ts_clocks = 0;

	ret = test_qspi_flash_hal(0, MEC_QSPI_CS_0, &qctx2, &qcmd, 0, 4u,
				  (uint8_t *)&spi_flash_jedec_id);
	LOG_INF("QSPI flash HAL read test returned (%d)", ret);
	LOG_INF("JEDEC-ID = 0x%08x", spi_flash_jedec_id);

	/* SPI flash opcode 0x05 is 1-0-1 read 8-bit status with no tri-state clocks */
	LOG_INF("Read STATUS1");
	qcmd.flags = MEC_SPI_CMD_ENCODE_PINS(1, 0, 1);
	qcmd.opcode = SPI_FLASH_RD_STS1_OPCODE;
	qcmd.mode_byte = 0;
	qcmd.mode_bits = 0; /* no mode byte */
	qcmd.ts_clocks = 0;

	spi_flash_status[0] = 0xffu;
	ret = test_qspi_flash_hal(0, MEC_QSPI_CS_0, &qctx2, &qcmd, 0, 1, &spi_flash_status[0]);
	LOG_INF("QSPI flash HAL read test returned (%d)", ret);
	LOG_INF("SPI flash STATUS1 = 0x%02x", spi_flash_status[0]);

	/* SPI flash opcode 0x35 is 1-0-1 read 8-bit status 2 with no tri-state clocks */
	LOG_INF("Read STATUS2");
	qcmd.flags = MEC_SPI_CMD_ENCODE_PINS(1, 0, 1);
	qcmd.opcode = SPI_FLASH_RD_STS2_OPCODE;
	qcmd.mode_byte = 0;
	qcmd.mode_bits = 0; /* no mode byte */
	qcmd.ts_clocks = 0;

	spi_flash_status[1] = 0xffu;
	ret = test_qspi_flash_hal(0, MEC_QSPI_CS_0, &qctx2, &qcmd, 0, 1, &spi_flash_status[1]);
	LOG_INF("QSPI flash HAL read test returned (%d)", ret);
	LOG_INF("SPI flash STATUS2 = 0x%02x", spi_flash_status[1]);

	/* SPI flash opcode 0x15 is 1-0-1 read 8-bit status 3 with no tri-state clocks */
	LOG_INF("Read STATUS3");
	qcmd.flags = MEC_SPI_CMD_ENCODE_PINS(1, 0, 1);
	qcmd.opcode = SPI_FLASH_RD_STS3_OPCODE;
	qcmd.mode_byte = 0;
	qcmd.mode_bits = 0; /* no mode byte */
	qcmd.ts_clocks = 0;

	spi_flash_status[2] = 0xffu;
	ret = test_qspi_flash_hal(0, MEC_QSPI_CS_0, &qctx2, &qcmd, 0, 1, &spi_flash_status[2]);
	LOG_INF("QSPI flash HAL read test returned (%d)", ret);
	LOG_INF("SPI flash STATUS2 = 0x%02x", spi_flash_status[2]);

	/* SPI flash opcode 0x03 is 1-1-1 read with no tri-state clocks and freq < 33 MHz */
	LOG_INF("Read 1-1-1 Slow (no tri-state clocks)");
	qcmd.flags = MEC_SPI_CMD_ENCODE_PINS(1, 1, 1) | MEC_SPI_CMD_ADDR24;
	qcmd.opcode = SPI_FLASH_RD_DATA_111_SLOW;
	qcmd.mode_byte = 0;
	qcmd.mode_bits = 0; /* no mode byte */
	qcmd.ts_clocks = 0;

	saddr = SPI_FLASH_TEST_ADDR;
	slen = 16u;
	ret = test_qspi_flash_hal(0, MEC_QSPI_CS_0, &qctx2, &qcmd, saddr, slen, buf1);
	LOG_INF("QSPI flash HAL read test returned (%d)", ret);
	LOG_HEXDUMP_INF(buf1, slen, "read data");

	memset(buf1, 0x55, sizeof(buf1));

	/* SPI flash opcode 0x0B is 1-1-1 read with 8 tri-state clocks */
	LOG_INF("Read 1-1-1 8 ts-clocks");
	qcmd.flags = MEC_SPI_CMD_ENCODE_PINS(1, 1, 1) | MEC_SPI_CMD_ADDR24;
	qcmd.opcode = SPI_FLASH_RD_DATA_111;
	qcmd.mode_byte = 0;
	qcmd.mode_bits = 0; /* no mode byte */
	qcmd.ts_clocks = 8;

	saddr = SPI_FLASH_TEST_ADDR;
	slen = 16u;
	ret = test_qspi_flash_hal(0, MEC_QSPI_CS_0, &qctx2, &qcmd, saddr, slen, buf1);
	LOG_INF("QSPI flash HAL read test returned (%d)", ret);
	LOG_HEXDUMP_INF(buf1, slen, "read data");

	memset(buf1, 0x55, sizeof(buf1));

	/* SPI flash opcode 0x3B is 1-1-2 read with 8 tri-state clocks */
	LOG_INF("Read 1-1-2 8 ts-clocks");
	qcmd.flags = MEC_SPI_CMD_ENCODE_PINS(1, 1, 2) | MEC_SPI_CMD_ADDR24;
	qcmd.opcode = SPI_FLASH_RD_DATA_112;
	qcmd.mode_byte = 0;
	qcmd.mode_bits = 0; /* no mode byte */
	qcmd.ts_clocks = 8;

	saddr = SPI_FLASH_TEST_ADDR;
	slen = 16u;
	ret = test_qspi_flash_hal(0, MEC_QSPI_CS_0, &qctx2, &qcmd, saddr, slen, buf1);
	LOG_INF("QSPI flash HAL read test returned (%d)", ret);
	LOG_HEXDUMP_INF(buf1, slen, "read data");

	memset(buf1, 0x55, sizeof(buf1));

	/* SPI flash opcode 0x6B is 1-1-4 read with 8 tri-state clocks */
	LOG_INF("Read 1-1-4 8 ts-clocks");
	qcmd.flags = MEC_SPI_CMD_ENCODE_PINS(1, 1, 4) | MEC_SPI_CMD_ADDR24;
	qcmd.opcode = SPI_FLASH_RD_DATA_114;
	qcmd.mode_byte = 0;
	qcmd.mode_bits = 0; /* no mode byte */
	qcmd.ts_clocks = 8;

	saddr = SPI_FLASH_TEST_ADDR;
	slen = 16u;
	ret = test_qspi_flash_hal(0, MEC_QSPI_CS_0, &qctx2, &qcmd, saddr, slen, buf1);
	LOG_INF("QSPI flash HAL read test returned (%d)", ret);
	LOG_HEXDUMP_INF(buf1, slen, "read data");

	memset(buf1, 0x55, sizeof(buf1));
	fill_buf_incr_val(buf1, SPI_FLASH_PAGE_SIZE);

	/* Write Enable */
	LOG_INF("Send Write Enable");
	qcmd.flags = MEC_SPI_CMD_ENCODE_PINS(1, 0, 0);
	qcmd.opcode = SPI_FLASH_WR_EN_OPCODE;
	qcmd.mode_byte = 0;
	qcmd.mode_bits = 0; /* no mode byte */
	qcmd.ts_clocks = 0;

	ret = test_qspi_flash_hal(0, MEC_QSPI_CS_0, &qctx2, &qcmd, 0, 0, NULL);
	LOG_INF("QSPI flash HAL read test returned (%d)", ret);

	/* Page program: opcode, page address, data */
	LOG_INF("Send Page Program");
	qcmd.flags = (MEC_SPI_CMD_ENCODE_PINS(1, 1, 1) | MEC_SPI_CMD_ADDR24 |
		      MEC_SPI_CMD_DATA_DIR_TX);
	qcmd.opcode = SPI_FLASH_PAGE_PROG_OPCODE;
	qcmd.mode_byte = 0;
	qcmd.mode_bits = 0; /* no mode byte */
	qcmd.ts_clocks = 0;

	saddr = SPI_FLASH_TEST_ADDR;
	slen = SPI_FLASH_PAGE_SIZE;
	ret = test_qspi_flash_hal(0, MEC_QSPI_CS_0, &qctx2, &qcmd, saddr, slen, buf1);
	LOG_INF("QSPI flash HAL read test returned (%d)", ret);

	/* Loop reading STATUS1 until BUSY clears */
	do {
		k_sleep(K_MSEC(1));
		qcmd.flags = MEC_SPI_CMD_ENCODE_PINS(1, 0, 1);
		qcmd.opcode = SPI_FLASH_RD_STS1_OPCODE;
		qcmd.mode_byte = 0;
		qcmd.mode_bits = 0; /* no mode byte */
		qcmd.ts_clocks = 0;

		spi_flash_status[0] = 0xffu;
		ret = test_qspi_flash_hal(0, MEC_QSPI_CS_0, &qctx2, &qcmd, 0, 1,
					  &spi_flash_status[0]);
		if (ret) {
			LOG_ERR("QSPI HAL read STATUS1 error (%d)", ret);
			break;
		}
	} while (spi_flash_status[0] & SPI_FLASH_STATUS1_BUSY);

	LOG_INF("Read Page and check contents");
	qcmd.flags = MEC_SPI_CMD_ENCODE_PINS(1, 1, 4) | MEC_SPI_CMD_ADDR24;
	qcmd.opcode = SPI_FLASH_RD_DATA_114;
	qcmd.mode_byte = 0;
	qcmd.mode_bits = 0; /* no mode byte */
	qcmd.ts_clocks = 8;

	saddr = SPI_FLASH_TEST_ADDR;
	slen = SPI_FLASH_PAGE_SIZE;
	ret = test_qspi_flash_hal(0, MEC_QSPI_CS_0, &qctx2, &qcmd, saddr, slen, buf2);
	LOG_INF("QSPI flash HAL read test returned (%d)", ret);

	ret = memcmp(buf1, buf2, slen);
	if (ret == 0) {
		LOG_INF("Page read-back matches programmed data: PASS");
	} else {
		LOG_ERR("Page program sequence: FAIL");
	}

	LOG_INF("Read programmed Page using Flash driver");
	memset(buf2, 0x55, sizeof(buf2));

	offset = SPI_FLASH_TEST_ADDR;
	len = SPI_FLASH_PAGE_SIZE;

	ret = flash_read(spi_flash0_dev, offset, buf2, len);
	LOG_INF("flash read (0x%0lx, %u) returned (%d)", offset, len, ret);

	ret = memcmp(buf1, buf2, slen);
	if (ret == 0) {
		LOG_INF("Page read-back using Flash driver matches programmed data: PASS");
	} else {
		LOG_ERR("Page read-back using Flash driver: FAIL");
	}

	LOG_INF("Application Done (%d)", ret);
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

static void fill_buf_incr_val(uint8_t *buf, uint32_t buflen)
{
	for (uint32_t n = 0; n < buflen; n++) {
		buf[n] = n % 256u;
	}
}

static int test_qspi_flash_hal(uint8_t qid, enum mec_qspi_cs cs, struct mec_qspi_context2 *ctx,
				struct mec_spi_command *cmd, uint32_t spi_offset,
				uint32_t datalen, uint8_t *databuf)
{
	struct mec_qspi_regs *qregs = NULL;
	uint32_t xflags = 0, spin_count = 0;
	int ret = 0;
	bool has_data = false;

	if (!ctx || !cmd) {
		LOG_ERR("Read specified but databuf is NULL");
		return -EINVAL;
	}

	if (MEC_SPI_CMD_DATA_NPINS(cmd->flags)) {
		if (!datalen) {
			LOG_ERR("Command has data phase but datalen is 0");
			return -EINVAL;
		}
		has_data = true;
	}

	qregs = mec_hal_qspi_regs_from_id(qid);
	if (qregs == NULL) {
		LOG_ERR("QSPI regs from id HAL API returned error (%d)", ret);
		return -EIO;
	}

	/* Initialize struct mec_qspi_context2 for the selected QSPI controller */
	ret = mec_hal_qspi_context2_init(ctx, qid);
	if (ret != MEC_RET_OK) {
		LOG_ERR("QSPI context 2 init HAL API returned error (%d)", ret);
		return -EIO;
	}

	/* Configure the chip select */
	ret = mec_hal_qspi_cs_select(qregs, cs);
	if (ret != MEC_RET_OK) {
		LOG_ERR("QSPI cs select HAL API returned error (%d)", ret);
		return -EIO;
	}

	/* Configure the SPI flash command specified in struct mec_spi_command and SPI address */
	ret = mec_hal_qspi_flash_cmd_setup(ctx, cmd, spi_offset);
	if (ret != MEC_RET_OK) {
		LOG_ERR("QSPI flash command setup HAL API returned error (%d)", ret);
		return -EIO;
	}

	/* Configure the flash data phase */
	if (has_data) {
		xflags = MEC_QSPI_FLASH_DATA_SETUP_DIR_RX;
		if (cmd->flags & MEC_SPI_CMD_DATA_DIR_TX) {
			xflags = MEC_QSPI_FLASH_DATA_SETUP_DIR_TX;
		}
		ret = mec_hal_qspi_flash_data_setup(ctx, cmd, databuf, datalen, xflags);
		if (ret != MEC_RET_OK) {
			LOG_ERR("QSPI flash data setup HAL API returned error (%d)", ret);
			return -EIO;
		}
	}

	/* Load the configuration into the QSPI controller */
	ret = mec_hal_qspi_flash_xfr_load(ctx);
	if (ret != MEC_RET_OK) {
		LOG_ERR("QSPI flash xfr load HAL API returned error (%d)", ret);
		return -EIO;
	}

	/* Start the QSPI hardware */
	ret = mec_hal_qspi_start(qregs, 0);
	if (ret != MEC_RET_OK) {
		LOG_ERR("QSPI start HAL API returned error (%d)", ret);
		return -EIO;
	}

	/* Poll for done */
	ret = mec_hal_qspi_done(qregs);
	while (ret != MEC_RET_OK) {
		spin_count++;
		ret = mec_hal_qspi_done(qregs);
	}

	LOG_INF("QSPI spin loop count = %u", spin_count);

	return 0;
}
