#include "qspi_nvparams.h"
#include "hardware_bridge.h"

#include <string.h>

#if defined(ZYNQ) && defined(ZYBO_Z720)

#include "xqspips.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "xstatus.h"
#include "sys.h"

#define QSPI_NV_CMD_OFF     0
#define QSPI_NV_ADDR1_OFF   1
#define QSPI_NV_ADDR2_OFF   2
#define QSPI_NV_ADDR3_OFF   3
#define QSPI_NV_DATA_OFF    4

#define QSPI_NV_WIP         0x01u
#define QSPI_NV_WIP_MS      8000

/* Spansion/Infineon S25FL128S; density 0x18 = 128 Mbit / 16 MiB. */
#define QSPI_NV_JEDEC_SPANSION_MFG  0x01u
#define QSPI_NV_JEDEC_SPANSION_TYPE 0x20u
#define QSPI_NV_JEDEC_DENSITY_16MIB 0x18u

static XQspiPs s_qspi;
static int s_ready;
static u32 s_store_offset = QSPI_NV_SECTOR_OFFSET;
static int s_slot_known;

static u8 s_tx[QSPI_NV_PAGE_BYTES + QSPI_NV_DATA_OFF] __attribute__((aligned(4)));
static u8 s_rx[QSPI_NV_PAGE_BYTES + QSPI_NV_DATA_OFF] __attribute__((aligned(4)));

static u32 crc32_ieee(const u8 *data, unsigned int len)
{
	u32 crc = 0xFFFFFFFFu;
	unsigned int i;
	int b;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (b = 0; b < 8; b++) {
			if (crc & 1u) {
				crc = (crc >> 1) ^ 0xEDB88320u;
			} else {
				crc >>= 1;
			}
		}
	}
	return crc ^ 0xFFFFFFFFu;
}

static void put_u32_le(u8 *p, u32 v)
{
	p[0] = (u8)(v);
	p[1] = (u8)(v >> 8);
	p[2] = (u8)(v >> 16);
	p[3] = (u8)(v >> 24);
}

static u32 get_u32_le(const u8 *p)
{
	return (u32)p[0]
	     | ((u32)p[1] << 8)
	     | ((u32)p[2] << 16)
	     | ((u32)p[3] << 24);
}

static int calibration_is_sane(float hanging, float angle_360)
{
	if (hanging != hanging || angle_360 != angle_360) {
		return 0;
	}
	/* Physical full-circle spans are close to the 12-bit ADC range. */
	if (angle_360 < 2048.0f || angle_360 > 8192.0f) {
		return 0;
	}
	if (hanging < 0.0f || hanging >= angle_360) {
		return 0;
	}
	return 1;
}

static void pack_record(u8 *page, float hanging, float angle_360)
{
	memset(page, 0xFF, QSPI_NV_PAGE_BYTES);
	put_u32_le(&page[0], QSPI_NV_MAGIC);
	put_u32_le(&page[4], QSPI_NV_VERSION);
	memcpy(&page[8], &hanging, 4);
	memcpy(&page[12], &angle_360, 4);
	put_u32_le(&page[16], crc32_ieee(page, 16));
}

static int unpack_record(const u8 *page, float *hanging_out, float *angle_360_out)
{
	u32 magic;
	u32 version;
	u32 crc_got;
	u32 crc_exp;
	float hanging;
	float angle_360;

	magic = get_u32_le(&page[0]);
	version = get_u32_le(&page[4]);
	memcpy(&hanging, &page[8], 4);
	memcpy(&angle_360, &page[12], 4);
	crc_got = get_u32_le(&page[16]);
	crc_exp = crc32_ieee(page, 16);

	if (magic != QSPI_NV_MAGIC || version != QSPI_NV_VERSION) {
		return -1;
	}
	if (crc_got != crc_exp) {
		return -1;
	}
	if (!calibration_is_sane(hanging, angle_360)) {
		return -1;
	}
	*hanging_out = hanging;
	*angle_360_out = angle_360;
	return 0;
}

static int flash_xfer(u8 *tx, u8 *rx, u32 nbytes)
{
	return XQspiPs_PolledTransfer(&s_qspi, tx, rx, nbytes);
}

static int flash_wren(void)
{
	s_tx[0] = XQSPIPS_FLASH_OPCODE_WREN;
	return flash_xfer(s_tx, s_rx, 1);
}

static int flash_wait_ready(void)
{
	int i;

	s_tx[0] = XQSPIPS_FLASH_OPCODE_RDSR1;
	s_tx[1] = 0;
	for (i = 0; i < QSPI_NV_WIP_MS; i++) {
		if (flash_xfer(s_tx, s_rx, 2) != XST_SUCCESS) {
			return -1;
		}
		if ((s_rx[1] & QSPI_NV_WIP) == 0u) {
			return 0;
		}
		Sleep_ms(1);
	}
	return -1;
}

static int flash_unprotect(void)
{
	if (flash_wren() != XST_SUCCESS) {
		return -1;
	}
	s_tx[0] = XQSPIPS_FLASH_OPCODE_WRSR;
	s_tx[1] = 0;
	if (flash_xfer(s_tx, s_rx, 2) != XST_SUCCESS) {
		return -1;
	}
	return flash_wait_ready();
}

static u8 erase_opcode_for_offset(u32 offset)
{
	if (offset == QSPI_NV_SUBSECTOR_OFF) {
		return XQSPIPS_FLASH_OPCODE_BE_4K;
	}
	return XQSPIPS_FLASH_OPCODE_SE;
}

static int flash_erase(u32 offset, u8 opcode)
{
	if (flash_wren() != XST_SUCCESS) {
		return -1;
	}
	s_tx[QSPI_NV_CMD_OFF] = opcode;
	s_tx[QSPI_NV_ADDR1_OFF] = (u8)((offset >> 16) & 0xFFu);
	s_tx[QSPI_NV_ADDR2_OFF] = (u8)((offset >> 8) & 0xFFu);
	s_tx[QSPI_NV_ADDR3_OFF] = (u8)(offset & 0xFFu);
	if (flash_xfer(s_tx, s_rx, 4) != XST_SUCCESS) {
		return -1;
	}
	return flash_wait_ready();
}

static int flash_read(u32 offset, u8 *dst, u32 nbytes)
{
	u32 i;

	memset(s_tx, 0, QSPI_NV_DATA_OFF + nbytes);
	s_tx[QSPI_NV_CMD_OFF] = XQSPIPS_FLASH_OPCODE_NORM_READ;
	s_tx[QSPI_NV_ADDR1_OFF] = (u8)((offset >> 16) & 0xFFu);
	s_tx[QSPI_NV_ADDR2_OFF] = (u8)((offset >> 8) & 0xFFu);
	s_tx[QSPI_NV_ADDR3_OFF] = (u8)(offset & 0xFFu);
	if (flash_xfer(s_tx, s_rx, QSPI_NV_DATA_OFF + nbytes) != XST_SUCCESS) {
		return -1;
	}
	for (i = 0; i < nbytes; i++) {
		dst[i] = s_rx[QSPI_NV_DATA_OFF + i];
	}
	return 0;
}

static int flash_program_page(u32 offset, const u8 *page)
{
	if (flash_wren() != XST_SUCCESS) {
		return -1;
	}
	s_tx[QSPI_NV_CMD_OFF] = XQSPIPS_FLASH_OPCODE_PP;
	s_tx[QSPI_NV_ADDR1_OFF] = (u8)((offset >> 16) & 0xFFu);
	s_tx[QSPI_NV_ADDR2_OFF] = (u8)((offset >> 8) & 0xFFu);
	s_tx[QSPI_NV_ADDR3_OFF] = (u8)(offset & 0xFFu);
	memcpy(&s_tx[QSPI_NV_DATA_OFF], page, QSPI_NV_PAGE_BYTES);
	if (flash_xfer(s_tx, s_rx, QSPI_NV_DATA_OFF + QSPI_NV_PAGE_BYTES) != XST_SUCCESS) {
		return -1;
	}
	return flash_wait_ready();
}

static int flash_write_record(u32 offset, u8 erase_opcode, const u8 *page)
{
	u8 verify[QSPI_NV_RECORD_BYTES];

	if (flash_erase(offset, erase_opcode) != 0) {
		return -1;
	}
	if (flash_program_page(offset, page) != 0) {
		return -1;
	}
	if (flash_read(offset, verify, QSPI_NV_RECORD_BYTES) != 0) {
		return -1;
	}
	if (memcmp(verify, page, QSPI_NV_RECORD_BYTES) != 0) {
		return -1;
	}
	return 0;
}

int QspiNv_Init(void)
{
	XQspiPs_Config *cfg;
	u32 options;
	u8 mfg;
	u8 type;
	u8 density;

	s_ready = 0;
	s_store_offset = QSPI_NV_SECTOR_OFFSET;
	s_slot_known = 0;

#ifndef XPAR_XQSPIPS_0_DEVICE_ID
	xil_printf("QSPI angle calibration: no XQspiPs in BSP\r\n");
	return -1;
#else
	cfg = XQspiPs_LookupConfig(XPAR_XQSPIPS_0_DEVICE_ID);
	if (cfg == NULL) {
		xil_printf("QSPI angle calibration: lookup failed\r\n");
		return -1;
	}
	if (XQspiPs_CfgInitialize(&s_qspi, cfg, cfg->BaseAddress) != XST_SUCCESS) {
		xil_printf("QSPI angle calibration: init failed\r\n");
		return -1;
	}

	/* Leave linear (LQSPI) mode used by FSBL; IO mode for erase/program. */
	options = XQSPIPS_FORCE_SSELECT_OPTION | XQSPIPS_HOLD_B_DRIVE_OPTION;
	if (XQspiPs_SetOptions(&s_qspi, options) != XST_SUCCESS) {
		xil_printf("QSPI angle calibration: set options failed\r\n");
		return -1;
	}
	if (XQspiPs_SetClkPrescaler(&s_qspi, XQSPIPS_CLK_PRESCALE_8) != XST_SUCCESS) {
		xil_printf("QSPI angle calibration: prescaler failed\r\n");
		return -1;
	}
	XQspiPs_SetSlaveSelect(&s_qspi);

	memset(s_tx, 0, 4);
	s_tx[0] = XQSPIPS_FLASH_OPCODE_RDID;
	if (flash_xfer(s_tx, s_rx, 4) != XST_SUCCESS) {
		xil_printf("QSPI angle calibration: JEDEC read failed\r\n");
		return -1;
	}
	mfg = s_rx[1];
	type = s_rx[2];
	density = s_rx[3];
	if (density != QSPI_NV_JEDEC_DENSITY_16MIB) {
		xil_printf("QSPI angle calibration: unexpected JEDEC %02x %02x %02x\r\n",
			   mfg, type, density);
		return -1;
	}
	if (mfg != QSPI_NV_JEDEC_SPANSION_MFG || type != QSPI_NV_JEDEC_SPANSION_TYPE) {
		xil_printf("QSPI angle calibration: JEDEC %02x %02x %02x (not S25FL128S, 16MiB OK)\r\n",
			   mfg, type, density);
	} else {
		xil_printf("QSPI angle calibration: S25FL128S ready\r\n");
	}

	if (flash_unprotect() != 0) {
		xil_printf("QSPI angle calibration: WRSR unprotect failed\r\n");
	}

	s_ready = 1;
	return 0;
#endif
}

int QspiNv_LoadCalibration(float *hanging_out, float *angle_360_out)
{
	u8 page[QSPI_NV_RECORD_BYTES];
	float hanging;
	float angle_360;

	if (hanging_out == NULL || angle_360_out == NULL || !s_ready) {
		return -1;
	}

	if (flash_read(QSPI_NV_SECTOR_OFFSET, page, QSPI_NV_RECORD_BYTES) == 0
	    && unpack_record(page, &hanging, &angle_360) == 0) {
		s_store_offset = QSPI_NV_SECTOR_OFFSET;
		s_slot_known = 1;
		*hanging_out = hanging;
		*angle_360_out = angle_360;
		return 0;
	}
	if (flash_read(QSPI_NV_SUBSECTOR_OFF, page, QSPI_NV_RECORD_BYTES) == 0
	    && unpack_record(page, &hanging, &angle_360) == 0) {
		s_store_offset = QSPI_NV_SUBSECTOR_OFF;
		s_slot_known = 1;
		*hanging_out = hanging;
		*angle_360_out = angle_360;
		return 0;
	}
	return -1;
}

int QspiNv_SaveCalibration(float hanging, float angle_360)
{
	u8 page[QSPI_NV_PAGE_BYTES];

	if (!s_ready || !calibration_is_sane(hanging, angle_360)) {
		return -1;
	}
	pack_record(page, hanging, angle_360);

	if (s_slot_known) {
		if (flash_write_record(s_store_offset,
				       erase_opcode_for_offset(s_store_offset),
				       page) == 0) {
			return 0;
		}
		xil_printf("QSPI angle calibration: known slot failed, retrying both\r\n");
	}

	if (flash_write_record(QSPI_NV_SECTOR_OFFSET, XQSPIPS_FLASH_OPCODE_SE, page) == 0) {
		s_store_offset = QSPI_NV_SECTOR_OFFSET;
		s_slot_known = 1;
		return 0;
	}
	xil_printf("QSPI angle calibration: 64KiB erase failed, trying 4KiB\r\n");
	if (flash_write_record(QSPI_NV_SUBSECTOR_OFF, XQSPIPS_FLASH_OPCODE_BE_4K, page) == 0) {
		s_store_offset = QSPI_NV_SUBSECTOR_OFF;
		s_slot_known = 1;
		return 0;
	}
	return -1;
}

#else /* !ZYBO_Z720 */

#ifdef ZYNQ
#include "xil_printf.h"
#endif

int QspiNv_Init(void)
{
#ifdef ZYNQ
	xil_printf("QSPI angle calibration: not enabled on this board\r\n");
#endif
	return -1;
}

int QspiNv_LoadCalibration(float *hanging_out, float *angle_360_out)
{
	(void)hanging_out;
	(void)angle_360_out;
	return -1;
}

int QspiNv_SaveCalibration(float hanging, float angle_360)
{
	(void)hanging;
	(void)angle_360;
	return -1;
}

#endif
